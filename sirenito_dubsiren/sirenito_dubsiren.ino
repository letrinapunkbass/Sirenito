/*
  SPDX-License-Identifier: MIT

  Sirenito - Dub Siren ESP32-S3 N16R8 + PCM5102A + USB-MIDI

  - Sintetizador autonomo con salida I2S.
  - Controlador USB-MIDI simultaneo.
  - Lectura de potenciometros estabilizada mediante:
      * mediana de 7 muestras,
      * filtro EMA,
      * zona muerta de entrada,
      * ajuste limpio de extremos,
      * histeresis y limitacion temporal MIDI.
  - Portadoras: seno, sierra PolyBLEP, cuadrada PolyBLEP y laser.
  - LFO: seno, sierra, cuadrada y apagado.
  - Delay mono de hasta 1,2 s con bypass real.

  Arduino IDE para ESP32-S3:
  - USB Mode: USB-OTG (TinyUSB)
  - USB CDC On Boot: Enabled (opcional)
  - Flash Size: 16 MB
  - PSRAM: OPI PSRAM
*/

#include <Arduino.h>
#include <ESP_I2S.h>
#include "USB.h"
#include "USBMIDI.h"
#include <math.h>

// ============================================================================
// CONFIGURACION GENERAL
// ============================================================================

#define SAMPLE_RATE 44100

// Canal MIDI configurable: valores validos de 1 a 16.
constexpr uint8_t MIDI_CHANNEL = 1;
static_assert(MIDI_CHANNEL >= 1 && MIDI_CHANNEL <= 16,
              "MIDI_CHANNEL debe estar entre 1 y 16");

constexpr uint8_t MIDI_TRIGGER_NOTE = 60;      // C4
constexpr uint8_t MIDI_TRIGGER_VELOCITY = 127;

// Mapa MIDI CC.
constexpr uint8_t CC_PITCH      = 20;
constexpr uint8_t CC_RATE       = 21;
constexpr uint8_t CC_DEPTH      = 22;
constexpr uint8_t CC_DECAY      = 23;
constexpr uint8_t CC_DELAY_TIME = 24;
constexpr uint8_t CC_DELAY_FB   = 25;
constexpr uint8_t CC_SWEEP      = 26;
constexpr uint8_t CC_VOLUME     = 27;
constexpr uint8_t CC_WAVEFORM   = 28;
constexpr uint8_t CC_LFO_MODE   = 29;

I2SClass I2S;
USBMIDI MIDI("sirenito");

// ============================================================================
// PINOUT DEFINITIVO
// ============================================================================

// Selector de portadora: cada posicion cierra a GND.
#define PIN_WAVE_SINE      4
#define PIN_WAVE_LASER     7
#define PIN_WAVE_SQUARE    6
#define PIN_WAVE_SAW       5

// Potenciometros lineales de 10k: wiper al GPIO, extremos a 3V3 y GND.
#define PIN_PITCH         15
#define PIN_DECAY         16
#define PIN_DEPTH         17
#define PIN_RATE          18

// Selector de LFO: cada posicion cierra a GND.
#define PIN_LFO_SINE       9
#define PIN_LFO_SAW       10
#define PIN_LFO_SQUARE    11
#define PIN_LFO_OFF       12

#define PIN_SWEEP          1
#define PIN_DELAY_FB      14
#define PIN_DELAY_TIME    13
#define PIN_VOLUME         2

#define PIN_BTN_TRIG      42
#define PIN_LED_TRIG      38

#define I2S_DOUT          41
#define I2S_WS            40
#define I2S_BCK           39

// GPIO19 y GPIO20 quedan libres para USB D- y D+.
// GPIO39-42 son pines JTAG en el ESP32-S3; este pinout usa 39-41 para I2S
// y 42 para el trigger, por lo que el depurador JTAG queda deshabilitado.
// GPIO38 puede coincidir con el LED RGB integrado de algunas DevKitC-1.

// ============================================================================
// TIPOS Y ESTADO COMPARTIDO
// ============================================================================

enum Waveform : uint8_t { SINE, SAW, SQUARE, LASER };
enum LfoMode  : uint8_t { LFO_SINE, LFO_SAW, LFO_SQUARE, LFO_OFF };

struct ControlState {
  float pitch;
  float rate;
  float depth;
  float decay;
  float delayTime;
  float delayFeedback;
  float sweep;
  float volume;
  bool trigger;
  Waveform carrier;
  LfoMode lfo;
};

ControlState controls = {
  0.5f, 0.5f, 0.5f, 0.5f,
  0.35f, 0.45f, 0.5f, 0.8f,
  false, SINE, LFO_SINE
};

portMUX_TYPE controlsMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t audioTaskHandle = nullptr;

// ============================================================================
// CONSTANTES DSP Y FILTRADO
// ============================================================================

constexpr int SINE_TABLE_SIZE = 1024;
float sineTable[SINE_TABLE_SIZE];

// Delay mono: 1,2 segundos a 44,1 kHz.
constexpr int DELAY_MAX = 52920;
int16_t delayBuffer[DELAY_MAX];

constexpr float DELAY_BYPASS_THRESHOLD = 0.01f;
constexpr float MAX_FEEDBACK = 0.92f;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 6;

// Filtrado de potenciometros.
constexpr uint8_t POT_MEDIAN_SAMPLES = 7;
constexpr float POT_FILTER_ALPHA = 0.10f;
constexpr float POT_INPUT_DEADBAND = 0.0015f;
constexpr float POT_END_DEADBAND = 0.004f;
constexpr uint32_t CONTROL_READ_INTERVAL_MS = 20;  // 50 Hz

// Filtrado adicional de la salida MIDI.
constexpr uint8_t MIDI_CC_HYSTERESIS = 2;
constexpr uint32_t MIDI_CC_MIN_INTERVAL_MS = 12;

// ============================================================================
// UTILIDADES DSP
// ============================================================================

static inline float clamp01(float value) {
  return fminf(1.0f, fmaxf(0.0f, value));
}

static inline float softClip(float value) {
  if (value >= 1.0f) return 1.0f;
  if (value <= -1.0f) return -1.0f;
  return value * (1.5f - 0.5f * value * value);
}

static inline float polyBlep(float phase, float phaseIncrement) {
  if (phaseIncrement <= 0.0f) return 0.0f;

  if (phase < phaseIncrement) {
    float t = phase / phaseIncrement;
    return t + t - t * t - 1.0f;
  }

  if (phase > 1.0f - phaseIncrement) {
    float t = (phase - 1.0f) / phaseIncrement;
    return t * t + t + t + 1.0f;
  }

  return 0.0f;
}

static inline float bandLimitedSaw(float phase, float phaseIncrement) {
  float dt = fminf(phaseIncrement, 0.49f);
  float output = 1.0f - 2.0f * phase;
  output -= polyBlep(phase, dt);
  return output;
}

static inline float bandLimitedSquare(float phase, float phaseIncrement) {
  float dt = fminf(phaseIncrement, 0.49f);
  float output = phase < 0.5f ? 1.0f : -1.0f;
  output += polyBlep(phase, dt);

  float shiftedPhase = phase + 0.5f;
  if (shiftedPhase >= 1.0f) shiftedPhase -= 1.0f;
  output -= polyBlep(shiftedPhase, dt);
  return output;
}

static void initSineTable() {
  for (int i = 0; i < SINE_TABLE_SIZE; ++i) {
    sineTable[i] = sinf(TWO_PI * (float)i / (float)SINE_TABLE_SIZE);
  }
}

static inline float genSineLUT(float phase) {
  float position = phase * SINE_TABLE_SIZE;
  int firstIndex = (int)position;
  if (firstIndex >= SINE_TABLE_SIZE) firstIndex = 0;

  int secondIndex = firstIndex + 1;
  if (secondIndex >= SINE_TABLE_SIZE) secondIndex = 0;

  float fraction = position - (float)firstIndex;
  return sineTable[firstIndex] +
         (sineTable[secondIndex] - sineTable[firstIndex]) * fraction;
}

static inline float genCarrier(Waveform waveform,
                               float phase,
                               float phaseIncrement) {
  switch (waveform) {
    case SINE:
      return genSineLUT(phase);
    case SAW:
      return bandLimitedSaw(phase, phaseIncrement);
    case SQUARE:
    case LASER:
      return bandLimitedSquare(phase, phaseIncrement);
  }
  return 0.0f;
}

// Curva exponencial normalizada: 1 al inicio y exactamente 0 al final.
static inline float normalizedExpDecay(float progress) {
  progress = clamp01(progress);
  constexpr float END_VALUE = 0.0183156389f;  // exp(-4)
  float exponential = expf(-4.0f * progress);
  return (exponential - END_VALUE) / (1.0f - END_VALUE);
}

static inline float centeredSweep(float potValue) {
  float centered = potValue - 0.5f;
  constexpr float CENTER_DEADBAND = 0.025f;

  if (fabsf(centered) <= CENTER_DEADBAND) return 0.0f;

  float magnitude =
      (fabsf(centered) - CENTER_DEADBAND) /
      (0.5f - CENTER_DEADBAND);

  return copysignf(clamp01(magnitude), centered) * 1200.0f;
}

// ============================================================================
// TAREA DE AUDIO
// ============================================================================

void AudioTask(void *pvParameters) {
  constexpr int FRAMES_PER_BLOCK = 256;
  int16_t outputBuffer[FRAMES_PER_BLOCK * 2];

  float carrierPhase = 0.0f;
  float lfoPhase = 0.0f;
  float currentAmplitude = 0.0f;
  bool lastTrigger = false;

  float sweepCounter = 0.0f;
  float sweepSamples = 1.0f;

  uint32_t laserPeriodCounter = 1000000000UL;
  uint32_t laserShotCounter = 0;
  bool laserShotActive = false;
  float laserSweepSamples = 1.0f;

  int delayWriteIndex = 0;
  float smoothedDelaySamples = 1.0f;
  float delayWet = 0.0f;
  float feedbackLowPass = 0.0f;
  float smoothedVolume = 0.0f;

  constexpr float DELAY_TIME_SMOOTH = 0.00035f;
  constexpr float DELAY_WET_SMOOTH = 0.0015f;
  constexpr float FEEDBACK_LP_COEFFICIENT = 0.16f;
  constexpr float VOLUME_SMOOTH = 0.0015f;

  while (true) {
    ControlState local;

    portENTER_CRITICAL(&controlsMux);
    local = controls;
    portEXIT_CRITICAL(&controlsMux);

    const float baseFrequency = 80.0f + local.pitch * 1120.0f;
    const float lfoFrequency = 1.0f + local.rate * 11.0f;
    const float sweepOffset = centeredSweep(local.sweep);
    const float decayTimeSeconds = 0.05f + local.decay * 3.95f;
    const float decaySamples = decayTimeSeconds * SAMPLE_RATE;
    const float attackSamples = 0.005f * SAMPLE_RATE;

    const bool delayEnabled =
        local.delayTime > DELAY_BYPASS_THRESHOLD;
    const float delayWetTarget = delayEnabled ? 1.0f : 0.0f;

    float normalizedDelay = 0.0f;
    if (delayEnabled) {
      normalizedDelay =
          (local.delayTime - DELAY_BYPASS_THRESHOLD) /
          (1.0f - DELAY_BYPASS_THRESHOLD);
    }
    normalizedDelay = clamp01(normalizedDelay);

    const float targetDelaySamples =
        1.0f + normalizedDelay * (DELAY_MAX - 2.0f);

    for (int frame = 0; frame < FRAMES_PER_BLOCK; ++frame) {
      // Envolvente tipo gate para seno, sierra y cuadrada.
      if (local.trigger) {
        currentAmplitude += 1.0f / attackSamples;
        if (currentAmplitude > 1.0f) currentAmplitude = 1.0f;

        if (!lastTrigger) {
          sweepSamples =
              fmaxf(1.0f, decayTimeSeconds * 0.8f * SAMPLE_RATE);
          sweepCounter = 0.0f;
          laserPeriodCounter = 1000000000UL;
        }
      } else {
        currentAmplitude -= 1.0f / decaySamples;
        if (currentAmplitude < 0.0f) currentAmplitude = 0.0f;
      }
      lastTrigger = local.trigger;

      float currentSweep = 0.0f;
      if (sweepCounter < sweepSamples) {
        float progress = sweepCounter / sweepSamples;
        currentSweep = sweepOffset * normalizedExpDecay(progress);
        sweepCounter += 1.0f;
      }

      float sample = 0.0f;

      if (local.carrier == LASER) {
        if (local.trigger) {
          const float laserRate = 1.0f + local.rate * 11.0f;
          const uint32_t samplesPerShot =
              (uint32_t)fmaxf(1.0f, SAMPLE_RATE / laserRate);

          ++laserPeriodCounter;
          if (laserPeriodCounter >= samplesPerShot) {
            laserPeriodCounter = 0;
            laserShotCounter = 0;
            laserShotActive = true;

            // En laser, DECAY controla la velocidad del sweep.
            laserSweepSamples =
                fmaxf(1.0f, decayTimeSeconds * SAMPLE_RATE);
          }

          if (laserShotActive) {
            const float shotDurationSeconds =
                fminf(0.25f, 0.75f / laserRate);
            const uint32_t shotDurationSamples =
                (uint32_t)fmaxf(
                    1.0f, shotDurationSeconds * SAMPLE_RATE);

            if (laserShotCounter < shotDurationSamples) {
              float shotProgress =
                  (float)laserShotCounter /
                  (float)shotDurationSamples;

              float shotAmplitude;
              if (laserShotCounter < (uint32_t)attackSamples) {
                shotAmplitude =
                    (float)laserShotCounter / attackSamples;
              } else {
                shotAmplitude = normalizedExpDecay(shotProgress);
              }

              float shotSweepProgress =
                  (float)laserShotCounter / laserSweepSamples;
              float shotSweep = 0.0f;

              if (shotSweepProgress < 1.0f) {
                shotSweep =
                    sweepOffset * normalizedExpDecay(shotSweepProgress);
              }

              float frequency =
                  fmaxf(20.0f, baseFrequency + shotSweep);
              float phaseIncrement =
                  fminf(frequency / SAMPLE_RATE, 0.49f);

              carrierPhase += phaseIncrement;
              if (carrierPhase >= 1.0f) carrierPhase -= 1.0f;

              sample =
                  bandLimitedSquare(carrierPhase, phaseIncrement) *
                  shotAmplitude;

              ++laserShotCounter;
            } else {
              laserShotActive = false;
            }
          }
        } else {
          // El laser se detiene inmediatamente al soltar el trigger.
          laserShotActive = false;
          laserPeriodCounter = 1000000000UL;
          sample = 0.0f;
        }
      } else {
        float lfoValue = 0.0f;

        if (local.lfo != LFO_OFF) {
          lfoPhase += lfoFrequency / SAMPLE_RATE;
          if (lfoPhase >= 1.0f) lfoPhase -= 1.0f;

          switch (local.lfo) {
            case LFO_SINE:
              lfoValue = genSineLUT(lfoPhase);
              break;
            case LFO_SAW:
              lfoValue = 1.0f - 2.0f * lfoPhase;
              break;
            case LFO_SQUARE:
              lfoValue = lfoPhase < 0.5f ? 1.0f : -1.0f;
              break;
            case LFO_OFF:
              break;
          }
        }

        float frequency =
            baseFrequency *
                (1.0f + local.depth * lfoValue * 0.8f) +
            currentSweep;

        frequency = fmaxf(20.0f, frequency);

        float phaseIncrement =
            fminf(frequency / SAMPLE_RATE, 0.49f);

        carrierPhase += phaseIncrement;
        if (carrierPhase >= 1.0f) carrierPhase -= 1.0f;

        sample =
            genCarrier(local.carrier, carrierPhase, phaseIncrement) *
            currentAmplitude;
      }

      // Delay mono con tiempo suavizado.
      smoothedDelaySamples +=
          (targetDelaySamples - smoothedDelaySamples) *
          DELAY_TIME_SMOOTH;

      smoothedDelaySamples =
          fminf(DELAY_MAX - 1.0f,
                fmaxf(1.0f, smoothedDelaySamples));

      // Crossfade de entrada/salida del bypass.
      delayWet +=
          (delayWetTarget - delayWet) * DELAY_WET_SMOOTH;

      float readIndexFloat =
          (float)delayWriteIndex - smoothedDelaySamples;

      if (readIndexFloat < 0.0f) readIndexFloat += DELAY_MAX;

      int firstIndex = (int)readIndexFloat;
      int secondIndex = firstIndex + 1;
      if (secondIndex >= DELAY_MAX) secondIndex = 0;

      float fraction = readIndexFloat - (float)firstIndex;
      float firstSample = (float)delayBuffer[firstIndex];
      float secondSample = (float)delayBuffer[secondIndex];

      float delayed =
          (firstSample +
           (secondSample - firstSample) * fraction) /
          32768.0f;

      float activeDelayed = delayed * delayWet;

      feedbackLowPass +=
          (activeDelayed - feedbackLowPass) *
          FEEDBACK_LP_COEFFICIENT;

      float delayInput =
          sample + feedbackLowPass * local.delayFeedback;

      float storedSample = softClip(delayInput * 0.90f);
      delayBuffer[delayWriteIndex] =
          (int16_t)lrintf(storedSample * 32767.0f);

      ++delayWriteIndex;
      if (delayWriteIndex >= DELAY_MAX) delayWriteIndex = 0;

      float output = sample * 0.85f + activeDelayed * 0.45f;
      output = softClip(output * 1.35f);

      smoothedVolume +=
          (local.volume - smoothedVolume) * VOLUME_SMOOTH;
      output *= smoothedVolume;

      int32_t quantized =
          (int32_t)lrintf(output * 32767.0f);
      quantized = constrain(quantized, -32768, 32767);

      // Señal mono duplicada en los dos canales del PCM5102A.
      outputBuffer[frame * 2] = (int16_t)quantized;
      outputBuffer[frame * 2 + 1] = (int16_t)quantized;
    }

    // Gestiona correctamente una eventual escritura I2S parcial.
    const uint8_t *data =
        reinterpret_cast<const uint8_t *>(outputBuffer);
    size_t remaining = sizeof(outputBuffer);

    while (remaining > 0) {
      size_t written = I2S.write(data, remaining);

      if (written == 0) {
        taskYIELD();
        break;
      }

      data += written;
      remaining -= written;
    }
  }
}

// ============================================================================
// CONVERSION Y SALIDA MIDI
// ============================================================================

static uint8_t midi7(float normalizedValue) {
  return (uint8_t)lroundf(clamp01(normalizedValue) * 127.0f);
}

static uint8_t midiSweep7(float normalizedValue) {
  constexpr float CENTER_DEADBAND = 0.025f;

  if (fabsf(normalizedValue - 0.5f) <= CENTER_DEADBAND) {
    return 64;
  }

  return midi7(normalizedValue);
}

static uint8_t midiDelayTime7(float normalizedValue) {
  if (normalizedValue <= DELAY_BYPASS_THRESHOLD) {
    return 0;
  }

  float rescaled =
      (normalizedValue - DELAY_BYPASS_THRESHOLD) /
      (1.0f - DELAY_BYPASS_THRESHOLD);

  return midi7(rescaled);
}

static void sendMidiCCIfChanged(uint8_t controller,
                                uint8_t value,
                                uint8_t &lastValue,
                                uint32_t &lastSendTime,
                                bool force = false) {
  uint32_t now = millis();

  if (force || lastValue == 255) {
    lastValue = value;
    lastSendTime = now;
    MIDI.controlChange(controller, value, MIDI_CHANNEL);
    return;
  }

  int difference = abs((int)value - (int)lastValue);

  if (difference < MIDI_CC_HYSTERESIS) {
    return;
  }

  if ((uint32_t)(now - lastSendTime) <
      MIDI_CC_MIN_INTERVAL_MS) {
    return;
  }

  lastValue = value;
  lastSendTime = now;
  MIDI.controlChange(controller, value, MIDI_CHANNEL);
}

static void updateMidiContinuousControls(const ControlState &state,
                                         bool force = false) {
  static uint8_t lastPitch = 255;
  static uint8_t lastRate = 255;
  static uint8_t lastDepth = 255;
  static uint8_t lastDecay = 255;
  static uint8_t lastDelay = 255;
  static uint8_t lastFeedback = 255;
  static uint8_t lastSweep = 255;
  static uint8_t lastVolume = 255;
  static uint8_t lastWaveform = 255;
  static uint8_t lastLfo = 255;

  static uint32_t timePitch = 0;
  static uint32_t timeRate = 0;
  static uint32_t timeDepth = 0;
  static uint32_t timeDecay = 0;
  static uint32_t timeDelay = 0;
  static uint32_t timeFeedback = 0;
  static uint32_t timeSweep = 0;
  static uint32_t timeVolume = 0;
  static uint32_t timeWaveform = 0;
  static uint32_t timeLfo = 0;

  sendMidiCCIfChanged(
      CC_PITCH, midi7(state.pitch),
      lastPitch, timePitch, force);

  sendMidiCCIfChanged(
      CC_RATE, midi7(state.rate),
      lastRate, timeRate, force);

  sendMidiCCIfChanged(
      CC_DEPTH, midi7(state.depth),
      lastDepth, timeDepth, force);

  sendMidiCCIfChanged(
      CC_DECAY, midi7(state.decay),
      lastDecay, timeDecay, force);

  sendMidiCCIfChanged(
      CC_DELAY_TIME, midiDelayTime7(state.delayTime),
      lastDelay, timeDelay, force);

  sendMidiCCIfChanged(
      CC_DELAY_FB,
      midi7(state.delayFeedback / MAX_FEEDBACK),
      lastFeedback, timeFeedback, force);

  sendMidiCCIfChanged(
      CC_SWEEP, midiSweep7(state.sweep),
      lastSweep, timeSweep, force);

  sendMidiCCIfChanged(
      CC_VOLUME, midi7(state.volume),
      lastVolume, timeVolume, force);

  uint8_t waveformValue = 0;

  switch (state.carrier) {
    case SINE:
      waveformValue = 0;
      break;
    case SAW:
      waveformValue = 42;
      break;
    case SQUARE:
      waveformValue = 84;
      break;
    case LASER:
      waveformValue = 127;
      break;
  }

  sendMidiCCIfChanged(
      CC_WAVEFORM, waveformValue,
      lastWaveform, timeWaveform, force);

  uint8_t lfoValue = 0;

  switch (state.lfo) {
    case LFO_SINE:
      lfoValue = 0;
      break;
    case LFO_SAW:
      lfoValue = 42;
      break;
    case LFO_SQUARE:
      lfoValue = 84;
      break;
    case LFO_OFF:
      lfoValue = 127;
      break;
  }

  sendMidiCCIfChanged(
      CC_LFO_MODE, lfoValue,
      lastLfo, timeLfo, force);
}

static void sendMidiTrigger(bool pressed) {
  static bool initialized = false;
  static bool lastPressed = false;

  if (initialized && pressed == lastPressed) return;

  initialized = true;
  lastPressed = pressed;

  if (pressed) {
    MIDI.noteOn(
        MIDI_TRIGGER_NOTE,
        MIDI_TRIGGER_VELOCITY,
        MIDI_CHANNEL);
  } else {
    MIDI.noteOff(
        MIDI_TRIGGER_NOTE,
        0,
        MIDI_CHANNEL);
  }
}

// ============================================================================
// LECTURA Y FILTRADO DE CONTROLES
// ============================================================================

struct DebouncedButton {
  bool rawPressed = false;
  bool stablePressed = false;
  uint32_t changedAtMs = 0;
};

DebouncedButton triggerButton;

float filteredPots[8] = {
  0.5f, 0.5f, 0.5f, 0.5f,
  0.35f, 0.45f, 0.5f, 0.8f
};

const uint8_t potPins[8] = {
  PIN_PITCH,
  PIN_RATE,
  PIN_DEPTH,
  PIN_DECAY,
  PIN_DELAY_TIME,
  PIN_DELAY_FB,
  PIN_SWEEP,
  PIN_VOLUME
};

static float clampPotEnds(float value) {
  if (value <= POT_END_DEADBAND) return 0.0f;
  if (value >= 1.0f - POT_END_DEADBAND) return 1.0f;
  return value;
}

static uint16_t readPotMedianRaw(uint8_t pin) {
  uint16_t samples[POT_MEDIAN_SAMPLES];

  for (uint8_t i = 0; i < POT_MEDIAN_SAMPLES; ++i) {
    samples[i] = analogRead(pin);
  }

  // Ordenacion por insercion. Es eficiente para solo siete elementos.
  for (uint8_t i = 1; i < POT_MEDIAN_SAMPLES; ++i) {
    uint16_t value = samples[i];
    int8_t position = (int8_t)i - 1;

    while (position >= 0 && samples[position] > value) {
      samples[position + 1] = samples[position];
      --position;
    }

    samples[position + 1] = value;
  }

  return samples[POT_MEDIAN_SAMPLES / 2];
}

static float readPotFilteredInput(uint8_t pin) {
  return (float)readPotMedianRaw(pin) / 4095.0f;
}

static Waveform readCarrierSwitch(Waveform previous) {
  if (!digitalRead(PIN_WAVE_SINE))   return SINE;
  if (!digitalRead(PIN_WAVE_SAW))    return SAW;
  if (!digitalRead(PIN_WAVE_SQUARE)) return SQUARE;
  if (!digitalRead(PIN_WAVE_LASER))  return LASER;
  return previous;
}

static LfoMode readLfoSwitch(LfoMode previous) {
  if (!digitalRead(PIN_LFO_SINE))   return LFO_SINE;
  if (!digitalRead(PIN_LFO_SAW))    return LFO_SAW;
  if (!digitalRead(PIN_LFO_SQUARE)) return LFO_SQUARE;
  if (!digitalRead(PIN_LFO_OFF))    return LFO_OFF;
  return previous;
}

static void updateTriggerDebounce(uint32_t nowMs) {
  bool rawPressed = digitalRead(PIN_BTN_TRIG) == LOW;

  if (rawPressed != triggerButton.rawPressed) {
    triggerButton.rawPressed = rawPressed;
    triggerButton.changedAtMs = nowMs;
  }

  if ((uint32_t)(nowMs - triggerButton.changedAtMs) >=
          BUTTON_DEBOUNCE_MS &&
      triggerButton.stablePressed !=
          triggerButton.rawPressed) {

    triggerButton.stablePressed = triggerButton.rawPressed;

    portENTER_CRITICAL(&controlsMux);
    controls.trigger = triggerButton.stablePressed;
    portEXIT_CRITICAL(&controlsMux);

    digitalWrite(
        PIN_LED_TRIG,
        triggerButton.stablePressed ? HIGH : LOW);

    sendMidiTrigger(triggerButton.stablePressed);
  }
}

static void readAndPublishControls(bool initializeFilters = false,
                                   bool forceMidi = false) {
  ControlState next;

  portENTER_CRITICAL(&controlsMux);
  next = controls;
  portEXIT_CRITICAL(&controlsMux);

  for (int i = 0; i < 8; ++i) {
    float raw = clampPotEnds(readPotFilteredInput(potPins[i]));

    if (initializeFilters) {
      filteredPots[i] = raw;
      continue;
    }

    float difference = raw - filteredPots[i];

    // Ignora pequeñas variaciones producidas por el ruido del ADC.
    if (fabsf(difference) <= POT_INPUT_DEADBAND) {
      continue;
    }

    // Retira la zona muerta antes del EMA para evitar un salto al salir.
    if (difference > 0.0f) {
      difference -= POT_INPUT_DEADBAND;
    } else {
      difference += POT_INPUT_DEADBAND;
    }

    filteredPots[i] += difference * POT_FILTER_ALPHA;
    filteredPots[i] = clampPotEnds(clamp01(filteredPots[i]));
  }

  next.pitch = filteredPots[0];
  next.rate = filteredPots[1];
  next.depth = filteredPots[2];
  next.decay = filteredPots[3];
  next.delayTime = filteredPots[4];
  next.delayFeedback = filteredPots[5] * MAX_FEEDBACK;
  next.sweep = filteredPots[6];
  next.volume = filteredPots[7];
  next.carrier = readCarrierSwitch(next.carrier);
  next.lfo = readLfoSwitch(next.lfo);
  next.trigger = triggerButton.stablePressed;

  portENTER_CRITICAL(&controlsMux);
  controls = next;
  portEXIT_CRITICAL(&controlsMux);

  updateMidiContinuousControls(next, forceMidi);
}

// ============================================================================
// SETUP Y LOOP
// ============================================================================

void setup() {
  pinMode(PIN_WAVE_SINE, INPUT_PULLUP);
  pinMode(PIN_WAVE_LASER, INPUT_PULLUP);
  pinMode(PIN_WAVE_SQUARE, INPUT_PULLUP);
  pinMode(PIN_WAVE_SAW, INPUT_PULLUP);

  pinMode(PIN_LFO_SINE, INPUT_PULLUP);
  pinMode(PIN_LFO_SAW, INPUT_PULLUP);
  pinMode(PIN_LFO_SQUARE, INPUT_PULLUP);
  pinMode(PIN_LFO_OFF, INPUT_PULLUP);

  pinMode(PIN_BTN_TRIG, INPUT_PULLUP);
  pinMode(PIN_LED_TRIG, OUTPUT);
  digitalWrite(PIN_LED_TRIG, LOW);

  analogReadResolution(12);

  for (uint8_t pin : potPins) {
    analogSetPinAttenuation(pin, ADC_11db);
  }

  initSineTable();
  memset(delayBuffer, 0, sizeof(delayBuffer));

  // Inicializacion USB-MIDI class-compliant.
  MIDI.begin();
  USB.begin();

  bool initialTrigger = digitalRead(PIN_BTN_TRIG) == LOW;
  triggerButton.rawPressed = initialTrigger;
  triggerButton.stablePressed = initialTrigger;
  triggerButton.changedAtMs = millis();

  digitalWrite(PIN_LED_TRIG, initialTrigger ? HIGH : LOW);

  // Lee los estados fisicos y envia un snapshot MIDI inicial.
  readAndPublishControls(true, true);
  sendMidiTrigger(initialTrigger);

  I2S.setPins(I2S_BCK, I2S_WS, I2S_DOUT);

  if (!I2S.begin(
          I2S_MODE_STD,
          SAMPLE_RATE,
          I2S_DATA_BIT_WIDTH_16BIT,
          I2S_SLOT_MODE_STEREO,
          I2S_STD_SLOT_BOTH)) {

    while (true) {
      digitalWrite(PIN_LED_TRIG, !digitalRead(PIN_LED_TRIG));
      delay(150);
    }
  }

  BaseType_t taskResult = xTaskCreatePinnedToCore(
      AudioTask,
      "AudioTask",
      8192,
      nullptr,
      20,
      &audioTaskHandle,
      0);

  if (taskResult != pdPASS) {
    while (true) {
      digitalWrite(PIN_LED_TRIG, !digitalRead(PIN_LED_TRIG));
      delay(500);
    }
  }
}

void loop() {
  static uint32_t lastControlReadMs = 0;
  uint32_t nowMs = millis();

  // El trigger se atiende cada milisegundo aproximadamente.
  updateTriggerDebounce(nowMs);

  // Potenciometros y selectores a 50 Hz.
  if ((uint32_t)(nowMs - lastControlReadMs) >=
      CONTROL_READ_INTERVAL_MS) {
    lastControlReadMs = nowMs;
    readAndPublishControls(false, false);
  }

  delay(1);
}
