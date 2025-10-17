#include <arduinoFFT.h>
#include <Arduino.h>

// ============ CONFIGURATION ============
#define SAMPLES      256
#define HOP_SIZE     128
#define SAMPLE_RATE  8000
#define NOISE_BUFFERS 5
#define ADC_PIN      A0
#define DAC_PIN      DAC0

// Safety thresholds
#define MIN_RANGE_THRESHOLD 1e-6
#define SETUP_TIMEOUT_MS 10000
#define MAX_PROCESSING_TIME_US 50000  // Alert if processing takes too long

// ============ GLOBALS ============
arduinoFFT FFT = arduinoFFT();

// Buffers
volatile uint16_t adcBufferA[SAMPLES];
volatile uint16_t adcBufferB[SAMPLES];
volatile bool useBufferA = true;
volatile int adcIndex = 0;

double vReal[SAMPLES], vImag[SAMPLES];
double noiseMag[SAMPLES/2];
double overlap[HOP_SIZE];
double outputBuffer[SAMPLES];

volatile uint16_t dacBuffer[SAMPLES];
volatile int dacIndex = 0;
volatile bool dacBufferReady = false;

// Status flags
volatile bool adcBufferReady = false;
volatile uint32_t processingTimeUs = 0;

// ============ SETUP ============
void setup() {
  analogReadResolution(12);
  analogWriteResolution(12);
  Serial.begin(115200);

  // Initialize overlap buffer
  for(int i = 0; i < HOP_SIZE; i++) {
    overlap[i] = 0;
  }

  // Setup timers
  setupADC_TC();
  setupDAC_TC();

  // Noise estimation with timeout
  Serial.println("Estimating noise profile...");
  estimateNoiseProfile();
  Serial.println("Noise profile complete. Starting real-time processing.");
}

// ============ MAIN LOOP ============
void loop() {
  if(adcBufferReady) {
    uint32_t startTime = micros();

    // Safely read ADC buffer
    double procBuffer[SAMPLES];
    noInterrupts();
    if(useBufferA) {
      // ISR filling B, safe to read A
      memcpy(procBuffer, (void*)adcBufferA, SAMPLES * sizeof(uint16_t));
    } else {
      // ISR filling A, safe to read B
      memcpy(procBuffer, (void*)adcBufferB, SAMPLES * sizeof(uint16_t));
    }
    adcBufferReady = false;
    interrupts();

    // Convert to double and remove DC bias
    double bufDouble[SAMPLES];
    for(int i = 0; i < SAMPLES; i++) {
      bufDouble[i] = (double)procBuffer[i];
    }
    removeDCBias(bufDouble);

    // ---- FFT Analysis ----
    for(int i = 0; i < SAMPLES; i++) {
      vReal[i] = bufDouble[i];
      vImag[i] = 0.0;
    }
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);

    // ---- Spectral Subtraction with Floor ----
    const double SPECTRAL_FLOOR = 0.3;  // Prevent over-subtraction artifacts
    for(int i = 0; i < SAMPLES/2; i++) {
      double mag = sqrt(vReal[i]*vReal[i] + vImag[i]*vImag[i]);
      double newMag = mag - noiseMag[i];
      
      // Apply spectral floor to reduce musical noise
      if(newMag < SPECTRAL_FLOOR * mag) {
        newMag = SPECTRAL_FLOOR * mag;
      }
      
      double phase = atan2(vImag[i], vReal[i]);
      vReal[i] = newMag * cos(phase);
      vImag[i] = newMag * sin(phase);
    }

    // ---- IFFT Synthesis ----
    FFT.Compute(vReal, vImag, SAMPLES, FFT_REVERSE);

    // ---- Overlap-Add ----
    for(int i = 0; i < HOP_SIZE; i++) {
      outputBuffer[i] = vReal[i] + overlap[i];
    }
    for(int i = HOP_SIZE; i < SAMPLES; i++) {
      outputBuffer[i] = vReal[i];
    }
    for(int i = 0; i < HOP_SIZE; i++) {
      overlap[i] = vReal[HOP_SIZE + i];
    }

    // ---- Output Scaling (Normalization) ----
    double minVal = outputBuffer[0];
    double maxVal = outputBuffer[0];
    for(int i = 1; i < SAMPLES; i++) {
      if(outputBuffer[i] < minVal) minVal = outputBuffer[i];
      if(outputBuffer[i] > maxVal) maxVal = outputBuffer[i];
    }

    double range = maxVal - minVal;
    if(range < MIN_RANGE_THRESHOLD) {
      range = MIN_RANGE_THRESHOLD;
    }

    // Fill DAC buffer
    noInterrupts();
    for(int i = 0; i < SAMPLES; i++) {
      double scaled = (outputBuffer[i] - minVal) / range * 4095.0;
      dacBuffer[i] = (uint16_t)constrain(scaled, 0, 4095);
    }
    dacIndex = 0;
    dacBufferReady = true;
    interrupts();

    // Monitor processing time
    uint32_t elapsed = micros() - startTime;
    if(elapsed > MAX_PROCESSING_TIME_US) {
      Serial.print("WARNING: Processing took ");
      Serial.print(elapsed);
      Serial.println(" us (max recommended: 50000 us)");
    }
  }

  // Optional: Print diagnostics periodically
  static unsigned long lastDiag = 0;
  if(millis() - lastDiag > 5000) {
    lastDiag = millis();
    Serial.println("System running normally.");
  }
}

// ============ NOISE PROFILE ESTIMATION ============
void estimateNoiseProfile() {
  // Initialize noise array
  for(int i = 0; i < SAMPLES/2; i++) {
    noiseMag[i] = 0;
  }

  int buffersCollected = 0;
  unsigned long startTime = millis();

  while(buffersCollected < NOISE_BUFFERS) {
    // Timeout safety check
    if(millis() - startTime > SETUP_TIMEOUT_MS) {
      Serial.println("ERROR: Noise profile timeout. Check ADC connections.");
      return;
    }

    if(adcBufferReady) {
      double buf[SAMPLES];
      
      noInterrupts();
      if(useBufferA) {
        memcpy(buf, (void*)adcBufferB, SAMPLES * sizeof(uint16_t));
      } else {
        memcpy(buf, (void*)adcBufferA, SAMPLES * sizeof(uint16_t));
      }
      adcBufferReady = false;
      interrupts();

      // Convert to double
      double bufDouble[SAMPLES];
      for(int i = 0; i < SAMPLES; i++) {
        bufDouble[i] = (double)buf[i];
      }
      removeDCBias(bufDouble);

      // FFT for noise characterization
      for(int i = 0; i < SAMPLES; i++) {
        vReal[i] = bufDouble[i];
        vImag[i] = 0.0;
      }
      FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);

      // Accumulate magnitude spectrum
      for(int i = 0; i < SAMPLES/2; i++) {
        double mag = sqrt(vReal[i]*vReal[i] + vImag[i]*vImag[i]);
        noiseMag[i] += mag;
      }

      buffersCollected++;
      Serial.print("Noise buffer ");
      Serial.print(buffersCollected);
      Serial.println(" collected.");
    }
  }

  // Average the noise magnitude
  for(int i = 0; i < SAMPLES/2; i++) {
    noiseMag[i] /= NOISE_BUFFERS;
  }
}

// ============ TIMER/ADC SETUP ============
void setupADC_TC() {
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);

  TC_Configure(TC0, 0,
    TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);

  uint32_t rc = VARIANT_MCK / 2 / SAMPLE_RATE;
  TC_SetRC(TC0, 0, rc);

  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);

  TC_Start(TC0, 0);
  Serial.println("ADC timer configured for 8 kHz sampling.");
}

// ============ TIMER/DAC SETUP ============
void setupDAC_TC() {
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC1);

  TC_Configure(TC1, 0,
    TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);

  uint32_t rc = VARIANT_MCK / 2 / SAMPLE_RATE;
  TC_SetRC(TC1, 0, rc);

  TC1->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC1_IRQn);  // FIXED: Was TC3_IRQn, now TC1_IRQn

  TC_Start(TC1, 0);
  Serial.println("DAC timer configured for 8 kHz output.");
}

// ============ INTERRUPT HANDLERS ============
void TC0_Handler() {
  TC_GetStatus(TC0, 0);
  
  if(adcIndex < SAMPLES) {
    if(useBufferA) {
      adcBufferA[adcIndex] = analogRead(ADC_PIN);
    } else {
      adcBufferB[adcIndex] = analogRead(ADC_PIN);
    }
    adcIndex++;
  } else {
    adcIndex = 0;
    useBufferA = !useBufferA;
    adcBufferReady = true;
  }
}

void TC1_Handler() {  // FIXED: Was TC3_Handler
  TC_GetStatus(TC1, 0);  // FIXED: Was TC_GetStatus(TC1, 0)
  
  if(dacBufferReady && dacIndex < SAMPLES) {
    analogWrite(DAC_PIN, dacBuffer[dacIndex]);
    dacIndex++;
  }
  // dacIndex will be reset to 0 by main loop when new buffer ready
}

// ============ HELPER FUNCTIONS ============
void removeDCBias(double* buffer) {
  double mean = 0.0;
  for(int i = 0; i < SAMPLES; i++) {
    mean += buffer[i];
  }
  mean /= SAMPLES;
  
  for(int i = 0; i < SAMPLES; i++) {
    buffer[i] -= mean;
  }
}