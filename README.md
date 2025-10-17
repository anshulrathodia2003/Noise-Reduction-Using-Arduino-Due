# Real-Time Audio Noise Suppression System

A high-performance embedded audio processing system for real-time noise reduction using spectral subtraction on Arduino Due. This project demonstrates advanced signal processing, hardware-software integration, and real-time embedded systems design.

---

## Overview

This system captures audio input via a microphone, analyzes its frequency spectrum, suppresses background noise using spectral subtraction, and outputs clean audio in real-time. Built with hardware timer-based interrupts for deterministic 8 kHz sampling and processing latency under 50 ms.

### Key Features

- **Real-time Processing:** 32 ms buffer latency with overlap-add windowing for smooth audio
- **Spectral Subtraction:** FFT-based noise reduction with spectral floor to minimize artifacts
- **Ping-Pong Buffering:** Lock-free dual-buffer design prevents race conditions
- **Hardware Interrupts:** TC0/TC1 timer-based sampling ensures jitter-free audio acquisition
- **Performance Monitoring:** Built-in diagnostics track processing time and system health
- **Noise Profile Learning:** Automatic noise characterization during startup
- **Dynamic Normalization:** Adaptive output scaling to 12-bit DAC range

---

## System Architecture

### Hardware Layer
```
Microphone → ADC (A0, 12-bit) → [TC0 ISR @ 8 kHz] → Ping-Pong Buffers
                                                              ↓
DAC (DAC0, 12-bit) ← [TC1 ISR @ 8 kHz] ← Processed Audio Buffer
                Headphones/Speaker
```

### Software Pipeline
```
Input Buffer → DC Bias Removal → Hamming Window → FFT Analysis
                                                    ↓
Spectral Subtraction (Noise Profile Subtraction with Floor)
   ↓
Inverse FFT → Overlap-Add → Dynamic Normalization → Output Buffer
```

### Timing Diagram
```
Time:     0 ms          32 ms         64 ms         96 ms
          ├─────────────┼─────────────┼─────────────┤
ADC:      [Buffer A ••••][Buffer B ••••][Buffer A ••••][Buffer B ••••]
Main:            [Proc B][Proc A]      [Proc B][Proc A]
DAC:             [Play B][Play A]      [Play B][Play A]
```

- **ISR Frequency:** Every 125 µs (8000 samples/sec)
- **Buffer Duration:** 256 samples = 32 ms
- **Processing Budget:** ≤ 50 ms per frame (15-20 ms typical)
- **Total Latency:** ~40-50 ms (input to audible output)

---

## Technical Details

### Signal Processing

**1. Windowing**
- Function: Hamming window applied before FFT
- Purpose: Reduces spectral leakage at frame boundaries
- Configuration: `FFT_WIN_TYP_HAMMING`

**2. Frequency Analysis**
- Transform: 256-point FFT (0–4 kHz bandwidth at 8 kHz sample rate)
- Resolution: ~31 Hz per bin
- Output: Magnitude and phase spectra

**3. Spectral Subtraction**
```
NewMagnitude = Magnitude - NoiseProfile
if (NewMagnitude < SPECTRAL_FLOOR × Magnitude)
    NewMagnitude = SPECTRAL_FLOOR × Magnitude
```
- **SPECTRAL_FLOOR:** 0.3 (prevents over-subtraction musical noise)
- **MIN_RANGE_THRESHOLD:** 1e-6 (handles silent frames)

**4. Synthesis & Overlap-Add**
- 50% overlap between frames (HOP_SIZE = 128)
- Smooth transitions eliminate clicks/pops
- Preserves original signal phase for natural audio

**5. Dynamic Normalization**
- Scales output to full 12-bit DAC range (0–4095)
- Adapts per-frame for consistent loudness
- Prevents clipping and maximizes SNR

### Real-Time Guarantees

**Ping-Pong Buffering Strategy:**
- While TC0 ISR fills Buffer A, main loop safely processes Buffer B
- No locks or mutexes needed—lock-free design
- Zero-copy handoff minimizes critical sections

**Critical Sections:**
```cpp
noInterrupts();
// Only buffer copy (memcpy) - ~10 µs
// ISRs suspended for <100 µs total per frame
interrupts();
```

---

## Hardware Requirements

### Microcontroller
- **Arduino Due** (ARM Cortex-M3, 84 MHz, SAM3X8E)
- Programming interface: Native USB port

### Audio I/O
- **Microphone:** Any electret condenser or dynamic mic (0–3.3V output)
  - Recommended: Analog microphone module with preamp
  - Input impedance: >1 kΩ
  - Frequency response: 100 Hz – 4 kHz acceptable for speech
  
- **Speaker/Headphone:** 4–16 Ω, 0.5–2W
  - DAC output impedance: ~1 kΩ (use amplifier for speaker)
  - PWM-based amplifier can drive 8 Ω speaker directly (with 1kΩ series resistor as safety)

### Optional Components
- 10 µF capacitors on ADC/DAC pins (noise filtering)
- Series 1 kΩ resistor on microphone input (ESD protection)
- Dual-supply op-amp buffer for mic preamp (improves SNR)

### Pinout
```
Arduino Due Connections:
┌─────────────────────┐
│ Microphone Input    │ → A0  (ADC input)
│ Speaker/Headphone   │ ← DAC0 (12-bit DAC output)
│ GND                 │ → GND
│ Power               │ ← +3.3V (DAC), +5V (external preamp)
└─────────────────────┘
```

---

## Installation & Setup

### Prerequisites
- Arduino IDE 1.8.0 or later
- Arduino Due board support installed
- `arduinoFFT` library (v1.9.2 or compatible)

### Library Installation

1. In Arduino IDE: **Sketch** → **Include Library** → **Manage Libraries**
2. Search for `arduinoFFT` by Enrique Condes
3. Click **Install**

### Board Selection
- **Tools** → **Board:** Arduino Due (Programming Port)
- **Tools** → **Port:** Select your Arduino's USB port

### Upload Code
1. Connect Arduino Due to computer via USB (Programming Port)
2. Open `NoiseReductionMain.ino` in Arduino IDE
3. Click **Upload** (Ctrl+U)
4. Open **Serial Monitor** (115200 baud) to see diagnostics

---

## Configuration Parameters

All settings in the `// ============ CONFIGURATION ============` section:

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `SAMPLES` | 256 | FFT frame size (power of 2) |
| `HOP_SIZE` | 128 | Overlap-add hop size (50% overlap) |
| `SAMPLE_RATE` | 8000 | ADC/DAC sampling frequency (Hz) |
| `NOISE_BUFFERS` | 5 | Frames used for noise profile (32 sec startup) |
| `ADC_PIN` | A0 | Microphone input analog pin |
| `DAC_PIN` | DAC0 | Speaker output DAC pin |
| `SPECTRAL_FLOOR` | 0.3 | Noise reduction aggressiveness (0–1) |
| `MAX_PROCESSING_TIME_US` | 50000 | Alert threshold (microseconds) |

### Tuning Tips

**More aggressive noise reduction:**
- Decrease `SPECTRAL_FLOOR` (e.g., 0.2)
- Trade-off: Increases musical noise artifacts

**Less residual noise:**
- Increase `NOISE_BUFFERS` (e.g., 10–20)
- Trade-off: Longer startup time, requires quiet environment

**Lower latency:**
- Decrease `SAMPLES` to 128 (requires FFT library support)
- Trade-off: Reduced frequency resolution

---

## Usage

### Startup Sequence
1. Connect microphone and speaker/headphone
2. Upload code to Arduino Due
3. Open Serial Monitor (115200 baud)
4. System outputs:
   ```
   Estimating noise profile...
   Noise buffer 1 collected.
   Noise buffer 2 collected.
   ...
   Noise profile complete. Starting real-time processing.
   System running normally.
   ```
5. Speak into microphone—listen to denoised output

### Serial Diagnostics
```
System running normally.                    [every 5 seconds]
WARNING: Processing took 52000 us (...)     [if overloaded]
ERROR: Noise profile timeout. Check ADC.    [if ADC fails]
```

### Manual Reset
Press the **RESET** button on Arduino Due to restart noise profile estimation.

---

## Troubleshooting

### No Audio Output
- Check speaker/headphone connections
- Verify DAC0 pin is not overloaded
- Try amplifying DAC output with op-amp buffer

### Distorted Output
- Reduce microphone gain (too loud → ADC clipping)
- Increase `SPECTRAL_FLOOR` (musical noise too aggressive)
- Check for USB power supply noise (use external PSU)

### Slow Processing Warnings
- Monitor Serial output for processing time
- If consistently >50 ms, reduce `SAMPLES` or increase `SAMPLE_RATE`
- Ensure no other interrupts competing with TC0/TC1

### Noise Profile Not Loading
- Serial output shows "ERROR: Noise profile timeout"
- Check ADC pin (A0) connection to microphone
- Verify analogRead() is working (test with analogRead example)
- Increase `SETUP_TIMEOUT_MS` if environment is extremely noisy

### Musical Noise / Over-Suppression
- Increase `SPECTRAL_FLOOR` (0.3 → 0.5)
- Decrease `NOISE_BUFFERS` (if noise profile is outdated)
- Consider using lower `SAMPLE_RATE` for cleaner noise floor

---

## Implementation Notes

### Why Spectral Subtraction?
- Simple to implement on embedded devices
- Fast (one FFT per frame = ~2 ms on Due)
- Works well for stationary noise (fans, HVAC, background chatter)
- Limitations: Struggles with non-stationary noise (traffic, music)

### Why Overlap-Add?
- Hamming window causes gain variation at frame boundaries
- 50% overlap ensures perfect reconstruction property
- Smooth transitions prevent audible clicks/pops

### Why Ping-Pong Buffers?
- Eliminates race conditions without mutexes
- Deterministic ISR timing (no waiting for locks)
- Maximizes CPU efficiency for main processing loop

---

## Future Enhancements

- [ ] **Wiener Filtering:** Adaptive filter for better quality
- [ ] **Voice Activity Detection:** Reduce noise when silent
- [ ] **Adaptive Noise Profile:** Online learning during playback
- [ ] **Multi-rate Processing:** 16 kHz for broader frequency range
- [ ] **Web Dashboard:** Real-time frequency plot via Ethernet shield

---

## Authors

- **Anshul Rathodia and Angela Singhal** - Initial implementation, EE301N Microprocessors and Digital Systems Design
- BTech 5th Sem Course Project, [IIT INDORE]

---

## License

This project is licensed under the **MIT License** - see the LICENSE file for details.

Free to use in personal, educational, and commercial projects with attribution.

---

## Acknowledgments

- Arduino community for hardware platforms and libraries
- Kosme (arduinoFFT maintainer) for FFT library
- Signal processing instructors for DSP fundamentals
- Classmates for hardware testing and feedback

---

## Contact & Support

**Questions?** Open an issue on GitHub or contact via email.

**Presentation/Demo:** This system is designed for real-time audio demonstration. Works best in moderately noisy environments (offices, classrooms, cafés).

**Hardware Support:** Compatible with Arduino Due only. Porting to other boards requires timer configuration adjustments.

---

**Last Updated:** October 2025 | **Status:** Active Development
