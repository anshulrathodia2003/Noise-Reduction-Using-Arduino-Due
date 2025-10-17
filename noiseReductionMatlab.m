%% ==========================
% Real-time FFT Noise Reduction Simulation (Offline)
% Records audio, adds noise, performs FFT noise reduction using OLA
% Saves clean_speech.wav and noisy_speech.wav
%% ==========================

clc; clear; close all;

%% ----- User Parameters -----
fs = 8000;           % Sampling frequency (Hz)
duration = 5;        % Recording duration (seconds)
deviceID = 8;        % Earbuds mic (from audiodevinfo)

SAMPLES = 256;       % FFT window size
HOP_SIZE = 128;      % Overlap size
NOISE_BUFFERS = 5;   % Number of buffers for noise estimation


%% ----- Step 1: Record Clean Speech -----
disp('🎤 Recording clean speech using earbuds mic...');
recObj = audiorecorder(fs,16,1,deviceID);
recordblocking(recObj, duration);
x_clean = getaudiodata(recObj);
disp('✅ Recording complete.');
sound(x_clean, fs);   % Playback
                   % Listen to the recorded speech
audiowrite('/home/argonaspect/Desktop/audioSignals/clean_speech.wav', x_clean, fs); % Save to WAV

sound(x_clean, fs);

%% ----- Step 2: Add Background Noise -----
% Compute RMS of clean speech
rms_speech = sqrt(mean(x_clean.^2));

% Desired SNR (Signal-to-Noise Ratio) in dB
SNR_dB = 10;  % for example, 10 dB

% Compute noise RMS
rms_noise = rms_speech / (10^(SNR_dB/20));

% Generate noise with this RMS
noise = rms_noise * randn(size(x_clean));

x_noisy = x_clean + noise;

% Save noisy speech
audiowrite('/home/argonaspect/Desktop/audioSignals/noisy_speech.wav', x_noisy, fs);
disp('Noisy speech saved as noisy_speech.wav');
sound(x_noisy, fs);      % Playback

%% ----- Step 3: Initialize FFT Buffers -----
vReal = zeros(SAMPLES,1);
vImag = zeros(SAMPLES,1);
overlap = zeros(HOP_SIZE,1);
outputBuffer = zeros(SAMPLES,1);

% Noise estimation buffer (first few windows)
noiseMag = zeros(SAMPLES/2,1);

%% ----- Step 4: Noise Profile Estimation -----
disp('Estimating noise profile...');
num_windows = floor(length(x_noisy)/HOP_SIZE)-1;
buffers_collected = 0;

for w = 1:num_windows
    startIdx = (w-1)*HOP_SIZE + 1;
    buf = x_noisy(startIdx : startIdx+SAMPLES-1);
    
    % Use first few buffers to estimate noise
    if buffers_collected < NOISE_BUFFERS
        buf = buf - mean(buf);           % Remove DC bias
        vReal(:) = buf; vImag(:)=0;
        vReal = vReal .* hamming(SAMPLES);
        V = fft(vReal);
        mag = abs(V(1:SAMPLES/2));
        noiseMag = noiseMag + mag;
        buffers_collected = buffers_collected +1;
    else
        break;
    end
end
noiseMag = noiseMag / NOISE_BUFFERS;
disp('Noise profile done.');


%% ----- Step 5: FFT Noise Reduction with Overlap-Add -----
y_cleaned = zeros(length(x_noisy),1);

for w = 1:num_windows
    startIdx = (w-1)*HOP_SIZE + 1;
    buf = x_noisy(startIdx : startIdx+SAMPLES-1);
    
    % Remove DC bias
    buf = buf - mean(buf);
    
    % FFT
    vReal(:) = buf; vImag(:)=0;
    vReal = vReal .* hamming(SAMPLES);
    V = fft(vReal);
    
    % Spectral subtraction
    mag = abs(V);
    phase = angle(V);
    mag(1:SAMPLES/2) = mag(1:SAMPLES/2) - noiseMag;
    mag(1:SAMPLES/2) = max(mag(1:SAMPLES/2), 0); % No negative magnitudes
    % Mirror for negative frequencies (CORRECTED)
mag(SAMPLES/2+2:end) = flipud(mag(2:SAMPLES/2)); % Left: 127 elements, Right: 127 elements
    
    % IFFT
    V_filtered = mag .* exp(1j*phase);
    x_ifft = real(ifft(V_filtered));
    
    % Overlap-Add
    outputBuffer(1:HOP_SIZE) = x_ifft(1:HOP_SIZE) + overlap;
    outputBuffer(HOP_SIZE+1:end) = x_ifft(HOP_SIZE+1:end);
    overlap = x_ifft(HOP_SIZE+1:end);
    
    % Store in final output
    y_cleaned(startIdx : startIdx+HOP_SIZE-1) = outputBuffer(1:HOP_SIZE);
end

%% ----- Step 6: Save and Listen Cleaned Output -----
audiowrite('/home/argonaspect/Desktop/audioSignals/cleaned_speech.wav', y_cleaned, fs);
disp('Cleaned speech saved as cleaned_speech.wav');
sound(y_cleaned, fs);

%% ----- End of Script -----
disp('✅ All done!');



info = audiodevinfo;
for k = 1:length(info.input)
    fprintf('ID: %d  Name: %s\n', k-1, info.input(k).Name);
end

info = audiodevinfo
info.input(2)
