// classifier.cpp
#include "classifier.h"
#include "FFT.h"
#include "FFT_signal.h"
#include <math.h>
#include "variant.h"
#include "fft_config.h"


#define SAMPLING_FREQUENCY 333  // Hz


String classifyBufferedData(float* accelX_buffer, float* accelY_buffer, float* accelZ_buffer, int num_samples) {
    float meanX = 0, meanY = 0, meanZ = 0;
    for (int i = 0; i < num_samples; i++) {
        meanX += accelX_buffer[i];
        meanY += accelY_buffer[i];
        meanZ += accelZ_buffer[i];
    }
    meanX /= num_samples;
    meanY /= num_samples;
    meanZ /= num_samples;

    for (int i = 0; i < num_samples; i++) {
        float x = accelX_buffer[i] - meanX;
        float y = accelY_buffer[i] - meanY;
        float z = accelZ_buffer[i] - meanZ;
        fft_input[i] = sqrt(x * x + y * y + z * z);
    }

    fft_config_t *fft_plan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input, fft_output);
    if (!fft_plan) {
        Serial.println("[ERROR] FFT plan allocation failed!");
        return "⚠️ Unclassified";
    }
    fft_execute(fft_plan);

    double total_energy = 0;
    double band_0_25 = 0;
    double band_125_200 = 0;

    for (int i = 1; i < FFT_N / 2; i++) {
        float real = fft_output[2 * i];
        float imag = fft_output[2 * i + 1];
        float mag = sqrt(real * real + imag * imag);
        float freq = i * 1.0 / (float)(FFT_N) * SAMPLING_FREQUENCY;
        total_energy += mag;
        if (freq <= 25) band_0_25 += mag;
        if (freq >= 125 && freq <= 200) band_125_200 += mag;
    }

    fft_destroy(fft_plan);

    float fft_0_25_ratio = total_energy > 0 ? band_0_25 / total_energy : 0;
    float fft_125_200_ratio = total_energy > 0 ? band_125_200 / total_energy : 0;

    float offset_x_y = 0;
    float offset_y_z = 0;
    float max_x = 0, max_y = 0, max_z = 0;
    for (int i = 0; i < num_samples; i++) {
        offset_x_y += (accelX_buffer[i] - accelY_buffer[i]);
        offset_y_z += (accelY_buffer[i] - accelZ_buffer[i]);
        if (abs(accelX_buffer[i]) > max_x) max_x = abs(accelX_buffer[i]);
        if (abs(accelY_buffer[i]) > max_y) max_y = abs(accelY_buffer[i]);
        if (abs(accelZ_buffer[i]) > max_z) max_z = abs(accelZ_buffer[i]);
    }
    offset_x_y /= num_samples;
    offset_y_z /= num_samples;

    int strike_count = 0;
    for (int i = 1; i < num_samples - 1; i++) {
        if (fft_input[i] > 5000 && fft_input[i] > fft_input[i - 1] && fft_input[i] > fft_input[i + 1]) {
            strike_count++;
        }
    }

    float hf_energy = 0;
    for (int i = FFT_N / 8; i < FFT_N / 2; i++) {
        float real = fft_output[2 * i];
        float imag = fft_output[2 * i + 1];
        hf_energy += sqrt(real * real + imag * imag);
    }
    hf_energy /= total_energy > 0 ? total_energy : 1;

    Serial.printf("\n--- Feature Summary ---\n");
    Serial.printf("hf_energy: %.2f\n", hf_energy);
    Serial.printf("fft_0_25_ratio: %.2f\n", fft_0_25_ratio);
    Serial.printf("fft_125_200_ratio: %.2f\n", fft_125_200_ratio);
    Serial.printf("strike_count: %d\n", strike_count);
    Serial.printf("offset_x_y: %.2f\n", offset_x_y);
    Serial.printf("offset_y_z: %.2f\n", offset_y_z);
    Serial.printf("max_y: %.2f\n", max_y);
    Serial.printf("max_z: %.2f\n", max_z);

    if (
        strike_count >= 1 && strike_count <= 100 &&
        hf_energy >= 0.4 &&
        fft_0_25_ratio > 0.3 &&
        fft_125_200_ratio < 0.22 &&
        offset_x_y >= 16000 &&
        abs(offset_y_z) <= 1200 &&
        max_y > 15000 && max_z > 15000
    ) return "✅ Likely Machete";

    else if (
        strike_count > 200 &&
        hf_energy > 0.6 &&
        fft_0_25_ratio < 0.3 &&
        fft_125_200_ratio > 0.15 &&
        (max_y > 3000 || max_z > 3000)
    ) return "🔧 Likely Chainsaw";

    else if (
        max_y < 2000 &&
        max_z < 2000 &&
        offset_x_y > 16000 &&
        fft_125_200_ratio > 0.12 && fft_125_200_ratio < 0.27 &&
        hf_energy > 0.7 &&
        strike_count > 600
    ) return "🌬️ Likely Non-Event";

    else if (
        strike_count >= 70 && strike_count <= 110 &&
        hf_energy > 0.45 &&
        fft_0_25_ratio >= 0.22 &&
        fft_125_200_ratio < 0.27 &&
        offset_x_y >= 15300 &&
        abs(offset_y_z) <= 1300 &&
        (max_y > 15000 || max_z > 15000)
    ) return "🟡 Possibly Machete";

    else if (
        fft_125_200_ratio > 0.28 &&
        hf_energy > 0.75 &&
        strike_count < 300 &&
        max_y > 10000 && max_z > 10000
    ) return "⚠️ Possibly Chainsaw (Low Activity)";

    else if (
        hf_energy > 0.5 &&
        strike_count > 20 &&
        (max_y > 1000 || max_z > 1000)
    ) return "⚠️ Vibration Detected (ambiguous)";

    return "⚠️ Unclassified";
}
