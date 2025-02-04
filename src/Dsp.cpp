/**
 * @file Dsp.cpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2025-01-26
 * 
 * @copyright Gabriel Garcia | 2024
 * @brief Implementation file for Class BandStopFilter.
 * @details This is a C++ band stop filter used to filter the resonnance frequencies of the rover.
 */

#include "Dsp.hpp"



// =====================================================
// FILTER
// =====================================================

void Dsp::create_filter(float f_center_p, float bandwidth_p, float fs_p)
{
    // --- Define filter design parameters ---
    // For the prototype design, you choose:
    //   ftype: the analog prototype (e.g., Butterworth)
    //   btype: the filter band type (e.g., band-stop for a notch filter)
    //   format: coefficient format (typically second-order sections (SOS))
    //   order: filter order (try 4 or 6 to start)
    //   fc: cutoff frequency (normalized: 0 < fc < 0.5; 0.5 corresponds to Nyquist frequency)
    //   f0: center frequency of the notch (normalized)
    //   Ap: passband ripple in dB (typically a small value, e.g., 1 dB)

    // =====================================================
    // User-specified parameters (in Hz)
    // =====================================================
    // float f_center  = 27.5f;   // Desired center frequency (notch center)
    // float bandwidth = 5.0f;    // Desired bandwidth of the notch
    // float fs        = 400.0f;  // Sampling frequency (2 times the signal's frequency)

    float f_center = f_center_p;    // Desired center frequency (notch center)
    float bandwidth = bandwidth_p;  // Desired bandwidth of the notch
    float fs = fs_p*2;            // Sampling frequency (*2 because of the Nyquist frequency)

    // =====================================================
    // Compute the lower and upper edges of the notch
    // =====================================================
    float f_low  = f_center - (bandwidth / 2.0f);
    float f_high = f_center + (bandwidth / 2.0f);


    // =====================================================
    // Compute the Nyquist frequency
    // =====================================================
    // Nyquist Criterion:
    // For a signal with a maximum frequency of 200 Hz, you should sample at least at 400 Hz to capture all the frequency information without ambiguity. 
    // Sampling at only 100 Hz is far below this requirement.
    float nyquist = fs / 2.0f;


    // =====================================================
    // Validate frequency ranges
    // =====================================================
    if (f_low <= 0.0f || f_high >= nyquist) {
        cerr << "Error: Frequency parameters out of range.\n"
                  << "f_low = " << f_low << " Hz, f_high = " << f_high 
                  << " Hz, Nyquist = " << nyquist << " Hz\n";
        return;
    }

    // =====================================================
    // Compute normalized frequencies (0 to 1; 1 corresponds to Nyquist)
    // =====================================================
    float fc_norm = f_low / nyquist;        // Normalized cutoff (lower edge) frequency
    float f0_norm = f_center / nyquist;     // Normalized center frequency

    cout << "Normalized lower edge (fc_norm): " << fc_norm << "\n";
    cout << "Normalized center frequency (f0_norm): " << f0_norm << "\n";


    // Create the band-stop (notch) filter.
    // The prototype design interface will compute the necessary second-order sections.
    _filter = iirfilt_crcf_create_prototype(
      LIQUID_IIRDES_BUTTER,                 // Analog prototype type (Butterworth)
      LIQUID_IIRDES_BANDSTOP,               // Filter type: band-stop (notch)
      LIQUID_IIRDES_SOS,                    // Design format: use second-order sections
      order,                                // Filter's order
      fc_norm,                              // Cutoff frequency
      f0_norm,                              // Notch (center) frequency
      Ap,                                   // Passband ripple in dB
      As                                    // Stopband attenuation in dB
    );

    if (!_filter) {
      cerr <<"Failed to create IIR filter!"<< endl;
      return;
    }
};


void Dsp::process_sample(complex<float> input, complex<float> *output)
{
    // Execute one filtering iteration.
    iirfilt_crcf_execute(_filter, input, output);
};

// =====================================================
// TOOLS
// =====================================================

// Generate a fake sinusoid signal
void Dsp::generate_simulated_signal()
{
    // Parameters for the sinusoid.
    double amplitude = 1.0;       // Amplitude of the sinusoid.
    double frequency = 10.0;      // Frequency in Hz.
    double fs = 1000.0;           // Sampling frequency in Hz.
    double duration = 1.0;        // Duration in seconds.

    // Frequencies (in Hz) and their amplitudes for the sinusoids.
    double f1 = 10.0, f2 = 26.0, f3 = 35.0, f4 = 30.0, f5 = 20.0;
    double a1 = 1.0, a2 = 1.0, a3 = 1.0, a4 = 1.0, a5 = 1.0;

    // Calculate the total number of samples.
    int N = static_cast<int>(fs * duration);

    // Create a vector to store the samples.
    sinusoid.reserve(N);

    // Generate the sinusoid.
    for (int n = 0; n < N; ++n) {
        double t = n / fs; // Current time in seconds.
        double sample = a1 * std::sin(2 * M_PI * f1 * t) +
                        a2 * std::sin(2 * M_PI * f2 * t) +
                        a3 * std::sin(2 * M_PI * f3 * t) +
                        a4 * std::sin(2 * M_PI * f4 * t) +
                        a5 * std::sin(2 * M_PI * f5 * t)
                        ;
        sinusoid.push_back(sample);
    }
};
