/**
 * @file Dsp.hpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2025-01-26
 * 
 * @copyright Gabriel Garcia | 2024
 * @brief Implementation file for Class BandStopFilter.
 * @details This is a C++ band stop filter used to filter the resonnance frequencies of the rover.
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip> // For better output formatting
#include <algorithm> // For min/max element
#include <deque>
#include <numeric>
#include <complex>

// Liquid-DSP
#include <liquid/liquid.h>


using namespace std;


class Dsp {
    public:
        Dsp(){};
        ~Dsp()
        {
            if (_filter)
                iirfilt_crcf_destroy(_filter);
        };

        // ===========================
        // Attributes
        // ===========================
        // The sinusoid attribute is used for simulating IMU data (debug)
        vector<double> sinusoid;

        // Liquid-DSP filter object (pointer type defined by Liquid-DSP)
        iirfilt_crcf _filter {nullptr};

        // ===========================
        // Methods
        // ===========================
        void create_filter(float f_center_p, float bandwidth_p, float fs_p);
        void process_sample(complex<float> input, complex<float> *output);
        void generate_simulated_signal();


    protected:
        // ===========================
        // Attributes
        // ===========================

        // Filter design parameters
        unsigned int order = 4;   // Filter order
        float Ap = 1.0f;          // Passband ripple in dB
        float As = 40.0f;         // Stopband attenuation in dB
};