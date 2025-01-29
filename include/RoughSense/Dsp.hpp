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

using namespace std;


class Dsp {
    public:
        Dsp(){};
        ~Dsp(){};

        // Filter a single sample
        vector<double> applySOSFilter();
        double processSample(double x_n);
        void computeBandstopCoefficients();
        void std_update(double x);
        double get_std() const;


    protected:
        // Sampling frequency and filter parameters
        double fs = 1000.0; // Sampling frequency (Hz)
        double f_low = 26.0; // Lower cutoff frequency (Hz)
        double f_high = 30.0; // Upper cutoff frequency (Hz)
        double Q = 15.0; // Quality factor
        vector<double> b;
        vector<double> a;
        double x_n_1, x_n_2; // Previous input samples
        double y_n_1, y_n_2; // Previous output samples
        deque<double> window;
        int window_size = 3;

};