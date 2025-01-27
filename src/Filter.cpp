/**
 * @file filter.cpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2025-01-26
 * 
 * @copyright Gabriel Garcia | 2024
 * @brief Implementation file for Class BandStopFilter.
 * @details This is a C++ band stop filter used to filter the resonnance frequencies of the rover.
 */
#include "Filter.hpp"


void BandStopFilter::initialize(const std::vector<double>& feedforward, const std::vector<double>& feedback)        
{
    b = feedforward;
    a = feedback;
    inputHistory.resize(b.size(), 0.0);
    outputHistory.resize(a.size(), 0.0);
    
    if (a.empty() || b.empty()) {
        throw std::invalid_argument("Filter coefficients cannot be empty.");
    }
    if (std::fabs(a[0] - 1.0) > 1e-10) {
        normalizeCoefficients();
    }
};

// Filter a single sample
double BandStopFilter::filterSample(double input) {
    inputHistory.push_front(input);
    inputHistory.pop_back();

    double output = 0.0;

    // Apply feedforward (b coefficients)
    for (size_t i = 0; i < b.size(); ++i) {
        output += b[i] * inputHistory[i];
    }

    // Apply feedback (a coefficients)
    for (size_t i = 1; i < a.size(); ++i) {  // Skip a[0]
        output -= a[i] * outputHistory[i];
    }

    // Update output history
    outputHistory.push_front(output);
    outputHistory.pop_back();

    return output;
};



void BandStopFilter::normalizeCoefficients() {
    double normalizationFactor = a[0];
    for (double& coeff : b) coeff /= normalizationFactor;
    for (double& coeff : a) coeff /= normalizationFactor;
};
