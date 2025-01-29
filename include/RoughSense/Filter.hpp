/**
 * @file Filter.hpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2025-01-26
 * 
 * @copyright Gabriel Garcia | 2024
 * @brief Implementation file for Class BandStopFilter.
 * @details This is a C++ band stop filter used to filter the resonnance frequencies of the rover.
 */

#include <vector>
#include <deque>
#include <cmath>
#include <stdexcept>



class BandStopFilter {
    public:
        BandStopFilter(){};
        ~BandStopFilter(){};

        // Filter a single sample
        double filterSample(double input); 
        void initialize(const std::vector<double>& feedforward, const std::vector<double>& feedback);

    protected:
        std::vector<double> b;  // Feedforward coefficients
        std::vector<double> a;  // Feedback coefficients
        std::deque<double> inputHistory;  // Input history buffer
        std::deque<double> outputHistory; // Output history buffer

        void normalizeCoefficients();
};