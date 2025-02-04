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

// Function to apply a basic second-order section (SOS) filter
// std::vector<double> applySOSFilter(const std::vector<double>& signal, const std::vector<double>& b, const std::vector<double>& a) 
// {
//     size_t n = signal.size();
//     std::vector<double> filtered_signal(n, 0.0);
//     double x_n_1 = 0.0, x_n_2 = 0.0; // Previous inputs
//     double y_n_1 = 0.0, y_n_2 = 0.0; // Previous outputs

//     for (size_t i = 0; i < n; ++i) {
//         double x_n = signal[i];
//         // Second-order difference equation: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
//         double y_n = b[0] * x_n + b[1] * x_n_1 + b[2] * x_n_2 - a[1] * y_n_1 - a[2] * y_n_2;

//         // Debugging output to trace computation
//         if (i < 20) { // Only print the first 20 iterations for clarity
//             std::cout << "Iteration " << i << ": x_n=" << x_n << ", x_n_1=" << x_n_1 << ", x_n_2=" << x_n_2
//                       << ", y_n_1=" << y_n_1 << ", y_n_2=" << y_n_2 << ", y_n=" << y_n << std::endl;
//         }

//         // Update state variables
//         x_n_2 = x_n_1;
//         x_n_1 = x_n;
//         y_n_2 = y_n_1;
//         y_n_1 = y_n;

//         // Store filtered output
//         filtered_signal[i] = y_n;
//     }

//     return filtered_signal;
// };

// Dsp::~Dsp()
// {
    
// };

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
    // float fs        = 400.0f;  // Sampling frequency

    float f_center = f_center_p;    // Desired center frequency (notch center)
    float bandwidth = bandwidth_p;  // Desired bandwidth of the notch
    float fs = fs_p * 2;            // Sampling frequency (*2 because of the Nyquist frequency)

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
        std::cerr << "Error: Frequency parameters out of range.\n"
                  << "f_low = " << f_low << " Hz, f_high = " << f_high 
                  << " Hz, Nyquist = " << nyquist << " Hz\n";
        return;
    }

    // =====================================================
    // Compute normalized frequencies (0 to 1; 1 corresponds to Nyquist)
    // =====================================================
    float fc_norm = f_low / nyquist;   // Normalized cutoff (lower edge) frequency
    float f0_norm = f_center / nyquist;  // Normalized center frequency

    std::cout << "Normalized lower edge (fc_norm): " << fc_norm << "\n";
    std::cout << "Normalized center frequency (f0_norm): " << f0_norm << "\n";

    // =====================================================
    // Filter design parameters
    // =====================================================
    unsigned int order = 4;   // Filter order
    float Ap = 1.0f;          // Passband ripple in dB
    float As = 40.0f;         // Stopband attenuation in dB

    // Create the band-stop (notch) filter.
    // The prototype design interface will compute the necessary second-order sections.
    _filter = iirfilt_crcf_create_prototype(
      LIQUID_IIRDES_BUTTER,    // analog prototype type (Butterworth)
      LIQUID_IIRDES_BANDSTOP,  // filter type: band-stop (notch)
      LIQUID_IIRDES_SOS,       // design format: use second-order sections
      order,
      fc_norm,                      // cutoff frequency
      f0_norm,                      // notch (center) frequency
      Ap,
      As
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



// double Dsp::processSample(double x_n) 
// {
//         double y_n = b[0] * x_n + b[1] * x_n_1 + b[2] * x_n_2 - a[1] * y_n_1 - a[2] * y_n_2;

//         // Shift past states
//         x_n_2 = x_n_1;
//         x_n_1 = x_n;
//         y_n_2 = y_n_1;
//         y_n_1 = y_n;

//         return y_n;
// };


// Function to compute bandstop filter coefficients
void Dsp::computeBandstopCoefficients()
{
    double f_center = (f_low + f_high) / 2.0;
    double bandwidth = f_high - f_low;
    double omega_center = 2.0 * M_PI * f_center / fs;
    double alpha = std::sin(omega_center) * std::sinh(std::log(2.0) / 2.0 * bandwidth * omega_center / std::sin(omega_center));

    b = {1.0, -2.0 * std::cos(omega_center), 1.0};
    a = {1.0 + alpha, -2.0 * std::cos(omega_center), 1.0 - alpha};

    // Debugging: Print computed coefficients

    // std::cout << "Coefficients before normalization:" << std::endl;
    // std::cout << "b: [" << b[0] << ", " << b[1] << ", " << b[2] << "]" << std::endl;
    // std::cout << "a: [" << a[0] << ", " << a[1] << ", " << a[2] << "]" << std::endl;

    // Normalize coefficients using high precision
    long double a0 = static_cast<long double>(a[0]);
    for (size_t i = 0; i < b.size(); ++i) {
        b[i] = static_cast<double>(static_cast<long double>(b[i]) / a0);
    }
    for (size_t i = 0; i < a.size(); ++i) {
        a[i] = static_cast<double>(static_cast<long double>(a[i]) / a0);
    }

    // Debugging: Print normalized coefficients
    // RCLCPP_INFO(rclcpp::get_logger("roughness_node"), "Coefficients after normalization:");
    // RCLCPP_INFO(rclcpp::get_logger("roughness_node"), "b: [%f, %f, %f]", b[0], b[1], b[2]);
    // RCLCPP_INFO(rclcpp::get_logger("roughness_node"), "a: [%f, %f, %f]", a[0], a[1], a[2]);
    // std::cout << "Coefficients after normalization:" << std::endl;
    // std::cout << "b: [" << b[0] << ", " << b[1] << ", " << b[2] << "]" << std::endl;
    // std::cout << "a: [" << a[0] << ", " << a[1] << ", " << a[2] << "]" << std::endl;
};


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
}


void Dsp::std_update(double x)
{
    if (window.size() == window_size) {
            window.pop_front();  // Remove the oldest value
        }
        window.push_back(x);
};

double Dsp::get_std() const 
{
    if (window.size() < 2) return 0.0;

    double mean = accumulate(window.begin(), window.end(), 0.0) / window.size();
    double variance = 0.0;
    
    for (double x : window) {
        variance += (x - mean) * (x - mean);
    }
    variance /= (window.size() - 1);
    return sqrt(variance);
}