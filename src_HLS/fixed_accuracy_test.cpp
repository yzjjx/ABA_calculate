#include "ABA_fixed.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

void ABA_float(
    const float q[DOF],
    const float dq[DOF],
    const float tau[DOF],
    float ddq[DOF]);

namespace
{
struct ErrorStats
{
    double absolute_sum = 0.0;
    double squared_sum = 0.0;
    double relative_sum = 0.0;
    double max_absolute = 0.0;
    double max_relative = 0.0;
    double max_actual_magnitude = 0.0;
    double max_reference_magnitude = 0.0;
    std::size_t relative_count = 0;
    std::size_t count = 0;
    std::vector<double> absolute_errors;

    void add(double actual, double reference)
    {
        const double absolute = std::abs(actual - reference);
        absolute_sum += absolute;
        squared_sum += absolute * absolute;
        max_absolute = std::max(max_absolute, absolute);
        max_actual_magnitude = std::max(max_actual_magnitude, std::abs(actual));
        max_reference_magnitude =
            std::max(max_reference_magnitude, std::abs(reference));
        absolute_errors.push_back(absolute);
        ++count;

        // Relative error is not meaningful around zero. The 1.0 floor also
        // makes this a useful normalized error for low-acceleration joints.
        const double relative = absolute / std::max(std::abs(reference), 1.0);
        relative_sum += relative;
        max_relative = std::max(max_relative, relative);
        ++relative_count;
    }

    double mean_absolute() const
    {
        return absolute_sum / static_cast<double>(count);
    }

    double root_mean_square() const
    {
        return std::sqrt(squared_sum / static_cast<double>(count));
    }

    double mean_normalized() const
    {
        return relative_sum / static_cast<double>(relative_count);
    }

    double percentile(double fraction)
    {
        std::sort(absolute_errors.begin(), absolute_errors.end());
        const std::size_t index = static_cast<std::size_t>(
            fraction * static_cast<double>(absolute_errors.size() - 1));
        return absolute_errors[index];
    }
};

bool read_state(std::istream& input, std::array<double, DOF>& q,
                std::array<double, DOF>& dq, std::array<double, DOF>& tau)
{
    for (double& value : q)
    {
        if (!(input >> value))
        {
            return false;
        }
    }
    for (double& value : dq)
    {
        input >> value;
    }
    for (double& value : tau)
    {
        input >> value;
    }
    return static_cast<bool>(input);
}

void evaluate_state(
    const std::array<double, DOF>& q_double,
    const std::array<double, DOF>& dq_double,
    const std::array<double, DOF>& tau_double,
    ErrorStats& total,
    std::array<ErrorStats, DOF>& joint_stats)
{
    joint_pos_t q[DOF];
    joint_vel_t dq[DOF];
    joint_tau_t tau[DOF];
    data_t ddq[DOF];
    float q_float[DOF];
    float dq_float[DOF];
    float tau_float[DOF];
    float ddq_float[DOF];

    for (int joint = 0; joint < DOF; ++joint)
    {
        q[joint] = q_double[joint];
        dq[joint] = dq_double[joint];
        tau[joint] = tau_double[joint];
        q_float[joint] = static_cast<float>(q_double[joint]);
        dq_float[joint] = static_cast<float>(dq_double[joint]);
        tau_float[joint] = static_cast<float>(tau_double[joint]);
    }

    ABA(q, dq, tau, ddq);
    ABA_float(q_float, dq_float, tau_float, ddq_float);

    for (int joint = 0; joint < DOF; ++joint)
    {
        const double actual = ddq[joint].to_double();
        const double expected = ddq_float[joint];
        total.add(actual, expected);
        joint_stats[joint].add(actual, expected);
    }
}
} // namespace

int main(int argc, char* argv[])
{
    const std::string input_path =
        argc > 1 ? argv[1] : "in_txt/aba_input.txt";
    std::ifstream input(input_path);
    if (!input)
    {
        std::cerr << "Cannot open input file: " << input_path << '\n';
        return 1;
    }

    ErrorStats total;
    std::array<ErrorStats, DOF> joint_stats;
    std::array<double, DOF> q_double{};
    std::array<double, DOF> dq_double{};
    std::array<double, DOF> tau_double{};
    std::size_t sample_count = 0;

    while (read_state(input, q_double, dq_double, tau_double))
    {
        evaluate_state(q_double, dq_double, tau_double, total, joint_stats);
        ++sample_count;
    }

    if (sample_count == 0)
    {
        std::cerr << "No input samples were read.\n";
        return 4;
    }

    ErrorStats edge_total;
    std::array<ErrorStats, DOF> edge_joint_stats;
    std::size_t edge_sample_count = 0;
    constexpr double kPi = 3.14159265358979323846;

    // Exact zero plus every q sign combination at the stated deployment
    // limits. dq and tau use different sign permutations to exercise mixed
    // Coriolis and torque directions.
    q_double.fill(0.0);
    dq_double.fill(0.0);
    tau_double.fill(0.0);
    evaluate_state(q_double, dq_double, tau_double,
                   edge_total, edge_joint_stats);
    ++edge_sample_count;

    for (unsigned mask = 0; mask < (1U << DOF); ++mask)
    {
        for (int joint = 0; joint < DOF; ++joint)
        {
            const double sign_q = ((mask >> joint) & 1U) ? 1.0 : -1.0;
            const double sign_dq = ((mask >> ((joint + 1) % DOF)) & 1U)
                                       ? 1.0 : -1.0;
            const double sign_tau = ((mask >> ((joint + 3) % DOF)) & 1U)
                                        ? 1.0 : -1.0;
            q_double[joint] = sign_q * kPi;
            dq_double[joint] = sign_dq * 2.0;
            tau_double[joint] = sign_tau * 10.0;
        }
        evaluate_state(q_double, dq_double, tau_double,
                       edge_total, edge_joint_stats);
        ++edge_sample_count;
    }

    std::cout << std::fixed << std::setprecision(9);
    std::cout << "fixed_format ap_fixed<" << ABA_DATA_W << ',' << ABA_DATA_I
              << ",AP_RND_CONV,AP_SAT>\n";
    std::cout << "input_formats q<32,4> dq<32,4> tau<32,8>\n";
    std::cout << "internal_formats transform<22,2> kinematic<28,8>"
                 " inertia<32,8> force<32,12> inverse<34,14>\n";
    std::cout << "samples " << sample_count << '\n';
    std::cout << "values " << total.count << '\n';
    std::cout << "mean_abs_error " << total.mean_absolute() << '\n';
    std::cout << "rmse " << total.root_mean_square() << '\n';
    std::cout << "p95_abs_error " << total.percentile(0.95) << '\n';
    std::cout << "p99_abs_error " << total.percentile(0.99) << '\n';
    std::cout << "max_abs_error " << total.max_absolute << '\n';
    std::cout << "mean_normalized_error " << total.mean_normalized() << '\n';
    std::cout << "max_normalized_error " << total.max_relative << '\n';
    std::cout << "max_fixed_output_magnitude "
              << total.max_actual_magnitude << '\n';
    std::cout << "max_reference_output_magnitude "
              << total.max_reference_magnitude << '\n';

    for (int joint = 0; joint < DOF; ++joint)
    {
        ErrorStats& stats = joint_stats[joint];
        std::cout << "joint " << (joint + 1)
                  << " mae " << stats.mean_absolute()
                  << " rmse " << stats.root_mean_square()
                  << " p99 " << stats.percentile(0.99)
                  << " max " << stats.max_absolute
                  << " mean_norm " << stats.mean_normalized() << '\n';
    }


    std::cout << "edge_samples " << edge_sample_count << '\n';
    std::cout << "edge_mean_abs_error " << edge_total.mean_absolute() << '\n';
    std::cout << "edge_rmse " << edge_total.root_mean_square() << '\n';
    std::cout << "edge_p99_abs_error " << edge_total.percentile(0.99) << '\n';
    std::cout << "edge_max_abs_error " << edge_total.max_absolute << '\n';
    std::cout << "edge_mean_normalized_error "
              << edge_total.mean_normalized() << '\n';
    std::cout << "edge_max_fixed_output_magnitude "
              << edge_total.max_actual_magnitude << '\n';
    std::cout << "edge_max_reference_output_magnitude "
              << edge_total.max_reference_magnitude << '\n';

    return 0;
}
