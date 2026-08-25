#include "ABA_fixed.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>

namespace
{
constexpr std::size_t kSampleCount = 1000;

std::filesystem::path default_input_path()
{
#ifdef ABA_PROJECT_ROOT
    return std::filesystem::path(ABA_PROJECT_ROOT) / "in_txt" / "aba_input.txt";
#else
    return std::filesystem::path("in_txt") / "aba_input.txt";
#endif
}
} // namespace

int main(int argc, char* argv[])
{
    const std::filesystem::path output_path =
        argc > 1 ? std::filesystem::path(argv[1]) : default_input_path();

    std::error_code error;
    if (!output_path.parent_path().empty())
    {
        std::filesystem::create_directories(output_path.parent_path(), error);
        if (error)
        {
            std::cerr << "Cannot create input directory: " << error.message() << '\n';
            return 1;
        }
    }

    std::ofstream output(output_path);
    if (!output)
    {
        std::cerr << "Cannot open output file: " << output_path << '\n';
        return 1;
    }

    // 固定随机种子保证每次生成相同的测试数据，便于对比计算结果和速度。
    std::mt19937 generator(20260824U);
    // Random generation is host-only. Generate as double, then quantize once
    // through data_t so the file exactly represents accelerator inputs.
    std::uniform_real_distribution<double> q_distribution(-3.14159265, 3.14159265);
    std::uniform_real_distribution<double> dq_distribution(-2.0, 2.0);
    std::uniform_real_distribution<double> tau_distribution(-10.0, 10.0);

    output << std::setprecision(9);
    for (std::size_t sample = 0; sample < kSampleCount; ++sample)
    {
        bool first_value = true;
        const auto write_value = [&](const auto& value) {
            if (!first_value)
            {
                output << ' ';
            }
            output << value;
            first_value = false;
        };

        for (int joint = 0; joint < DOF; ++joint)
        {
            write_value(joint_pos_t(q_distribution(generator)));
        }
        for (int joint = 0; joint < DOF; ++joint)
        {
            write_value(joint_vel_t(dq_distribution(generator)));
        }
        for (int joint = 0; joint < DOF; ++joint)
        {
            write_value(joint_tau_t(tau_distribution(generator)));
        }
        output << '\n';
    }

    if (!output)
    {
        std::cerr << "Failed while writing input file: " << output_path << '\n';
        return 1;
    }

    std::cout << "Generated " << kSampleCount << " input rows: " << output_path << '\n';
    std::cout << "Each row contains q[6], dq[6], tau[6].\n";
    return 0;
}
