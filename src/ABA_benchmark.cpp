#include <ABA_HLS.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace
{
constexpr std::size_t kSampleCount = 1000;

struct AbaInput
{
    std::array<data_t, DOF> q{};
    std::array<data_t, DOF> dq{};
    std::array<data_t, DOF> tau{};
};

using AbaOutput = std::array<data_t, DOF>;

std::filesystem::path project_path(const char* relative_path)
{
#ifdef ABA_PROJECT_ROOT
    return std::filesystem::path(ABA_PROJECT_ROOT) / relative_path;
#else
    return std::filesystem::path(relative_path);
#endif
}

bool read_input_file(const std::filesystem::path& path, std::vector<AbaInput>& inputs)
{
    std::ifstream input_file(path);
    if (!input_file)
    {
        std::cerr << "Cannot open input file: " << path << '\n';
        return false;
    }

    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input_file, line))
    {
        ++line_number;
        if (line.empty())
        {
            std::cerr << "Empty input row at line " << line_number << '\n';
            return false;
        }

        AbaInput sample;
        std::istringstream row(line);
        for (data_t& value : sample.q)
        {
            if (!(row >> value))
            {
                std::cerr << "Invalid q data at line " << line_number << '\n';
                return false;
            }
        }
        for (data_t& value : sample.dq)
        {
            if (!(row >> value))
            {
                std::cerr << "Invalid dq data at line " << line_number << '\n';
                return false;
            }
        }
        for (data_t& value : sample.tau)
        {
            if (!(row >> value))
            {
                std::cerr << "Invalid tau data at line " << line_number << '\n';
                return false;
            }
        }

        std::string extra_value;
        if (row >> extra_value)
        {
            std::cerr << "Too many values at line " << line_number
                      << "; expected exactly 18 values.\n";
            return false;
        }
        inputs.push_back(sample);
    }

    if (inputs.size() != kSampleCount)
    {
        std::cerr << "Input file must contain exactly " << kSampleCount
                  << " rows, but found " << inputs.size() << ".\n";
        return false;
    }
    return true;
}

bool write_outputs(const std::filesystem::path& path,
                   const std::vector<AbaOutput>& outputs)
{
    std::error_code error;
    if (!path.parent_path().empty())
    {
        std::filesystem::create_directories(path.parent_path(), error);
        if (error)
        {
            std::cerr << "Cannot create output directory: " << error.message() << '\n';
            return false;
        }
    }

    std::ofstream output_file(path);
    if (!output_file)
    {
        std::cerr << "Cannot open output file: " << path << '\n';
        return false;
    }

    output_file << std::setprecision(9);
    for (const AbaOutput& output : outputs)
    {
        for (int joint = 0; joint < DOF; ++joint)
        {
            if (joint != 0)
            {
                output_file << ' ';
            }
            output_file << output[joint];
        }
        output_file << '\n';
    }
    return static_cast<bool>(output_file);
}

bool write_timing(const std::filesystem::path& path,
                  double total_us,
                  double average_us,
                  double calculations_per_second)
{
    std::ofstream timing_file(path);
    if (!timing_file)
    {
        std::cerr << "Cannot open timing file: " << path << '\n';
        return false;
    }

    timing_file << std::fixed << std::setprecision(6)
                << "sample_count " << kSampleCount << '\n'
                << "total_us " << total_us << '\n'
                << "average_us " << average_us << '\n'
                << "calculations_per_second " << calculations_per_second << '\n';
    return static_cast<bool>(timing_file);
}
} // namespace

int main(int argc, char* argv[])
{
    const std::filesystem::path input_path =
        argc > 1 ? std::filesystem::path(argv[1]) : project_path("in_txt/aba_input.txt");
    const std::filesystem::path output_path =
        argc > 2 ? std::filesystem::path(argv[2]) : project_path("out_txt/aba_output.txt");
    const std::filesystem::path timing_path =
        argc > 3 ? std::filesystem::path(argv[3]) : project_path("out_txt/aba_timing.txt");

    std::vector<AbaInput> inputs;
    inputs.reserve(kSampleCount);
    if (!read_input_file(input_path, inputs))
    {
        return 1;
    }

    std::vector<AbaOutput> outputs(kSampleCount);

    // 预热一次，避免首次调用开销影响 1000 次 ABA 核心计算计时。
    AbaOutput warmup_output{};
    ABA(inputs.front().q.data(), inputs.front().dq.data(), inputs.front().tau.data(),
        warmup_output.data());

    const auto start = std::chrono::steady_clock::now();
    for (std::size_t sample = 0; sample < kSampleCount; ++sample)
    {
        ABA(inputs[sample].q.data(), inputs[sample].dq.data(), inputs[sample].tau.data(),
            outputs[sample].data());
    }
    const auto end = std::chrono::steady_clock::now();

    // 文件写入不在计时范围内，计时只覆盖 1000 次 ABA() 调用。
    const double total_us =
        std::chrono::duration<double, std::micro>(end - start).count();
    const double average_us = total_us / static_cast<double>(kSampleCount);
    const double calculations_per_second =
        static_cast<double>(kSampleCount) * 1000000.0 / total_us;

    if (!write_outputs(output_path, outputs) ||
        !write_timing(timing_path, total_us, average_us, calculations_per_second))
    {
        return 1;
    }

    std::cout << std::fixed << std::setprecision(6)
              << "Input rows:             " << kSampleCount << '\n'
              << "Total ABA time:          " << total_us << " us\n"
              << "Average time per ABA:    " << average_us << " us\n"
              << "ABA calculations/second: " << calculations_per_second << '\n'
              << "Output file:             " << output_path << '\n'
              << "Timing file:             " << timing_path << '\n';
    return 0;
}
