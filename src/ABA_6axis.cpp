#include <iostream>
#include <iomanip>
#include <chrono>
#include <ABA_HLS.h>

const data_t PI = 3.14159265358979323846f;

int main()
{
    // 定义关节角 q，单位：rad
    data_t q[DOF] = {
        0.1,
        -0.2,
        0.3,
        -0.1,
        0.2,
        0.0
    };

    // 定义关节速度 dq，单位：rad/s
    data_t dq[DOF] = {
        0,
        0,
        0,
        0,
        0,
        0
    };

    // 定义关节力矩 tau，单位：N·m
    data_t tau[DOF] = {
        0.1,
        0.2,
        0.3,
        0.4,
        0.5,
        0.6
    };

    data_t ddq[DOF];

    // ==========================================
    // 单次 ABA 计算时间测试
    // ==========================================
    auto start = std::chrono::high_resolution_clock::now();

    ABA(q, dq, tau, ddq);

    auto end = std::chrono::high_resolution_clock::now();

    // 纳秒
    auto duration_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            end - start
        ).count();

    // 微秒
    double duration_us = duration_ns / 1000.0;

    // ==========================================
    // 打印 ABA 计算结果
    // ==========================================
    std::cout << std::fixed << std::setprecision(9);

    std::cout << "================ ABA Result ================" << std::endl;

    for (int i = 0; i < DOF; i++)
    {
        std::cout << "Joint " << i + 1 << std::endl;
        std::cout << "ddq[" << i << "] = "
                  << std::setw(15)
                  << ddq[i]
                  << " rad/s^2"
                  << std::endl;
    }

    std::cout << "============================================" << std::endl;

    // ==========================================
    // 打印计算时间
    // ==========================================
    std::cout << "ABA computation time:" << std::endl;

    std::cout << "    "
              << duration_ns
              << " ns"
              << std::endl;

    std::cout << "    "
              << duration_us
              << " us"
              << std::endl;

    std::cout << "============================================" << std::endl;

    return 0;
}