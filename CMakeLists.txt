cmake_minimum_required(VERSION 3.10)
project(ABA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(ABA
    src/ABA_6axis.cpp
    src/ABA_HLS.cpp
    src/T_R_out.cpp
    src/v_ori.cpp
    src/c.cpp
    src/cal_p.cpp
    src/back.cpp
    src/pass3.cpp
    src/ABA_parms.cpp
    src/block_mat.cpp

    # #================================
    # #单独计算空间惯性矩阵
    # src/I_space_out.cpp
    # src/I_space.cpp
    # src/ABA_parms.cpp
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(generate_input
    src/generate_input.cpp
)

add_executable(ABA_benchmark
    src/ABA_benchmark.cpp
    src/ABA_HLS.cpp
    src/T_R_out.cpp
    src/v_ori.cpp
    src/c.cpp
    src/cal_p.cpp
    src/back.cpp
    src/pass3.cpp
    src/ABA_parms.cpp
    src/block_mat.cpp
)

# 让程序从 build 目录运行时也能找到工程内的 in_txt/out_txt。
target_compile_definitions(generate_input PRIVATE
    ABA_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_compile_definitions(ABA_benchmark PRIVATE
    ABA_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}"
)

