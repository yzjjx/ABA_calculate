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

