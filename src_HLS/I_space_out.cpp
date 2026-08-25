// 该代码用来生成空间惯量矩阵
#include <iostream>
#include <iomanip>
#include "ABA_fixed.h"

int main()
{
    inertia_t I_spa[DOF][6][6];

    I_space(
        mass,
        c_of_mass,
        I,
        I_spa
    );

    // 输出 I_spa
    std::cout << std::fixed << std::setprecision(13);

    for (int k = 0; k < DOF; k++)
    {
        std::cout << "========================================" << std::endl;
        std::cout << "I_spa[" << k << "] =" << std::endl;
        std::cout << "========================================" << std::endl;

        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                std::cout << std::setw(16)
                          << I_spa[k][i][j]
                          << ", ";
            }

            std::cout << std::endl;
        }

        std::cout << std::endl;
    }

    return 0;
}
