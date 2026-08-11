#include <iostream>
#include <iomanip>
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

    // 定义关节角加速度 dq，单位：rad
    data_t dq[DOF] = {
        0,
        0,
        0,
        0,
        0,
        0
    };

    data_t tau[DOF] = {
        0.1,
        0.2,
        0.3,
        0.4,
        0.5,
        0.6
    };

    data_t ddq[DOF];

    ABA(q,dq,tau,ddq);

    for(int i=0;i<DOF;i++)
    {
        std::cout << "Joint " << i + 1 << std::endl;
        std::cout << "ddq[" << i << "] =" << std::endl;
        std::cout << std::setw(12) << ddq[i]<< " ";
        std::cout << std::endl;
    }

}