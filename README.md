该代码用于实现ABA计算的重构  
利用关节运动子空间 S 的稀疏性，将乘积为0的计算去掉，在src\back.cpp中：
```c++
// 计算U
for(int m = 0;m<6;m++)
{
    U[i][m] = I_A[i][m][2]*S[2];
}
```

删除无意义的计算：
```c++
    // // 调用函数
    // cal_R(alpha, a, d, q, T, R, R_Trans, P);

    // 计算v_J
    cal_v_J(dq,v_J);
```

将代码中的所有除法转换为乘法
```c++
// 将除法转换为乘法
inv_D[i] = 1.0f / D[i];

// 计算I_a
for(int ii = 0;ii<6;ii++)
{
    I_a[i][ii][0] = I_A[i][ii][0]-U[i][ii]*U[i][0]*inv_D[i];
    I_a[i][ii][1] = I_A[i][ii][1]-U[i][ii]*U[i][1]*inv_D[i];
    I_a[i][ii][2] = I_A[i][ii][2]-U[i][ii]*U[i][2]*inv_D[i];
    I_a[i][ii][3] = I_A[i][ii][3]-U[i][ii]*U[i][3]*inv_D[i];
    I_a[i][ii][4] = I_A[i][ii][4]-U[i][ii]*U[i][4]*inv_D[i];
    I_a[i][ii][5] = I_A[i][ii][5]-U[i][ii]*U[i][5]*inv_D[i];
}
```

将计算中一定为0的步骤设置为0
```cpp
void c_fina(
    const data_t v[DOF][6],
    const data_t v_J[6][DOF],
    data_t c[DOF][6]
)
{   
    for(int i = 0;i < DOF;i++)
    {
        // 1. 提取 v_J 的第 i 列（第 i 个关节的 6D 速度）
        data_t current_v_J[6];
        current_v_J[0] = v_J[0][i];
        current_v_J[1] = v_J[1][i];
        current_v_J[2] = v_J[2][i];
        current_v_J[3] = v_J[3][i];
        current_v_J[4] = v_J[4][i];
        current_v_J[5] = v_J[5][i];

        space_cross(v[i], current_v_J, c[i]);
    }
}
```
修改后为：
```cpp
void c_fina(
    const data_t v[DOF][6],
    const data_t dq[DOF],
    data_t c[DOF][6])
{
    for (int i = 0; i < DOF; i++)
    {
        const data_t wi = dq[i];

        c[i][0] =  v[i][1] * wi;
        c[i][1] = -v[i][0] * wi;
        c[i][2] =  0.0f;

        c[i][3] =  v[i][4] * wi;
        c[i][4] = -v[i][3] * wi;
        c[i][5] =  0.0f;
    }
}
```

函数优化，舍弃多余二维数组；舍弃结果一定为0的计算
```c++
void tip_pass3(
    const data_t X_lam[DOF][6][6],
    const data_t c[DOF][6],
    const data_t D[DOF],
    const data_t u[DOF],
    const data_t U[DOF][6],
    data_t ddq[DOF]
)
{
    data_t S[6] = {0, 0, 1, 0, 0, 0};
    data_t a_parent[6] = {0, 0, 0, 0, 0, 9.81};

    data_t a_s[DOF][6];
    data_t a_a[DOF][6];

    for(int i = 0;i<DOF;i++)
    {
        for(int j = 0;j<6;j++)
        {
            a_s[i][j] =  X_lam[i][j][0]*a_parent[0]+X_lam[i][j][1]*a_parent[1]
                        +X_lam[i][j][2]*a_parent[2]+X_lam[i][j][3]*a_parent[3]
                        +X_lam[i][j][4]*a_parent[4]+X_lam[i][j][5]*a_parent[5]
                        + c[i][j];
        }
        
        ddq[i] = (u[i]-(U[i][0]*a_s[i][0]+U[i][1]*a_s[i][1]
                  +U[i][2]*a_s[i][2]+U[i][3]*a_s[i][3]
                  +U[i][4]*a_s[i][4]+U[i][5]*a_s[i][5]))/D[i];
                  
        for(int k = 0;k<6;k++)
        {
            a_a[i][k] = a_s[i][k] + S[k]*ddq[i];
            a_parent[k] = a_a[i][k];
        }
    }
}
```
1、因为a_s和a_i、a_parent计算不需要二维数组，因此更换为一维
2、将括号内的计算单独提出来，将除法转换为乘法
3、舍弃一定为0的计算结果，重新写循环ddq
## 修改2：彻底删除X_lam
因为公式为:
$$
 X_{\lambda1} = \ ^1X_0=\begin{bmatrix} R_1^T &0 \\

R_1^T[p_1]_{\times} & R_1^T \end{bmatrix} 
$$
可以看到左上角矩阵与右下角矩阵是一个矩阵，定义：
$$
E =  R_1^T   
F =  R_1^T[p_1]_{\times}
$$
因此整理为紧凑格式，即保存E[3][3]和F[3][3]即可
因为空间速度向量可以分为两个三维部分，因此矩阵乘法可以使用块矩阵乘法，也就是：
$$
\begin{bmatrix}\omega _{new}
 \\
v _{new}
\end{bmatrix}=\begin{bmatrix}
 E & 0\\
F  &E
\end{bmatrix}\ast \begin{bmatrix}\omega
 \\
v 
\end{bmatrix}=\begin{bmatrix}
 E*\omega\\
F*\omega+E*v
\end{bmatrix}
$$

删除所有不用或者重复使用的函数、变量  
一次通用的6*6矩阵乘法包含216次乘法，因此所有矩阵简化为块矩阵计算  
目前代码中最大头的计算为
```c++
data_t out_1[DOF][6][6];
data_t out_fin[DOF][6][6];
mat_6x6(X_lam_Trans[i+1],I_a[i+1],out_1[i+1]);
mat_6x6(out_1[i+1],X_lam[i+1],out_fin[i+1]);
```
也就是：
$$
out_{fina}=X^T*I_a*X
$$
因为矩阵X含有零矩阵的部分，因此可以分为两部分：
$$
\left\{\begin{matrix}I_x=I_a*X
 \\
out_{fina}=X^T*I_x
\end{matrix}\right.
$$
空间惯量矩阵Ia为对称矩阵，因此结果也是对称的

在修改完这部分以后，计算速度为15us一次

src\back.cpp:进行后向递推的滚动递推优化，因为反向递推计算关节 i 时，只需要关节 i+1 的结果，不需要同时保存所有六个关节的 I_A、I_a、p_A、p_a，因此可以用一组“当前关节变量”和一组“子关节传播变量”反复覆盖使用，旧版代码需要：  
I_A：6×6×6 = 216个float  
I_a：6×6×6 = 216个float  
p_A：6×6   = 36个float  
p_a：6×6   = 36个float  

合计：504个float  
新版代码滚动版本可以优化为：  
IA_cur：36  
Ia_cur：36  
pA_cur：6  
pa_cur：6  
child_I：36  
child_p：6  

合计：126个float  

去除一定为0的计算结果，例如在back.cpp中：
```c++
const data_t Iac =
    Ia_cur[r][0] * c[i][0] +
    Ia_cur[r][1] * c[i][1] +
    Ia_cur[r][2] * c[i][2] +
    Ia_cur[r][3] * c[i][3] +
    Ia_cur[r][4] * c[i][4] +
    Ia_cur[r][5] * c[i][5];
```
其中从c.cpp中可以得到：
```c++
c[i][0] =  v[i][1] * wi;
c[i][1] = -v[i][0] * wi;
c[i][2] =  0.0f;

c[i][3] =  v[i][4] * wi;
c[i][4] = -v[i][3] * wi;
c[i][5] =  0.0f;
```
因此可以更改为：
```c++
const data_t s0 =
    Ia_cur[r][0] * c[i][0] +
    Ia_cur[r][1] * c[i][1];

const data_t s1 =
    Ia_cur[r][3] * c[i][3] +
    Ia_cur[r][4] * c[i][4];

const data_t Iac = s0 + s1;
```
单次乘法从6降低为4，总计36次计算降低为24次