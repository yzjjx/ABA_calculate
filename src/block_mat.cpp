#include <block_mat.h>

// 用来进行（空间）速度矩阵的计算，使用了空间叉乘矩阵
void v_mat_cal(
    const data_t E[3][3],
    const data_t F[3][3],
    const data_t in[6],
    data_t out[6])
{
    for (int r = 0; r < 3; r++)
    {
        // 上半部分：E * 角速度
        out[r] =
            E[r][0] * in[0] +
            E[r][1] * in[1] +
            E[r][2] * in[2];

        // 下半部分：F * 角速度 + E * 线速度
        out[r + 3] =
            F[r][0] * in[0] +
            F[r][1] * in[1] +
            F[r][2] * in[2] +
            E[r][0] * in[3] +
            E[r][1] * in[4] +
            E[r][2] * in[5];
    }
}

// 用来进行空间力矩阵的块矩阵计算
void f_mat_cal(
    const data_t E[3][3],
    const data_t F[3][3],
    const data_t in[6],
    data_t out[6])
{
    for (int r = 0; r < 3; r++)
    {
        out[r] =
            E[0][r] * in[0] +
            E[1][r] * in[1] +
            E[2][r] * in[2] +
            F[0][r] * in[3] +
            F[1][r] * in[4] +
            F[2][r] * in[5];

        out[r + 3] =
            E[0][r] * in[3] +
            E[1][r] * in[4] +
            E[2][r] * in[5];
    }
}

// 点积计算函数
static data_t dot3(
    const data_t a0,
    const data_t a1,
    const data_t a2,
    const data_t b0,
    const data_t b1,
    const data_t b2
)
{

    const data_t p0 = a0 * b0;
    const data_t p1 = a1 * b1;
    const data_t p2 = a2 * b2;

    return (p0 + p1) + p2;
}

// 空间惯量大型矩阵转换
void transform_inertia_ef(
    const data_t E[3][3],
    const data_t F[3][3],
    const data_t Ia[6][6],
    data_t result[6][6]
)
{

    /*
     * IX = Ia * X
     *
     * X = [E 0]
     *     [F E]
     */
    data_t IX[6][6];

    /*
     * X的左三列：
     *
     * X[:,0:3] = [E]
     *            [F]
     *
     * 所以：
     *
     * IX[row][col]
     * = Ia[row][0:3] * E[:,col]
     * + Ia[row][3:6] * F[:,col]
     */
    for (int row = 0; row < 6; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            const data_t part_E = dot3(
                Ia[row][0],
                Ia[row][1],
                Ia[row][2],
                E[0][col],
                E[1][col],
                E[2][col]
            );

            const data_t part_F = dot3(
                Ia[row][3],
                Ia[row][4],
                Ia[row][5],
                F[0][col],
                F[1][col],
                F[2][col]
            );

            IX[row][col] = part_E + part_F;
        }
    }

    /*
     * X的右三列：
     *
     * X[:,3:6] = [0]
     *            [E]
     *
     * 所以：
     *
     * IX[row][col+3]
     * = Ia[row][3:6] * E[:,col]
     */
    for (int row = 0; row < 6; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            IX[row][col + 3] = dot3(
                Ia[row][3],
                Ia[row][4],
                Ia[row][5],
                E[0][col],
                E[1][col],
                E[2][col]
            );
        }
    }

    /*
     * result左上角3×3：
     *
     * result[a][b]
     * = [E[:,a];F[:,a]]^T * IX[:,b]
     *
     * 只计算上三角，然后复制到下三角。
     */
    for (int a = 0; a < 3; a++)
    {
        for (int b = a; b < 3; b++)
        {
            const data_t part_E = dot3(
                E[0][a],
                E[1][a],
                E[2][a],
                IX[0][b],
                IX[1][b],
                IX[2][b]
            );

            const data_t part_F = dot3(
                F[0][a],
                F[1][a],
                F[2][a],
                IX[3][b],
                IX[4][b],
                IX[5][b]
            );

            const data_t value = part_E + part_F;

            result[a][b] = value;
            result[b][a] = value;
        }
    }

    /*
     * result右上角3×3：
     *
     * 左侧列来自 [E;F]
     * 右侧列来自 [0;E]
     */
    for (int a = 0; a < 3; a++)
    {
        for (int b = 0; b < 3; b++)
        {
            const data_t part_E = dot3(
                E[0][a],
                E[1][a],
                E[2][a],
                IX[0][b + 3],
                IX[1][b + 3],
                IX[2][b + 3]
            );

            const data_t part_F = dot3(
                F[0][a],
                F[1][a],
                F[2][a],
                IX[3][b + 3],
                IX[4][b + 3],
                IX[5][b + 3]
            );

            const data_t value = part_E + part_F;

            result[a][b + 3] = value;

            /*
             * 惯量变换结果对称，
             * 左下角直接由右上角镜像得到。
             */
            result[b + 3][a] = value;
        }
    }

    /*
     * result右下角3×3：
     *
     * 左、右两侧都只剩E块。
     * 同样只计算上三角。
     */
    for (int a = 0; a < 3; a++)
    {
        for (int b = a; b < 3; b++)
        {
            const data_t value = dot3(
                E[0][a],
                E[1][a],
                E[2][a],
                IX[3][b + 3],
                IX[4][b + 3],
                IX[5][b + 3]
            );

            result[a + 3][b + 3] = value;
            result[b + 3][a + 3] = value;
        }
    }
}