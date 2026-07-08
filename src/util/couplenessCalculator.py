import os
import pickle
import sympy as sp


def generate_coupleness_analysis(
    n, m, g0, g1, x, u, dx, state_to_input, filename="coupleness_matrix.pkl"
):
    """自动计算系统的耦合矩阵 C, C_q, C_p 和 P 矩阵，并输出 LaTeX 以及保存至 pkl 文件。

    参数:
    ----------
    n : int
        状态变量的维度
    m : int
        输入变量的维度
    g0 : sympy.Matrix (n x 1)
        系统的漂移向量场 (Drift vector field)
    g1 : sympy.Matrix (n x m)
        系统的控制输入矩阵 (Control input matrix)
    x : list
        状态变量符号列表
    u : list
        输入变量符号列表
    dx : list
        状态增量量(Delta x)符号列表
    state_to_input : dict
        状态索引到输入控制通道索引的映射 (例如: {2: 0} 表示 dx_3 是由 u_1/u_3 驱动的)。
        如果某个状态没有直接控制输入输入（欠驱动），则不填或映射为 None。
    filename : str
        保存结果的 pickle 文件名
    """
    # 1. 计算总系统动力学 f = g0 + g1 * u
    u_matrix = sp.Matrix(u)
    f = g0 + g1 * u_matrix

    C = sp.Matrix.zeros(n, n)
    C_q = sp.Matrix.zeros(n, n)
    C_p = sp.Matrix.zeros(n, n)
    P = sp.Matrix.zeros(n, n)

    # 内部辅助函数：计算两个向量场的 Lie Bracket [X, Y] = J_Y * X - J_X * Y
    def lie_bracket(X, Y, state_vars):
        X_mat = sp.Matrix(X)
        Y_mat = sp.Matrix(Y)
        state_mat = sp.Matrix(state_vars)
        return Y_mat.jacobian(state_mat) * X_mat - X_mat.jacobian(state_mat) * Y

    print("正在进行符号动力学推导与 Lie 括号展开...\n")

    # 2. 遍历计算耦合矩阵的各个元素
    for j in range(n):
        for i in range(n):
            if i == j:
                # 对角线元素 C_ii = 1
                C[j, i] = sp.Integer(1)
                C_q[j, i] = sp.Integer(1)
                C_p[j, i] = sp.Integer(0)
                P[j, i] = sp.Integer(0)
            else:
                # 2.1 计算一阶项: (∂f_j / ∂x_i) * (1 / x_i_dot) * Δx_i
                dfj_dxi = sp.diff(f[j], x[i])
                dxi_dt = f[i]

                # 避免分母为 0 的极端符号情况
                if dxi_dt == 0:
                    term1 = sp.Integer(0)
                else:
                    term1 = (dfj_dxi / dxi_dt) * dx[i]

                # 2.2 计算二阶迭代 Lie Bracket 项: 1/6 * [g0, [g0, g1_col]]_j * (Δx_i)^2
                input_idx = state_to_input.get(i, None)
                if input_idx is not None and input_idx < m:
                    g1_col = g1[:, input_idx]
                    lb1 = lie_bracket(g0, g1_col, x)
                    lb2 = lie_bracket(g0, lb1, x)
                    term2 = (sp.Rational(1, 6)) * lb2[j] * (dx[i] ** 2)
                else:
                    term2 = sp.Integer(0)

                # 总耦合度
                C_ji = term1 + term2
                C[j, i] = sp.simplify(C_ji)

                # 2.3 分离 C_q (常数耦合) 和 C_p (比例耦合)
                # 原理：当驱动该状态的输入控制 u_var 趋于无穷大时，p/u 项消失，剩下的即为 q
                if input_idx is not None and input_idx < m:
                    u_var = u[input_idx]
                    try:
                        q_val = sp.limit(C[j, i], u_var, sp.oo)
                    except Exception:
                        q_val = C[j, i].subs(u_var, sp.oo)

                    C_q[j, i] = sp.simplify(q_val)
                    C_p[j, i] = sp.simplify(C[j, i] - C_q[j, i])
                    P[j, i] = sp.simplify(C_p[j, i] * u_var)
                else:
                    # 如果该通道无直接控制输入，则全部归为常数耦合
                    C_q[j, i] = C[j, i]
                    C_p[j, i] = sp.Integer(0)
                    P[j, i] = sp.Integer(0)

    # 3. 输出 LaTeX 格式结果
    print("=" * 60)
    print(" 生成的 LaTeX 公式代码 ")
    print("=" * 60)
    print(f"\\mathcal{{C}} = {sp.latex(C)}\n")
    print(f"\\mathcal{{C}}_q = {sp.latex(C_q)}\n")
    print(f"\\mathcal{{C}}_p = {sp.latex(C_p)}\n")
    print(f"P = {sp.latex(P)}")
    print("=" * 60)

    # 4. 序列化储存至 pkl 文件
    results = {
        "C": C,
        "C_q": C_q,
        "C_p": C_p,
        "P": P,
        "C_latex": sp.latex(C),
        "C_q_latex": sp.latex(C_q),
        "C_p_latex": sp.latex(C_p),
        "P_latex": sp.latex(P),
    }

    with open(filename, "wb") as f_pkl:
        pickle.dump(results, f_pkl)

    print(f"\n[成功] 符号矩阵及 LaTeX 文本已储存至: '{filename}'")
    return results


# ==========================================
# 示例运行：非完整独轮车系统 (Unicycle)
# ==========================================
if __name__ == "__main__":
    # 状态数 n=3, 输入数 m=1
    n_dim = 3
    m_dim = 1

    # 定义基础符号
    x1, x2, x3 = sp.symbols("x_1 x_2 x_3")
    u3 = sp.symbols("u_3")  # 控制输入（角速度）
    V = sp.symbols("V")  # 前向常数速度

    # 定义状态增量符号（用于输出显示）
    dx1, dx2, dx3 = sp.symbols(r"\Delta\ x_1 \Delta\ x_2 \Delta\ x_3")

    # 组装输入向量
    states = [x1, x2, x3]
    inputs = [u3]
    deltas = [dx1, dx2, dx3]

    # 定义 g0(x) 和 g1(x)
    g0_matrix = sp.Matrix([V * sp.cos(x3), V * sp.sin(x3), 0])
    g1_matrix = sp.Matrix([0, 0, 1])

    # 映射字典：状态索引 -> 输入通道索引
    # x_1 (idx 0) -> 无直接控制输入
    # x_2 (idx 1) -> 无直接控制输入
    # x_3 (idx 2) -> 由 inputs[0] (也就是 u_3) 直接驱动
    mapping = {2: 0}

    # 指定保存的 pkl 文件名
    pkl_name = "unicycle_coupleness.pkl"

    # 运行计算
    res = generate_coupleness_analysis(
        n=n_dim,
        m=m_dim,
        g0=g0_matrix,
        g1=g1_matrix,
        x=states,
        u=inputs,
        dx=deltas,
        state_to_input=mapping,
        filename=pkl_name,
    )