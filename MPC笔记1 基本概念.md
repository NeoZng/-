# MPC笔记1：基本概念与推导

## 最优控制

　　最优控制理论是最优化理论的一个分支，其目的在于找到动态系统在特定时间内的控制，使特定的优化目标最小（或最大）。

　　一般连续时间最优控制的优化目标如下：
$$
J\left[\boldsymbol{x}(\cdot), \boldsymbol{u}(\cdot), t_{0}, t_{f}\right]:=E\left[\boldsymbol{x}\left(t_{0}\right), t_{0}, \boldsymbol{x}\left(t_{f}\right), t_{f}\right]+\int_{t_{0}}^{t_{f}} P[\boldsymbol{x}(t), \boldsymbol{u}(t), t] \mathrm{d} t
$$

## MPC

　　MPC根据通过模型预测出的系统在未来一段时间内的表现进行优化控制，系统模型多采用离散形式表达：
$$
\begin{aligned}
\boldsymbol {x}_{k+1}&=\boldsymbol A \boldsymbol{x}_k+\boldsymbol B \boldsymbol{u}_k \\
\boldsymbol{y}_{k}&=\boldsymbol C \boldsymbol{x}_k+\boldsymbol D \boldsymbol{u}_k
\end{aligned}
$$

### 算法流程

　　对于 $k$ 时刻：

1. 测量、估计当前时刻系统状态。

2. 根据模型及 $u_k,u_{k+1},...,u_{k+N-1}$ 预测系统未来表现。

3. 根据系统表现求解最优化问题。

4. 实施最优化求解得到的控制 $\boldsymbol u_k$。

### 二次规划

　　二次规划是一种常见的最优化问题，具有丰富的求解工具。因此我们可通过将MPC的优化问题转化为二次规划形式以进行求解。二次规划具有一般形式：
$$
\text{min:}\ \ f(\boldsymbol x)=\boldsymbol x^T\boldsymbol Q\boldsymbol x+\boldsymbol c^T\boldsymbol x
$$
根据不同的参数特性，可以得到对问题不同的结论：

- 若 $\boldsymbol Q$ 为半正定矩阵，则优化目标为凸函数，若约束条件定义的可行域不为空，且目标函数在此可行域有下界，则有全局最小值。
- 若 $\boldsymbol Q$ 为正定矩阵，则有唯一的全局最小值。
- 若 $\boldsymbol Q$ 为非正定矩阵，则目标函数是有多个驻点和局部极小点的NP难问题。
- 如果 $\boldsymbol Q=\boldsymbol O$，则变成线性规划问题。

### 镇定问题推导

　　考虑离散系统：
$$
\boldsymbol x_{k+1} = \boldsymbol A\boldsymbol x_k+\boldsymbol B\boldsymbol u_k\\
\boldsymbol y = \boldsymbol x
$$
对于 $k$ 时刻，系统状态 $\boldsymbol X_k$ 与输入 $\boldsymbol U_k$ 可表示为：
$$
\boldsymbol X_k = \left[\begin{array}{c}
\boldsymbol x_{k+1|k}\\
\vdots\\
\boldsymbol x_{k+N|k}
\end{array}\right],\quad
\boldsymbol U_k = \left[\begin{array}{c}
\boldsymbol u_{k|k}\\
\vdots\\
\boldsymbol u_{k+N-1|k}
\end{array}\right]
$$
其中 $N$ 为预测区间（Predictive Horizon）。

　　假设系统参考输入 $\boldsymbol r = \boldsymbol O$，即误差 $\boldsymbol e = \boldsymbol y-\boldsymbol r=\boldsymbol x$，有：
$$
J = \boldsymbol x_{k+N|k}^T\boldsymbol P \boldsymbol x_{k+N|k}+\sum_{i=1}^{N-1}\boldsymbol x_{k+i|k}^T\boldsymbol Q \boldsymbol x_{k+i|k}+\sum_{i=0}^{N-1}\boldsymbol u_{k+i|k}^T\boldsymbol R \boldsymbol u_{k+i|k}
$$
对于二次规划问题一般形式 $f(\boldsymbol x)=\boldsymbol x^T\boldsymbol Q\boldsymbol x+\boldsymbol c^T\boldsymbol x$，仅为 $\boldsymbol x$ 的函数，而上式中存在 $\boldsymbol x,\boldsymbol u$ 两个变量，因此需先通过模型消去变量 $\boldsymbol x$。根据 $\boldsymbol x_{k+1} = \boldsymbol A\boldsymbol x_k+\boldsymbol B\boldsymbol u_k$，有：
$$
\begin{align}
 \boldsymbol x_{k+1|k} &= \boldsymbol A\boldsymbol x_{k|k}+\boldsymbol B\boldsymbol u_{k|k}\\
 \boldsymbol x_{k+2|k} &= \boldsymbol A(\boldsymbol A\boldsymbol x_{k|k}+\boldsymbol B\boldsymbol u_{k|k})+\boldsymbol B\boldsymbol u_{k+1|k}\\
&= \boldsymbol A^2\boldsymbol x_{k|k}+\boldsymbol A\boldsymbol B\boldsymbol u_{k|k}+\boldsymbol B\boldsymbol u_{k+1|k}\\
 \boldsymbol x_{k+3|k} &= \boldsymbol A(\boldsymbol A^2\boldsymbol x_{k|k}+\boldsymbol A\boldsymbol B\boldsymbol u_{k|k}+\boldsymbol B\boldsymbol u_{k+1|k})+\boldsymbol B\boldsymbol u_{k+2|k}\\
&= \boldsymbol A^3\boldsymbol x_{k|k}+\boldsymbol A^2\boldsymbol B\boldsymbol u_{k|k}+\boldsymbol A\boldsymbol B\boldsymbol u_{k+1|k}+\boldsymbol B\boldsymbol u_{k+2|k}\\
&\vdots\\
\boldsymbol x_{k+N|k} &=  \boldsymbol A^{N}\boldsymbol x_{k|k}+\boldsymbol A^{N-1}\boldsymbol B\boldsymbol u_{k|k}+\boldsymbol A^{N-2}\boldsymbol B\boldsymbol u_{k+1|k}+\cdots+\boldsymbol B\boldsymbol u_{k+N-1|k}\\
&= \boldsymbol A^{N}\boldsymbol x_{k|k}+\sum_{j=1}^N\boldsymbol A^{N-j}\boldsymbol B
\boldsymbol u_{k+j-1|k}
\end{align}
$$
进一步，有：
$$
\boldsymbol X_k = \left[\begin{array}{c}
\boldsymbol A\\
\boldsymbol A^2\\
\vdots\\
\boldsymbol A^N
\end{array}\right]\boldsymbol x_k+
\left[\begin{array}{ccccc}
\boldsymbol B & \boldsymbol O & \boldsymbol O & \cdots & \boldsymbol O \\
\boldsymbol A\boldsymbol B & \boldsymbol B & \boldsymbol O & \cdots & \boldsymbol O \\
\vdots & \vdots & \vdots & \ddots & \vdots & \\
\boldsymbol A^{N-1}\boldsymbol B  & \boldsymbol A^{N-2}\boldsymbol B &  \boldsymbol A^{N-3}\boldsymbol B & \cdots & \boldsymbol B
\end{array}\right]
\boldsymbol U_k
$$
应注意其中 $\boldsymbol O$ 为与 $\boldsymbol B$ 同阶的零矩阵，记：
$$
\boldsymbol F =\left[\begin{array}{c}
\boldsymbol A\\
\boldsymbol A^2\\
\vdots\\
\boldsymbol A^N
\end{array}\right],\quad \boldsymbol \Phi = \left[\begin{array}{ccccc}
\boldsymbol B & \boldsymbol O & \boldsymbol O & \cdots & \boldsymbol O \\
\boldsymbol A\boldsymbol B & \boldsymbol B & \boldsymbol O & \cdots & \boldsymbol O \\
\vdots & \vdots & \vdots & \ddots & \vdots & \\
\boldsymbol A^{N-1}\boldsymbol B  & \boldsymbol A^{N-2}\boldsymbol B &  \boldsymbol A^{N-3}\boldsymbol B & \cdots & \boldsymbol B
\end{array}\right]
$$
得：
$$
\boldsymbol X_k = \boldsymbol F\boldsymbol x_k+\boldsymbol \Phi\boldsymbol U_k
$$
　　整理代价函数：
$$
\begin{align}
J &= \sum_{i=1}^{N-1}\boldsymbol x_{k+i|k}^T\boldsymbol Q \boldsymbol x_{k+i|k}+\boldsymbol x_{k+N|k}^T\boldsymbol P \boldsymbol x_{k+N|k}+\sum_{i=0}^{N-1}\boldsymbol u_{k+i|k}^T\boldsymbol R \boldsymbol u_{k+i|k}\\
&=\boldsymbol X_k^T
\left[\begin{array}{cccc}
\boldsymbol Q & \boldsymbol O & \cdots & \boldsymbol O \\
\boldsymbol O & \boldsymbol Q &  \cdots & \boldsymbol O \\
\vdots & \vdots &  \ddots & \vdots & \\
\boldsymbol O  &  \boldsymbol O & \cdots & \boldsymbol P
\end{array}\right]\boldsymbol X_k+\boldsymbol U_k^T
\left[\begin{array}{cccc}
\boldsymbol R & \boldsymbol O & \cdots & \boldsymbol O \\
\boldsymbol O & \boldsymbol R &  \cdots & \boldsymbol O \\
\vdots & \vdots &  \ddots & \vdots & \\
\boldsymbol O  &  \boldsymbol O & \cdots & \boldsymbol R
\end{array}\right]\boldsymbol U_k
\end{align}
$$
记：
$$
\bar{\boldsymbol Q} =\left[\begin{array}{cccc}
\boldsymbol Q & \boldsymbol O & \cdots & \boldsymbol O \\
\boldsymbol O & \boldsymbol Q &  \cdots & \boldsymbol O \\
\vdots & \vdots &  \ddots & \vdots & \\
\boldsymbol O  &  \boldsymbol O & \cdots & \boldsymbol P
\end{array}\right],\quad
\bar{\boldsymbol R} = \left[\begin{array}{cccc}
\boldsymbol R & \boldsymbol O & \cdots & \boldsymbol O \\
\boldsymbol O & \boldsymbol R &  \cdots & \boldsymbol O \\
\vdots & \vdots &  \ddots & \vdots & \\
\boldsymbol O  &  \boldsymbol O & \cdots & \boldsymbol R
\end{array}\right]
$$
得：
$$
J = \boldsymbol X_k^T\bar{\boldsymbol Q} \boldsymbol X_k+\boldsymbol U_k^T\bar{\boldsymbol R} \boldsymbol U_k
$$
将 $\boldsymbol X_k = \boldsymbol F\boldsymbol x_k+\boldsymbol \Phi\boldsymbol U_k$ 代入，有：
$$
\begin{align}
J &= (\boldsymbol F\boldsymbol x_k+\boldsymbol \Phi\boldsymbol U_k)^T\bar{\boldsymbol Q} (\boldsymbol F\boldsymbol x_k+\boldsymbol \Phi\boldsymbol U_k)+\boldsymbol U_k^T\bar{\boldsymbol R} \boldsymbol U_k\\
&=\boldsymbol x_k^T\boldsymbol F^T\bar{\boldsymbol Q}\boldsymbol F\boldsymbol x_k+
\boldsymbol U_k^T\boldsymbol \Phi^T\bar{\boldsymbol Q}\boldsymbol F\boldsymbol x_k+
\boldsymbol x_k^T\boldsymbol F^T\bar{\boldsymbol Q}\boldsymbol \Phi\boldsymbol U_k+
\boldsymbol U_k^T\boldsymbol \Phi^T\bar{\boldsymbol Q}\boldsymbol \Phi\boldsymbol U_k+\boldsymbol U_k^T\bar{\boldsymbol R} \boldsymbol U_k\\
&=\boldsymbol x_k^T\boldsymbol F^T\bar{\boldsymbol Q}\boldsymbol F\boldsymbol x_k+
2\boldsymbol x_k^T\boldsymbol F^T\bar{\boldsymbol Q}\boldsymbol \Phi\boldsymbol U_k+
\boldsymbol U_k^T\left(\boldsymbol \Phi^T\bar{\boldsymbol Q}\boldsymbol \Phi+\bar{\boldsymbol R}\right)\boldsymbol U_k
\end{align}
$$
考虑到 $\boldsymbol x_k^T\boldsymbol F^T\bar{\boldsymbol Q}\boldsymbol F\boldsymbol x_k$ 与 $\boldsymbol U_k$ 无关，可转化为以下二次规划问题：
$$
J=\boldsymbol U_k^T\left(\boldsymbol \Phi^T\bar{\boldsymbol Q}\boldsymbol \Phi+\bar{\boldsymbol R}\right)\boldsymbol U_k+2\boldsymbol x_k^T\boldsymbol F^T\bar{\boldsymbol Q}\boldsymbol \Phi\boldsymbol U_k
$$

为求其最小值，令其求导等于零：
$$
\left.\frac{\partial J}{\partial \boldsymbol U}\right|_{\boldsymbol U=\bar{\boldsymbol U}} = 0
$$
即：
$$
\left.
2\boldsymbol U_k^T\left(\boldsymbol \Phi^T\bar{\boldsymbol Q}\boldsymbol \Phi+\bar{\boldsymbol R}\right)\right|_{\boldsymbol U=\bar{\boldsymbol U}}+2\boldsymbol x_k^T\boldsymbol F^T\bar{\boldsymbol Q}\boldsymbol \Phi=0
$$
解得：
$$
\bar{\boldsymbol  U}_k =-\left(\boldsymbol \Phi^T\bar{\boldsymbol Q}\boldsymbol \Phi+\bar{\boldsymbol R}\right)^{-1}\boldsymbol \Phi^T\bar{\boldsymbol Q}\boldsymbol F\boldsymbol x_k
$$
由此可见，无约束线性MPC在镇定问题中与状态反馈等价。

### 跟踪问题推导

　　考虑离散系统：
$$
\boldsymbol x_{k+1} = \boldsymbol A\boldsymbol x_k+\boldsymbol B\boldsymbol u_k\\
\boldsymbol y = \boldsymbol C\boldsymbol x
$$
　　定义误差 $\boldsymbol e_k =  \widetilde{\boldsymbol x}_k-\boldsymbol x_k$，其中 $\widetilde{\boldsymbol x}_k$ 为期望状态，选取代价函数为：
$$
J = \boldsymbol e_{k+N|k}^T\boldsymbol P \boldsymbol e_{k+N|k}+\sum_{i=1}^{N-1}\boldsymbol e_{k+i|k}^T\boldsymbol Q \boldsymbol e_{k+i|k}+\sum_{i=0}^{N-1}\boldsymbol u_{k+i|k}^T\boldsymbol R \boldsymbol u_{k+i|k}
$$
定义向量：
$$
\widetilde{\boldsymbol X}_k = \left[\begin{array}{c}
\widetilde{\boldsymbol x}_{k+1|k}\\
\vdots\\
\widetilde{\boldsymbol x}_{k+N|k}
\end{array}\right],\quad
\boldsymbol E_k =\widetilde{\boldsymbol X}_k- \boldsymbol X_k=\left[\begin{array}{c}
\boldsymbol e_{k+1|k}\\
\vdots\\
\boldsymbol e_{k+N|k}
\end{array}\right]
$$
　　整理代价函数：
$$
\begin{align}
J &= \sum_{i=1}^{N-1}\boldsymbol e_{k+i|k}^T\boldsymbol Q \boldsymbol e_{k+i|k}+\boldsymbol e_{k+N|k}^T\boldsymbol P \boldsymbol e_{k+N|k}+\sum_{i=0}^{N-1}\boldsymbol u_{k+i|k}^T\boldsymbol R \boldsymbol u_{k+i|k}\\
&=\boldsymbol E_k^T
\left[\begin{array}{cccc}
\boldsymbol Q & \boldsymbol O & \cdots & \boldsymbol O \\
\boldsymbol O & \boldsymbol Q &  \cdots & \boldsymbol O \\
\vdots & \vdots &  \ddots & \vdots & \\
\boldsymbol O  &  \boldsymbol O & \cdots & \boldsymbol P
\end{array}\right]\boldsymbol E_k+\boldsymbol U_k^T
\left[\begin{array}{cccc}
\boldsymbol R & \boldsymbol O & \cdots & \boldsymbol O \\
\boldsymbol O & \boldsymbol R &  \cdots & \boldsymbol O \\
\vdots & \vdots &  \ddots & \vdots & \\
\boldsymbol O  &  \boldsymbol O & \cdots & \boldsymbol R
\end{array}\right]\boldsymbol U_k
\end{align}
$$
记：
$$
\bar{\boldsymbol Q} =\left[\begin{array}{cccc}
\boldsymbol Q & \boldsymbol O & \cdots & \boldsymbol O \\
\boldsymbol O & \boldsymbol Q &  \cdots & \boldsymbol O \\
\vdots & \vdots &  \ddots & \vdots & \\
\boldsymbol O  &  \boldsymbol O & \cdots & \boldsymbol P
\end{array}\right],\quad
\bar{\boldsymbol R} = \left[\begin{array}{cccc}
\boldsymbol R & \boldsymbol O & \cdots & \boldsymbol O \\
\boldsymbol O & \boldsymbol R &  \cdots & \boldsymbol O \\
\vdots & \vdots &  \ddots & \vdots & \\
\boldsymbol O  &  \boldsymbol O & \cdots & \boldsymbol R
\end{array}\right]
$$
得：
$$
J = \left(\widetilde{\boldsymbol X}_k- \boldsymbol X_k\right)^T\bar{\boldsymbol Q} \left(\widetilde{\boldsymbol X}_k- \boldsymbol X_k\right)+\boldsymbol U_k^T\bar{\boldsymbol R} \boldsymbol U_k
$$
将 $\boldsymbol X_k = \boldsymbol F\boldsymbol x_k+\boldsymbol \Phi\boldsymbol U_k$ 代入，有：
$$
\begin{align}
J =& \left(\widetilde{\boldsymbol X}_k- \boldsymbol F\boldsymbol x_k-\boldsymbol \Phi\boldsymbol U_k\right)^T\bar{\boldsymbol Q} \left(\widetilde{\boldsymbol X}_k- \boldsymbol F\boldsymbol x_k-\boldsymbol \Phi\boldsymbol U_k\right)+\boldsymbol U_k^T\bar{\boldsymbol R} \boldsymbol U_k\\
=&\left(\widetilde{\boldsymbol X}_k- \boldsymbol F\boldsymbol x_k\right)^T\bar{\boldsymbol Q}\left(\widetilde{\boldsymbol X}_k- \boldsymbol F\boldsymbol x_k\right)-
\boldsymbol U_k^T\boldsymbol \Phi^T\bar{\boldsymbol Q}\left(\widetilde{\boldsymbol X}_k- \boldsymbol F\boldsymbol x_k\right)-
\left(\widetilde{\boldsymbol X}_k- \boldsymbol F\boldsymbol x_k\right)^T\bar{\boldsymbol Q}\boldsymbol \Phi\boldsymbol U_k\\
&+
\boldsymbol U_k^T\boldsymbol \Phi^T\bar{\boldsymbol Q}\boldsymbol \Phi\boldsymbol U_k+\boldsymbol U_k^T\bar{\boldsymbol R} \boldsymbol U_k\\
=&\left(\widetilde{\boldsymbol X}_k- \boldsymbol F\boldsymbol x_k\right)^T\bar{\boldsymbol Q}\left(\widetilde{\boldsymbol X}_k- \boldsymbol F\boldsymbol x_k\right)-
2\left(\widetilde{\boldsymbol X}_k- \boldsymbol F\boldsymbol x_k\right)^T\bar{\boldsymbol Q}\boldsymbol \Phi\boldsymbol U_k+
\boldsymbol U_k^T\left(\boldsymbol \Phi^T\bar{\boldsymbol Q}\boldsymbol \Phi+\bar{\boldsymbol R}\right)\boldsymbol U_k
\end{align}
$$
考虑到 $\left(\widetilde{\boldsymbol X}_k- \boldsymbol F\boldsymbol x_k\right)^T\bar{\boldsymbol Q}\left(\widetilde{\boldsymbol X}_k- \boldsymbol F\boldsymbol x_k\right)$ 与 $\boldsymbol U_k$ 无关，可转化为以下二次规划问题：
$$
J=
\boldsymbol U_k^T\left(\boldsymbol \Phi^T\bar{\boldsymbol Q}\boldsymbol \Phi+\bar{\boldsymbol R}\right)\boldsymbol U_k-
2\left(\widetilde{\boldsymbol X}_k- \boldsymbol F\boldsymbol x_k\right)^T\bar{\boldsymbol Q}\boldsymbol \Phi\boldsymbol U_k
$$

为求其最小值，令其求导等于零：
$$
\left.\frac{\partial J}{\partial \boldsymbol U}\right|_{\boldsymbol U=\bar{\boldsymbol U}} = 0
$$
即：
$$
\left.2\boldsymbol U_k^T\left(\boldsymbol \Phi^T\bar{\boldsymbol Q}\boldsymbol \Phi+\bar{\boldsymbol R}\right)\right|_{\boldsymbol U=\bar{\boldsymbol U}}-
2\left(\widetilde{\boldsymbol X}_k- \boldsymbol F\boldsymbol x_k\right)^T\bar{\boldsymbol Q}\boldsymbol \Phi=0
$$
解得：
$$
\bar{\boldsymbol  U}_k =\left(\boldsymbol \Phi^T\bar{\boldsymbol Q}\boldsymbol \Phi+\bar{\boldsymbol R}\right)^{-1}\boldsymbol \Phi^T\bar{\boldsymbol Q}\left(\widetilde{\boldsymbol X}_k- \boldsymbol F\boldsymbol x_k\right)
$$

