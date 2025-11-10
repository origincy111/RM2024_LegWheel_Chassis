%LQR求解
clear;

syms x dot_x ddot_x;
syms x_body dot_x_body ddot_x_body;
syms R L L_M l m_w m_p M I_w I_p I_M g;
syms roll dot_roll ddot_roll;
syms yaw dot_yaw ddot_yaw;

syms theta_l dot_theta_l ddot_theta_l;
syms phi_l dot_phi_l ddot_phi_l;
syms Tw_l Tp_l;
syms P_M_l N_M_l P_l N_l L_l L_M_l 

syms theta_r dot_theta_r ddot_theta_r;
syms phi_r dot_phi_r ddot_phi_r;
syms Tw_r Tp_r;
syms P_M_r N_M_r P_r N_r L_r L_M_r


model = 2;                  %1单腿建模，2双腿建模
body_fusion = 1;            %机体速度
g = 9.8;                    %重力加速度
R = 0.076;                  %轮半径
m_w = 1.35;                 %轮质量
m_p = 0.987;                %摆杆质量
M = 5.577;                  %机体质量
I_w = 0.01638248;           %轮转动惯量
I_M = 0.170604/2;           %机体转动惯量
l = 0.03455;                %机体质心离转轴距离
L_w = 0.384;                %机体宽度

I_x = 0 ;
I_y = 0 ;
I_z = 0 ;



Q_cost=diag([200 120 240 60 1200 300]);
R_cost=diag([2,0.25]);

if body_fusion
    ddot_x = ddot_x_body - (L+L_M)*cos(theta)*ddot_theta + (L+L_M)*sin(theta)*dot_theta^2;
end

%对机体受力分析
%1.7
N_M_l = M * (ddot_x + (L_l+L_M_l) * (-dot_theta_l^2*sin(theta_l) + ddot_theta_l*cos(theta_l)) - l*(-dot_phi_l^2*sin(phi_l) + ddot_phi_l*cos(phi_l)));
N_M_r = M * (ddot_x + (L_r+L_M_r) * (-dot_theta_r^2*sin(theta_r) + ddot_theta_r*cos(theta_r)) - l*(-dot_phi_r^2*sin(phi_r) + ddot_phi_r*cos(phi_r)));
%1.8
P_M_l = M*g + M*((L_l+L_M_l)*(-dot_theta_l^2*cos(theta_l) - ddot_theta_l*sin(theta_l)) + l*(-dot_phi_l^2*cos(phi_l) - ddot_phi_l*sin(phi_l)));
P_M_r = M*g + M*((L_r+L_M_r)*(-dot_theta_r^2*cos(theta_r) - ddot_theta_r*sin(theta_r)) + l*(-dot_phi_r^2*cos(phi_r) - ddot_phi_r*sin(phi_r)));
%1.4
N_l = N_M_l + m_p*(ddot_x + L_l*(-dot_theta_l^2*sin(theta_l) + ddot_theta_l*cos(theta_l)));
N_r = N_M_r + m_p*(ddot_x + L_r*(-dot_theta_r^2*sin(theta_r) + ddot_theta_r*cos(theta_r)));
%1.5
P_l = P_M_l + m_p*g + m_p*L_l*(-dot_theta_l^2*cos(theta_l) - ddot_theta_l*sin(theta_l));
P_r = P_M_r + m_p*g + m_p*L_r*(-dot_theta_l^2*cos(theta_r) - ddot_theta_r*sin(theta_r));
%方程求解
%1.3(左右对加速度产生的合贡献)
equ1 = ddot_x - (T_l - N_l*R)/(I_w/R + m_w*R) - (T_r-N_r*R)/(I_w/R + m_w*R);
%1.6
equ2 = (P_l*L_l + P_M_l*L_M_l)*sin(theta_l) - (N_l*L_l+N_M_l*L_M_l)*cos(theta_l) - Tw_l + Tp_l - I_p*ddot_theta_l;
equ3 = (P_r*L_r + P_M_r*L_M_r)*sin(theta_r) - (N_r*L_r+N_M_r*L_M_r)*cos(theta_r) - Tw_r + Tp_r - I_p*ddot_theta_r;
%1.9
equ4 = Tp_l + N_M_l * l * cos(phi_l) + P_M_l * l * sin(phi_l) - I_M * ddot_phi_l;
equ5 = Tp_r + N_M_r * l * cos(phi_r) + P_M_r * l * sin(phi_r) - I_M * ddot_phi_r;

if model == 2
    %补充方程
    equ6 = I_x * ddot_roll - (P_M_l-P_M_r) * 0.5 * L_w;
    equ7 = -I_y * ddot_phi_l - Tp_r - Tp_l;
    equ8 = I_z * ddot_yaw - (N_M_r-N_M_l)*0.5*L_w;
    equ9 = M*ddot_x - N_M_l + N_M_r;
    equ10 = M*g - P_M_l - P_M_r;
    equ11 = roll - atan(((L_l+L_M_l)*theta_l - (L_r+L_M_r)*theta_r)/L_w);
    
    % 求解方程组
    [ddot_theta_l, ddot_theta_r, ddot_phi_l, ddot_phi_r, ddot_x, ddot_roll, ddot_yaw] = ...
        solve([equ1, equ2, equ3, equ4, equ5, equ6, equ7, equ8, equ9, equ10], ...
              [ddot_theta_l, ddot_theta_r, ddot_phi_l, ddot_phi_r, ddot_x, ddot_roll, ddot_yaw]);
    
    % 定义状态向量: [x, dot_x, yaw, dot_yaw, roll, dot_roll, theta_l, dot_theta_l, theta_r, dot_theta_r, phi_l, dot_phi_l]
    state_vars = [x, dot_x, yaw, dot_yaw, roll, dot_roll, theta_l, dot_theta_l, theta_r, dot_theta_r, phi_l, dot_phi_l];
    control_vars = [Tw_l, Tp_l, Tw_r, Tp_r];
    
    % 构建状态导数向量
    state_derivatives = [dot_x, ddot_x, dot_yaw, ddot_yaw, dot_roll, ddot_roll, ...
                        dot_theta_l, ddot_theta_l, dot_theta_r, ddot_theta_r, dot_phi_l, ddot_phi_l];
    
    % 计算雅可比矩阵
    Ja = jacobian(state_derivatives, state_vars);
    Jb = jacobian(state_derivatives, control_vars);
    
    % 在平衡点线性化
    A = simplify(vpa(subs(Ja, [x, dot_x, yaw, dot_yaw, roll, dot_roll, ...
                              theta_l, dot_theta_l, theta_r, dot_theta_r, phi_l, dot_phi_l], ...
                         zeros(1,12))));
    B = simplify(vpa(subs(Jb, [x, dot_x, yaw, dot_yaw, roll, dot_roll, ...
                              theta_l, dot_theta_l, theta_r, dot_theta_r, phi_l, dot_phi_l], ...
                         zeros(1,12))));
end

%% LQR计算
% 更新Q和R矩阵以适应12状态4输入系统
Q_cost = diag([10, 1, 50, 5, 50, 5, 200, 20, 200, 20, 1200, 120]);  % 12x12状态权重
R_cost = diag([2, 0.25, 2, 0.25]);  % 4x4控制权重

% 双腿高度参数范围
leg_var_l = 0.08;  % 左腿初始高度
leg_var_r = 0.08;  % 右腿初始高度

K = zeros(60, 48);  % 60个采样点，48个系数（12状态×4输入）
leglen_l = zeros(60, 1);
leglen_r = zeros(60, 1);

for i = 1:60
    leg_var_l = leg_var_l + 0.005;  % 左腿高度增加5mm
    leg_var_r = leg_var_r + 0.005;  % 右腿高度增加5mm
    
    % 计算左腿参数
    ll_l = 0.605814 * leg_var_l * leg_var_l + 0.037998 * leg_var_l + 0.062925;
    llm_l = -0.605814 * leg_var_l * leg_var_l + 0.962002 * leg_var_l - 0.062925;
    i_p_l = 0.311354 * leg_var_l^3 - 0.305621 * leg_var_l^2 + 0.076419 * leg_var_l + 0.017774;
    
    % 计算右腿参数  
    ll_r = 0.605814 * leg_var_r * leg_var_r + 0.037998 * leg_var_r + 0.062925;
    llm_r = -0.605814 * leg_var_r * leg_var_r + 0.962002 * leg_var_r - 0.062925;
    i_p_r = 0.311354 * leg_var_r^3 - 0.305621 * leg_var_r^2 + 0.076419 * leg_var_r + 0.017774;
    
    leglen_l(i) = leg_var_l;
    leglen_r(i) = leg_var_r;
    
    % 代入参数计算A和B矩阵
    trans_A = double(subs(A, [L_l, L_M_l, I_p, L_r, L_M_r], [ll_l, llm_l, i_p_l, ll_r, llm_r]));
    trans_B = double(subs(B, [L_l, L_M_l, I_p, L_r, L_M_r], [ll_l, llm_l, i_p_l, ll_r, llm_r]));
    
    % 离散化LQR计算
    KK = lqrd(trans_A, trans_B, Q_cost, R_cost, 0.001);
    KK_t = KK.';
    K(i, :) = KK_t(:);
end

%% 系数拟合（基于两个腿高）
K_cons = zeros(48, 9);  % 48个系数，每个用9个参数拟合（双变量三次多项式）

% 创建网格数据用于拟合
[L1, L2] = meshgrid(leglen_l, leglen_r);
L1_flat = L1(:);
L2_flat = L2(:);

for i = 1:48
    % 提取第i个系数在所有腿高组合下的值
    coeff_values = K(:, i);
    
    % 使用双变量三次多项式拟合
    % 模型: f(l1,l2) = p1 + p2*l1 + p3*l2 + p4*l1^2 + p5*l1*l2 + p6*l2^2 + p7*l1^3 + p8*l1^2*l2 + p9*l1*l2^2
    % 构建设计矩阵
    X = [ones(60,1), leglen_l, leglen_r, leglen_l.^2, leglen_l.*leglen_r, leglen_r.^2, ...
         leglen_l.^3, leglen_l.^2.*leglen_r, leglen_l.*leglen_r.^2];
    
    % 最小二乘拟合
    p = X \ coeff_values;
    K_cons(i, :) = p.';
end

%% 输出拟合系数
fprintf('双腿LQR拟合系数（基于左右腿高）：\n');
for i = 1:size(K_cons, 1)
    fprintf('{');
    for j = 1:size(K_cons, 2)
        if j == size(K_cons, 2)
            fprintf('%.6ff', K_cons(i, j));
        else
            fprintf('%.6ff, ', K_cons(i, j));
        end
    end
    fprintf('},\n');
end
