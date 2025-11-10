%LQR求解
clear;

syms theta dot_theta ddot_theta;
syms x dot_x ddot_x;
syms x_body dot_x_body ddot_x_body;
syms phi dot_phi ddot_phi;
syms T T_p;
syms R L L_M l m_w m_p M I_w I_p I_M g;
model=1;                    %1单腿建模，2双腿建模
body_fusion = 1;            %机体速度
g = 9.8;                    %重力加速度
R = 0.076;                  %轮半径
m_w = 1.35;                 %轮质量
m_p = 0.987;                %摆杆质量
M = 5.577;                  %机体质量
I_w = 0.01638248;           %轮转动惯量
I_M = 0.170604/2;           %机体转动惯量
l = 0.03455;                %机体质心离转轴距离
L_w = 0.384;                 %机体宽度


Q_cost=diag([200 160 200 80 1500 50]);
R_cost=diag([1.5,0.25]);

if body_fusion
    ddot_x = ddot_x_body - (L+L_M)*cos(theta)*ddot_theta + (L+L_M)*sin(theta)*dot_theta^2;
end
%对机体受力分析
%1.7
N_M = M * (ddot_x + (L + L_M) * (-dot_theta^2*sin(theta) + ddot_theta*cos(theta)) - l*(-dot_phi^2*sin(phi) + ddot_phi*cos(phi)));
%1.8
P_M = M*g + M*((L+L_M)*(-dot_theta^2*cos(theta) - ddot_theta*sin(theta)) + l*(-dot_phi^2*cos(phi) - ddot_phi*sin(phi)));
%1.4
N = N_M + m_p*(ddot_x + L*(-dot_theta^2*sin(theta) + ddot_theta*cos(theta)));
%1.5
P = P_M + m_p*g + m_p*L*(-dot_theta^2*cos(theta) - ddot_theta*sin(theta));

%方程求解
%1.3
equ1 = ddot_x - (T - N*R)/(I_w/R + m_w*R);
%1.6
equ2 = (P*L + P_M*L_M)*sin(theta) - (N*L+N_M*L_M)*cos(theta) - T + T_p - I_p*ddot_theta;
%1.9
equ3 = T_p + N_M * l * cos(phi) + P_M * l * sin(phi) - I_M * ddot_phi;
if model == 1
if body_fusion
    [ddot_theta, ddot_x_body, ddot_phi] = solve([equ1, equ2, equ3], [ddot_theta, ddot_x_body, ddot_phi]);
    Ja = jacobian([dot_theta ddot_theta dot_x_body ddot_x_body dot_phi ddot_phi], [theta, dot_theta, x_body, dot_x_body, phi, dot_phi]);
    Jb = jacobian([dot_theta ddot_theta dot_x_body ddot_x_body dot_phi ddot_phi], [T, T_p]);
    
    A = simplify(vpa(subs(Ja, [theta, dot_theta, x_body, dot_x_body, phi, dot_phi], [0, 0, 0, 0, 0, 0])));
    B = simplify(vpa(subs(Jb, [theta, dot_theta, x_body, dot_x_body, phi, dot_phi], [0, 0, 0, 0, 0, 0])));
    
else
    [ddot_theta, ddot_x, ddot_phi] = solve([equ1, equ2, equ3], [ddot_theta, ddot_x, ddot_phi]);
    Ja = jacobian([dot_theta ddot_theta dot_x ddot_x dot_phi ddot_phi], [theta, dot_theta, x, dot_x, phi, dot_phi]);
    Jb = jacobian([dot_theta ddot_theta dot_x ddot_x dot_phi ddot_phi], [T, T_p]);
    
    A = simplify(vpa(subs(Ja, [theta, dot_theta, x, dot_x, phi, dot_phi], [0, 0, 0, 0, 0, 0])));
    B = simplify(vpa(subs(Jb, [theta, dot_theta, x, dot_x, phi, dot_phi], [0, 0, 0, 0, 0, 0])));
end

elseif model == 2
    %补充方程


end
%% LQR计算

leg_var = 0.08;
K=zeros(60,12);
leglen=zeros(60,1);
for i=1:60
    leg_var=leg_var+0.005; % 5mm线性化一次
    llm= - 0.605814 * leg_var * leg_var + 0.962002 * leg_var - 0.062925;;
    ll = 0.605814 *  leg_var * leg_var + 0.037998 * leg_var + 0.062925;
    leglen(i)=leg_var;
    i_p = 0.311354 * leg_var * leg_var * leg_var -0.305621 * leg_var * leg_var + 0.076419 * leg_var + 0.017774;
    trans_A=double(subs(A,[L L_M I_p],[ll llm i_p]));
    trans_B=double(subs(B,[L L_M I_p],[ll llm i_p]));
    KK=lqrd(trans_A,trans_B,Q_cost,R_cost,0.001);
    KK_t=KK.';
    K(i,:)=KK_t(:);
end

%% 系数拟合
K_cons=zeros(12,4);  %排列顺序是

for i=1:12
    res=fit(leglen,K(:,i),'poly3');
    K_cons(i,:)=[res.p1,res.p2,res.p3,res.p4];
end
for i = 1:size(K_cons, 1)
    fprintf('{%.6ff, %.6ff, %.6ff, %.6ff},\n', ...
            K_cons(i,1), K_cons(i,2), K_cons(i,3), K_cons(i,4));
end