%计算不同腿长下适合的K矩阵，再进行多项式拟合，得到2*6矩阵每个参数对应的多项式参数
tic
j=1;
% 腿长范围改为0.12到0.28，步长0.01
leg=0.08:0.005:0.36;

% 预分配数组
k11 = zeros(1, length(leg));
k12 = zeros(1, length(leg));
k13 = zeros(1, length(leg));
k14 = zeros(1, length(leg));
k15 = zeros(1, length(leg));
k16 = zeros(1, length(leg));
k21 = zeros(1, length(leg));
k22 = zeros(1, length(leg));
k23 = zeros(1, length(leg));
k24 = zeros(1, length(leg));
k25 = zeros(1, length(leg));
k26 = zeros(1, length(leg));

for i=leg
    try
        k=get_k_length(i);
        
        % 检查k的维度是否正确
        if size(k,1) == 2 && size(k,2) == 6
            k11(j) = k(1,1);
            k12(j) = k(1,2);
            k13(j) = k(1,3);
            k14(j) = k(1,4);
            k15(j) = k(1,5);
            k16(j) = k(1,6);

            k21(j) = k(2,1);
            k22(j) = k(2,2);
            k23(j) = k(2,3);
            k24(j) = k(2,4);
            k25(j) = k(2,5);
            k26(j) = k(2,6);
            
            fprintf('成功计算腿长 %.3f 的K矩阵\n', i);
            j=j+1;
        else
            fprintf('警告: 腿长 %.3f 的K矩阵维度不正确: %dx%d\n', i, size(k,1), size(k,2));
        end
    catch ME
        fprintf('错误: 腿长 %.3f 计算失败: %s\n', i, ME.message);
        % 跳过这个腿长值
        continue;
    end
end

% 检查实际计算成功的点数
if j-1 < length(leg)
    fprintf('警告: 只成功计算了 %d/%d 个腿长点\n', j-1, length(leg));
    % 截断数组
    leg = leg(1:j-1);
    k11 = k11(1:j-1);
    k12 = k12(1:j-1);
    k13 = k13(1:j-1);
    k14 = k14(1:j-1);
    k15 = k15(1:j-1);
    k16 = k16(1:j-1);
    k21 = k21(1:j-1);
    k22 = k22(1:j-1);
    k23 = k23(1:j-1);
    k24 = k24(1:j-1);
    k25 = k25(1:j-1);
    k26 = k26(1:j-1);
end

% 检查是否有足够的数据点进行拟合
if length(leg) < 4
    error('数据点不足，至少需要4个点进行三次多项式拟合');
end

% 进行多项式拟合
a11=polyfit(leg,k11,3);
a12=polyfit(leg,k12,3);
a13=polyfit(leg,k13,3);
a14=polyfit(leg,k14,3);
a15=polyfit(leg,k15,3);
a16=polyfit(leg,k16,3);

a21=polyfit(leg,k21,3);
a22=polyfit(leg,k22,3);
a23=polyfit(leg,k23,3);
a24=polyfit(leg,k24,3);
a25=polyfit(leg,k25,3);
a26=polyfit(leg,k26,3);

% 绘制拟合曲线
figure('Position', [100, 100, 1400, 900]);

% 第一行控制输入的各状态增益
subplot(3,4,1);
plot_fit_curve(leg, k11, a11, 'k11');
subplot(3,4,2);
plot_fit_curve(leg, k12, a12, 'k12');
subplot(3,4,3);
plot_fit_curve(leg, k13, a13, 'k13');
subplot(3,4,4);
plot_fit_curve(leg, k14, a14, 'k14');

% 第二行控制输入的各状态增益
subplot(3,4,5);
plot_fit_curve(leg, k15, a15, 'k15');
subplot(3,4,6);
plot_fit_curve(leg, k16, a16, 'k16');
subplot(3,4,7);
plot_fit_curve(leg, k21, a21, 'k21');
subplot(3,4,8);
plot_fit_curve(leg, k22, a22, 'k22');

% 第三行控制输入的各状态增益
subplot(3,4,9);
plot_fit_curve(leg, k23, a23, 'k23');
subplot(3,4,10);
plot_fit_curve(leg, k24, a24, 'k24');
subplot(3,4,11);
plot_fit_curve(leg, k25, a25, 'k25');
subplot(3,4,12);
plot_fit_curve(leg, k26, a26, 'k26');

sgtitle('LQR增益系数三次多项式拟合结果');

% 输出拟合系数
fprintf('{%.6ff, %.6ff, %.6ff, %.6ff},\n',a11(1),a11(2),a11(3),a11(4));
fprintf('{%.6ff, %.6ff, %.6ff, %.6ff},\n',a12(1),a12(2),a12(3),a12(4));
fprintf('{%.6ff, %.6ff, %.6ff, %.6ff},\n',a13(1),a13(2),a13(3),a13(4));
fprintf('{%.6ff, %.6ff, %.6ff, %.6ff},\n',a14(1),a14(2),a14(3),a14(4));
fprintf('{%.6ff, %.6ff, %.6ff, %.6ff},\n',a15(1),a15(2),a15(3),a15(4));
fprintf('{%.6ff, %.6ff, %.6ff, %.6ff},\n',a16(1),a16(2),a16(3),a16(4));

fprintf('{%.6ff, %.6ff, %.6ff, %.6ff},\n',a21(1),a21(2),a21(3),a21(4));
fprintf('{%.6ff, %.6ff, %.6ff, %.6ff},\n',a22(1),a22(2),a22(3),a22(4));
fprintf('{%.6ff, %.6ff, %.6ff, %.6ff},\n',a23(1),a23(2),a23(3),a23(4));
fprintf('{%.6ff, %.6ff, %.6ff, %.6ff},\n',a24(1),a24(2),a24(3),a24(4));
fprintf('{%.6ff, %.6ff, %.6ff, %.6ff},\n',a25(1),a25(2),a25(3),a25(4));
fprintf('{%.6ff, %.6ff, %.6ff, %.6ff}\n',a26(1),a26(2),a26(3),a26(4));

% 显示统计信息
fprintf('腿长范围: %.2f ~ %.2f m\n', min(leg), max(leg));
fprintf('有效数据点数量: %d\n', length(leg));
toc

% 定义绘制拟合曲线的函数
function plot_fit_curve(leg, k_data, coefficients, title_str)
    % 生成密集的点用于绘制平滑曲线
    leg_dense = linspace(min(leg), max(leg), 100);
    k_fit = polyval(coefficients, leg_dense);
    
    % 绘制原始数据点
    plot(leg, k_data, 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
    hold on;
    
    % 绘制拟合曲线
    plot(leg_dense, k_fit, 'r-', 'LineWidth', 2);
    
    % 在原始点上也绘制拟合值，检查拟合精度
    k_fit_points = polyval(coefficients, leg);
    plot(leg, k_fit_points, 'rx', 'MarkerSize', 8, 'LineWidth', 1.5);
    
    xlabel('腿长 (m)');
    ylabel('增益值');
    title(title_str);
    legend('原始LQR', '拟合曲线', '拟合点', 'Location', 'best');
    grid on;
    
    % 计算并显示R²
    y_mean = mean(k_data);
    SS_res = sum((k_data - k_fit_points).^2);
    SS_tot = sum((k_data - y_mean).^2);
    R_squared = 1 - SS_res / SS_tot;
    
    % 在图上添加R²信息
    text(0.05, 0.95, sprintf('R² = %.4f', R_squared), ...
         'Units', 'normalized', 'BackgroundColor', 'white', ...
         'FontSize', 8);
end