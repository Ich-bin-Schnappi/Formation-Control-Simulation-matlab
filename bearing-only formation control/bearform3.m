clc;clear;

m=10000;
delta_t = 0.01;
alpha = 0.1;
p1 = zeros(m,2);
p2 = zeros(m,2);
p3 = zeros(m,2);
p4 = zeros(m,2);
p1(1,:) = [1.2,1.3];
p2(1,:) = [1.5,1.4];
p3(1,:) = [0.2,1.3];
p4(1,:) = [-0.5,-0.4];
g21_star = [0,-1];
g31_star = [1,0];
g32_star = [2^0.5/2,2^0.5/2];
g42_star = [1,0];
g43_star = [0,-1];

theta1 = zeros(m,1);
theta2 = zeros(m,1);
theta3 = zeros(m,1);
theta4 = zeros(m,1);

theta1(1) = pi/4;
theta2(1) = pi/3;
theta3(1) = pi/2;
theta4(1) = pi/1;

I2 = [1 0
    0 1];

beta = 0;

for t = 1:m-1
    
    g21 = Cal_g(p1(t,:),p2(t,:));
    g31 = Cal_g(p1(t,:),p3(t,:));
    g32 = Cal_g(p2(t,:),p3(t,:));
    g42 = Cal_g(p2(t,:),p4(t,:));
    g43 = Cal_g(p3(t,:),p4(t,:));
    
    
    
    %计算姿态矩阵
    Q1 = Cal_Q(theta1(t));
    Q2 = Cal_Q(theta2(t));
    Q3 = Cal_Q(theta3(t));
    Q4 = Cal_Q(theta4(t));
    
%     Qat = Cal_Q(beta);
%     beta = beta + alpha*delta_t;
%     
%     g21_star = g21_star*Qat';
%     g31_star = g31_star*Qat';
%     g32_star = g32_star*Qat';
%     g42_star = g42_star*Qat';
%     g43_star = g43_star*Qat';
    
    
    
    %姿态角一致
    theta1(t+1) = theta1(t) + delta_t*( theta2(t) - theta1(t) + theta3(t) - theta1(t) );
    theta2(t+1) = theta2(t) + delta_t*( theta1(t) - theta2(t) + theta3(t) - theta2(t) + theta4(t) - theta2(t));
    theta3(t+1) = theta3(t) + delta_t*( theta1(t) - theta3(t) + theta2(t) - theta3(t) + theta4(t) - theta3(t));
    theta4(t+1) = theta4(t) + delta_t*( theta2(t) - theta4(t) + theta3(t) - theta4(t) );
    

    
    %非全局坐标系下
    p1(t+1,:) = p1(t,:) - delta_t*(   -g21_star*(I2+Q2'*Q1)*Cal_P(g21*Q1,2) - g31_star*(I2+Q3'*Q1)*Cal_P(g31*Q1,2)  )*Q1';
    p2(t+1,:) = p2(t,:) - delta_t*(   g21_star*(I2+Q1'*Q2)*Cal_P(g21*Q2,2) - g32_star*(I2+Q3'*Q2)*Cal_P(g32*Q2,2)  - g42_star*(I2+Q4'*Q2)*Cal_P(g42*Q2,2))*Q2';
    p3(t+1,:) = p3(t,:) - delta_t*(   g31_star*(I2+Q1'*Q3)*Cal_P(g31*Q3,2) + g32_star*(I2+Q2'*Q3)*Cal_P(g32*Q3,2)  - g43_star*(I2+Q4'*Q3)*Cal_P(g43*Q3,2))*Q3';
    p4(t+1,:) = p4(t,:) - delta_t*(   g42_star*(I2+Q2'*Q4)*Cal_P(g42*Q4,2) + g43_star*(I2+Q3'*Q4)*Cal_P(g43*Q4,2)  )*Q4';
    
    
    
    %全局坐标系下
%     p1(t+1,:) = p1(t,:) - delta_t*(   -g21_star*Cal_P(g21,2) - g31_star*Cal_P(g31,2)  );
%     p2(t+1,:) = p2(t,:) - delta_t*(   g21_star*Cal_P(g21,2) - g32_star*Cal_P(g32,2)  - g42_star*Cal_P(g42,2));
%     p3(t+1,:) = p3(t,:) - delta_t*(   g31_star*Cal_P(g31,2) + g32_star*Cal_P(g32,2)  - g43_star*Cal_P(g43,2));
%     p4(t+1,:) = p4(t,:) - delta_t*(   g42_star*Cal_P(g42,2) + g43_star*Cal_P(g43,2)  );
    
end

%画姿态角
plot(theta1);
hold on;
plot(theta2);
hold on;
plot(theta3);
hold on;
plot(theta4);
figure;

% 画点
plot(p1(t,1),p1(t,2),'ro');
hold on;
plot(p2(t,1),p2(t,2),'ro');
hold on;
plot(p3(t,1),p3(t,2),'ro');
hold on;
plot(p4(t,1),p4(t,2),'ro');
hold on;
%legend('leader', 'follower');

%画结构
plot([p1(t,1),p2(t,1)],[p1(t,2),p2(t,2)],'k')
hold on
plot([p1(t,1),p3(t,1)],[p1(t,2),p3(t,2)],'k')
hold on
plot([p4(t,1),p2(t,1)],[p4(t,2),p2(t,2)],'k')
hold on
plot([p3(t,1),p4(t,1)],[p3(t,2),p4(t,2)],'k')
hold on
plot([p2(t,1),p3(t,1)],[p2(t,2),p3(t,2)],'k')
hold on


% 画路线
plot(p1(:,1),p1(:,2),'b')
hold on
plot(p2(:,1),p2(:,2),'b')
hold on
plot(p3(:,1),p3(:,2),'b')
hold on
plot(p4(:,1),p4(:,2),'b')
hold on


%画初始点
plot(p3(1,1),p3(1,2),'bo')
hold on
plot(p4(1,1),p4(1,2),'bo')
hold on
plot(p1(1,1),p1(1,2),'bo')
hold on
plot(p2(1,1),p2(1,2),'bo')

axis auto equal

% % 画位置误差
% figure;
% plot(x,e3(:,1),'b')
% hold on
% plot(x,e4(:,1),'g')
% hold on
% plot(x,e1(:,1),'c')
% hold on
% plot(x,e2(:,1),'m')
% xlabel('Time/s');
% ylabel('Position Error');
% 
% % 画方位角误差
% figure;
% plot(x,eg31(:,1),'k')
% hold on
% plot(x,eg32(:,1),'b')
% hold on
% plot(x,eg42(:,1),'c')
% hold on
% plot(x,eg43(:,1),'m')
% hold on
% plot(x,eg53(:,1),'g')
% hold on
% plot(x,eg54(:,1),'r')
% hold on
% plot(x,eg64(:,1),'y')
% hold on
% plot(x,eg65(:,1),'m')
% xlabel('Time/s');
% ylabel('Bearing Error');












