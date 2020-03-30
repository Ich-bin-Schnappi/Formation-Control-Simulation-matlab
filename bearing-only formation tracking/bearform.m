clc;clear;
%% 初始化
t=length(0:0.1:400);
d = 2;
n = 4;
p = zeros(n, d, t);

%二维
p(:,:,1) = [0 1
            2 1
            3 1
            6 1];

g_star = [0 -1
          -1 0
          -2^0.5/2 2^0.5/2
          -1 0
          0 1];

H = [-1 1 0 0
     -1 0 0 1
     0 -1 0 1
     0 -1 1 0
     0 0 -1 1];

m = size(g_star,1);

Id = eye(d);
H_bar = kron(H, Id);


%% Control with a global reference frame

for i=1:t
    
    g = CalG(p(:,:,i), H);
    DP = blkdiag(eye(d)-g(1,:)'*g(1,:),eye(d)-g(2,:)'*g(2,:),eye(d)-g(3,:)'*g(3,:),eye(d)-g(4,:)'*g(4,:),eye(d)-g(5,:)'*g(5,:));
    v = H_bar'*DP*reshape(g_star',m*d,1);
    p(:,:,i+1) = reshape(v,d,n)' .* 0.1 + p(:,:,i);
    
end

%% Control without a global reference frame
% for i=1:n
%     
%     
%     
%     
%     
% end

%% 作图
x1 = zeros(t,1);
y1 = zeros(t,1);
x2 = zeros(t,1);
y2 = zeros(t,1);
x3 = zeros(t,1);
y3 = zeros(t,1);
x4 = zeros(t,1);
y4 = zeros(t,1);
for j=1:t
    x1(j) = p(1,1,j);
    y1(j) = p(1,2,j);
    
    x2(j) = p(2,1,j);
    y2(j) = p(2,2,j);
    
    x3(j) = p(3,1,j);
    y3(j) = p(3,2,j);
    
    x4(j) = p(4,1,j);
    y4(j) = p(4,2,j);
end

plot(x1,y1,x2,y2,x3,y3,x4,y4)
hold on
plot(x1(t),y1(t),'o',x2(t),y2(t),'o',x3(t),y3(t),'o',x4(t),y4(t),'o')
hold on

plot([x1(t),x2(t)],[y1(t),y2(t)],'k')
hold on
plot([x3(t),x2(t)],[y3(t),y2(t)],'k')
hold on
plot([x1(t),x4(t)],[y1(t),y4(t)],'k')
hold on
plot([x4(t),x2(t)],[y4(t),y2(t)],'k')
hold on
plot([x3(t),x4(t)],[y3(t),y4(t)],'k')
hold on

axis auto equal























