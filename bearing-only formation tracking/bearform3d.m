clc;clear;
%% ³õÊ¼»¯
t=length(0:0.1:100);
d = 3;
n = 8;
p = zeros(n, d, t);

p(:,:,1) = [0 1 1
            2 1 1
            3 1 1
            6 1 1
            0 2 1
            2 2 1
            3 2 1
            6 2 1];


g_star = [0 1 0
          -1 0 0
          0 -1 0
          1 0 0
          0 0 1
          0 0 1
          0 0 1
          0 0 1
          0 1 0
          -1 0 0
          0 -1 0
          1 0 0
          3^0.5/3 -3^0.5/3 3^0.5/3];

H = [-1 1 0 0 0 0 0 0
     0 -1 1 0 0 0 0 0
     0 0 -1 1 0 0 0 0
     1 0 0 -1 0 0 0 0
     -1 0 0 0 1 0 0 0
     0 -1 0 0 0 1 0 0
     0 0 -1 0 0 0 1 0
     0 0 0 -1 0 0 0 1
     0 0 0 0 -1 1 0 0
     0 0 0 0 0 -1 1 0
     0 0 0 0 0 0 -1 1
     0 0 0 0 1 0 0 -1
     0 0 -1 0 1 0 0 0];

m = size(g_star,1);

Id = eye(d);
H_bar = kron(H, Id);


%% Control with a global reference frame

for i=1:(t-1)
    
    g = CalG(p(:,:,i), H);
%     DP = blkdiag(eye(d)-g(1,:)'*g(1,:),eye(d)-g(2,:)'*g(2,:),eye(d)-g(3,:)'*g(3,:),eye(d)-g(4,:)'*g(4,:),eye(d)-g(5,:)'*g(5,:));
    
    DP = blkdiag(eye(d)-g(1,:)'*g(1,:),eye(d)-g(2,:)'*g(2,:),eye(d)-g(3,:)'*g(3,:),eye(d)-g(4,:)'*g(4,:),...
        eye(d)-g(5,:)'*g(5,:),eye(d)-g(6,:)'*g(6,:),eye(d)-g(7,:)'*g(7,:),eye(d)-g(8,:)'*g(8,:),...
        eye(d)-g(9,:)'*g(9,:),eye(d)-g(10,:)'*g(10,:),eye(d)-g(11,:)'*g(11,:),eye(d)-g(12,:)'*g(12,:),...
        eye(d)-g(13,:)'*g(13,:));
    
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

%% ×÷Í¼
x1 = zeros(t,1);y1 = zeros(t,1);z1 = zeros(t,1);
x2 = zeros(t,1);y2 = zeros(t,1);z2 = zeros(t,1);
x3 = zeros(t,1);y3 = zeros(t,1);z3 = zeros(t,1);
x4 = zeros(t,1);y4 = zeros(t,1);z4 = zeros(t,1);
x5 = zeros(t,1);y5 = zeros(t,1);z5 = zeros(t,1);
x6 = zeros(t,1);y6 = zeros(t,1);z6 = zeros(t,1);
x7 = zeros(t,1);y7 = zeros(t,1);z7 = zeros(t,1);
x8 = zeros(t,1);y8 = zeros(t,1);z8 = zeros(t,1);




for j=1:t
    x1(j) = p(1,1,j);y1(j) = p(1,2,j);z1(j) = p(1,3,j);
    x2(j) = p(2,1,j);y2(j) = p(2,2,j);z2(j) = p(2,3,j);
    x3(j) = p(3,1,j);y3(j) = p(3,2,j);z3(j) = p(3,3,j);
    x4(j) = p(4,1,j);y4(j) = p(4,2,j);z4(j) = p(4,3,j);
    x5(j) = p(5,1,j);y5(j) = p(5,2,j);z5(j) = p(5,3,j);
    x6(j) = p(6,1,j);y6(j) = p(6,2,j);z6(j) = p(6,3,j);
    x7(j) = p(7,1,j);y7(j) = p(7,2,j);z7(j) = p(7,3,j);
    x8(j) = p(8,1,j);y8(j) = p(8,2,j);z8(j) = p(8,3,j);
    
end

plot3(x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,x5,y5,z5,x6,y6,z6,x7,y7,z7,x8,y8,z8)
hold on
plot3(x1(t),y1(t),z1(t),'o',x2(t),y2(t),z2(t),'o',x3(t),y3(t),z3(t),'o',x4(t),y4(t),z4(t),...
    'o',x5(t),y5(t),z5(t),'o',x6(t),y6(t),z6(t),'o',x7(t),y7(t),z7(t),'o',x8(t),y8(t),z8(t),'o')
hold on

plot3([x1(t),x2(t)],[y1(t),y2(t)],[z1(t),z2(t)],'k');hold on
plot3([x3(t),x2(t)],[y3(t),y2(t)],[z3(t),z2(t)],'k');hold on
plot3([x3(t),x4(t)],[y3(t),y4(t)],[z3(t),z4(t)],'k');hold on
plot3([x4(t),x1(t)],[y4(t),y1(t)],[z4(t),z1(t)],'k');hold on
plot3([x1(t),x5(t)],[y1(t),y5(t)],[z1(t),z5(t)],'k');hold on
plot3([x2(t),x6(t)],[y2(t),y6(t)],[z2(t),z6(t)],'k');hold on
plot3([x3(t),x7(t)],[y3(t),y7(t)],[z3(t),z7(t)],'k');hold on
plot3([x4(t),x8(t)],[y4(t),y8(t)],[z4(t),z8(t)],'k');hold on
plot3([x5(t),x6(t)],[y5(t),y6(t)],[z5(t),z6(t)],'k');hold on
plot3([x6(t),x7(t)],[y6(t),y7(t)],[z6(t),z7(t)],'k');hold on
plot3([x7(t),x8(t)],[y7(t),y8(t)],[z7(t),z8(t)],'k');hold on
plot3([x8(t),x5(t)],[y8(t),y5(t)],[z8(t),z5(t)],'k');hold on
plot3([x3(t),x5(t)],[y3(t),y5(t)],[z3(t),z5(t)],'k');hold on

axis auto equal























