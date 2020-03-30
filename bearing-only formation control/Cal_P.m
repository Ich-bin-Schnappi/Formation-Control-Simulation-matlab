function P = Cal_P(g, n)
%UNTITLED3 此处显示有关此函数的摘要
%   此处显示详细说明
P = eye(n) - g'*g;
end

