function g = CalG(p, H)
% 计算角度向量矩阵g
    e = H*p;
    e_norm = zeros(size(H,1),1);
    g = zeros(size(H,1),size(p,2));
    for i = 1:size(e, 1)
        e_norm(i) = norm(e(i,:));
        if (e_norm(i)~=0)
            g(i,:) = e(i,:)./e_norm(i);
        else
            g(i,:) = e(i,:);
        end
    end
    
    
    
    





