function [a, sel_xy] = getOptimalLineImage_constrained(xy, ~, ptsinit, delta)
%#codegen


a = zeros(3,1);


sel_xy = [] ;
[U,~,~]=svd(cov(ptsinit')) ;

a(1:2) = U(:,2) ;
a(3) = -[a(1), a(2)]*mean(ptsinit,2) ;

min_val = 1e-10 ;
a0 = a;
sigma = delta ; %[delta, delta/4 ] ;
for i = 1 : length(sigma)
    for j = 1 : 5
        
        rr = abs(a(1)*xy(1,:) + a(2)*xy(2,:) + a(3))/sigma(i) ;
        zerovls = rr>2 ;
        w = exp( -0.5*rr.^2 ) ;
        
        w(zerovls) = 0 ;
        w = w/sum(w) ;
        
        %mnxy = sum(bsxfun(@times, xy, w),2) ;  
        xyw=zeros(size(xy));
        for k=1:size(xy,1),
            xyw(k,:)= xy(k,:).*w;
        end
        mnxy=sum(xyw,2);
        
        %wd = bsxfun(@times, bsxfun(@minus,xy,mnxy), sqrt(w)) ;
        sub=zeros(size(xy));
        for k=1:size(xy,2),
            sub(:,k)= xy(:,k)-mnxy;
        end
        
        wd=zeros(size(sub));
        for k=1:size(sub,1),
            wd(k,:)= sub(k,:).*sqrt(w);
        end
        
        
        C = wd*wd' ;
        [U,~,~] = svd(C);
        a(1:2) = U(:,2) ;  a(3) = -[a(1), a(2)]*mnxy ;
        
        if mean(abs(a0-a)) < min_val
            break ;
        end
        a0 = a ;
        
    end
end
if nargout > 1
    sel_xy = xy(:,zerovls==0) ;
end
end