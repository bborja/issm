%%
% Originally a part of: Maggot (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function component = momentMatchPdf(Mu1, Mu2, Cov1, Cov2, w)
%#codegen


sum_w = sum(w) ;

w = w/sum_w ;

% % new_mu =  sum(bsxfun(@times,Mu, w),2) ;
%new_mu =  sum(bsxfun(@times,[Mu1,Mu2], w),2) ;
a = [Mu1,Mu2];        
multi=zeros(size(a));
for k=1:size(a,2),
    multi(:,k)= a(:,k).*w(k);
end
new_mu=sum(multi,2);

% %n = size(new_mu,1) ;
% %     new_Cov = zeros(n,n) ;
% %    if n==1
% %         new_Cov = sum(w.*(cell2mat(Cov) + Mu.*Mu)) ;
% %     else
% %         for j=1:length(w)
% %             
% %             new_Cov = new_Cov + w(j)*( Cov{j} + Mu(:,j)*Mu(:,j)') ;
% %         end
% %     end
% %     new_Cov = new_Cov - new_mu*new_mu' ;
component=MixtureComponent(1);
component.Mu=w(1)*Mu1+w(2)*Mu2;
component.Cov=w(1)*(Cov1+Mu1*Mu1')+w(2)*(Cov2+Mu2*Mu2')-new_mu*new_mu';

component.w=sum_w;
end

