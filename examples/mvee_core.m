function [A,x0,coreset,core_ind] = mvee_core(P,epsilon)

% Input: 
%         P is a matrix whose columns are the coordinates of each swarm
% robot. P is a d X N matrix.

% Output: 
%         A---d X d symmetric positive definite matrix describing the shape
% of the rendered ellipsoid
%         x0--d X 1 column vector describing the coordinates of the center
%         of ellipsoid.
%         coreset---  core set matrix whose columns are the coordinates of each
%         core robots.
%         K--- convex hull matrix whose columns are the corrdinates of each
%         robot locating at the convex hull of P.
%
%         Created By: Wenhao Luo, and revised on 2/23/2015


[d,n] = size(P);
userflag = 0;
% K = convhulln(P');
% K = unique(K(:)); %return the convex hull of P
% convexhull = P(:,K);
core_ind=[];

ba=P';cc=unique(ba,'rows');ba=cc';
nn = size(ba,2);
%% Core Set Initialization
% if nn <=2*d
% coreset = P; % directly use the initial set as core-sets without duplicated point
% core_ind = 1:n;
% % coreset = P(:,P_index);
% % core_ind = [];
% % core_ind = P_index;
% A=[];
% x0=[];
% return;
% 
% else
%      direction=eye(d);
     direction=[1/sqrt(2),-1/sqrt(2);1/sqrt(2),1/sqrt(2)];
%    direction=[1/2,-sqrt(3)/2;sqrt(3)/2,1/2];
% direction=[1/sqrt(2),-1/sqrt(2);1/sqrt(2),1/sqrt(2)];
    for i=1:d
        b=direction(:,i);
        proj_vec=b'*P;
        [max_value,max_index]=max(proj_vec);
        core_ind=[core_ind,max_index];
        Q(:,2*i-1)=P(:,max_index);
        [min_value,min_index]=min(proj_vec);
         core_ind=[core_ind,min_index];
        Q(:,2*i)=P(:,min_index);
    end
% end

%% check if there is possible singularity problem


ba=Q';cc=unique(ba,'rows');ba=cc';

% if (size(ba,2) ~= size(Q,2))
if 1
    userflag = issameline(ba);
    if userflag        
        coreset = P; % directly use the initial set as core-sets without duplicated point
        core_ind = 1:n; 
        A=[];
        x0=[];     
        return;
    end   
end



%  ba=Q';cc=unique(ba,'rows');ba=cc';
%  
%  if size(ba,2) ~= size(Q,2)
%     if n <=2*d
% Q=P; % directly use the initial set as core-sets
% core_ind = 1:numel(n);
% 
% P_index = unique(core_ind);
% coreset = P(:,P_index);
% core_ind = [];
% core_ind = P_index;
% A=[];
% x0=[];
% return;
% 
% else
%     direction=[1/sqrt(2),-1/sqrt(2);1/sqrt(2),1/sqrt(2)];
% %    direction=[1/2,-sqrt(3)/2;sqrt(3)/2,1/2];
% % direction=[1/sqrt(2),-1/sqrt(2);1/sqrt(2),1/sqrt(2)];
%     for i=1:d
%         b=direction(:,i);
%         proj_vec=b'*P;
%         [max_value,max_index]=max(proj_vec);
%         core_ind=[core_ind,max_index];
%         Q(:,2*i-1)=P(:,max_index);
%         [min_value,min_index]=min(proj_vec);
%          core_ind=[core_ind,min_index];
%         Q(:,2*i)=P(:,min_index);
%     end
% end 
%  end
%% Core Set updating and Ellispoid Calculation

for i=1:ceil(d*log(d)+d/epsilon)
    [A, x0] = lowner(Q,epsilon);
    
    for j=1:n
        v(j)=(P(:,j)-x0)'*A*(P(:,j)-x0)-(1 + 1 * epsilon)^2;
    end
    [max_v,max_vindex]=max(v);
    if max_v < 0
    coreset=Q;
    
    P_index = unique(core_ind);
    coreset = P(:,P_index);
    core_ind = [];
    core_ind = P_index;   
    
    return;
    else
    Q(1:end,end+1)=P(:,max_vindex);
     core_ind=[core_ind,max_vindex];
    end
end
coreset=Q;


% newly revision

P_index = unique(core_ind);
coreset = P(:,P_index);
core_ind = [];
core_ind = P_index;

end


