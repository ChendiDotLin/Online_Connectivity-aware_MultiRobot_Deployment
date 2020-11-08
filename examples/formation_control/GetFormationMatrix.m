% Get Laplacian Matrix and the weights matrix defining desired inter-robot
% distance to form a rectangle formation
% Wenhao Luo (whluo12@gmail.com)
%

%with size of S x L: side order from bot 1 (left-top) and clockwise:
%l-s-l-s

%Require - number of robots = N = 4*n where n is the number of robots on
%each side of the 4 sides of the rectangle
%To form a rigid formation, we need 2*N-3, which in our algorithm boils
%down to (3N-N-4)+1 = 2N-3
%3N-N-4: each robot has three edges connecting to 1) both of its direct
%neighbours and 2) the corresponding vertex furthest on the same side
%anti-clockwise. Example:  if 1<-2-<3-<4 for the top side, then for bot 3
%the topology is 3-2, 3-4, and 3-1
%+1:  we add the edge connecting bot 1 (left-top) to the diagnal bot 2n+1, so for bot 1 the topology is: 1-2, 1-4n, and 1-(2n+1)

function [L,weights] = GetFormationMatrix(N,d1,d2)

% d1=l/n   desired segment length on top and bottom sides
% d2=s/n   desired segment length on left and right sides

% N number of robots

if mod(N,4)
    error('number of robots should be 4n with n an integer');
end
n = floor(N/4);

l_length = n*d1;
s_length = n*d2;

L = 3*eye(N);
weights = zeros(N,N);

for ijk = 1:N
    
    % idx_right = rem(ijk,N)+1;
    if ijk==1
        idx_left = N;
        idx_right = ijk+1;
    elseif ijk==N
        idx_left = ijk-1;
        idx_right = 1;
    else
        idx_left = ijk-1;
        idx_right = ijk+1;
    end
    
    L(ijk,idx_left) = -1;
    L(ijk,idx_right) = -1;
    
    if ijk==1
        weights(ijk,idx_left) = d2;
        weights(ijk,idx_right) = d1;
        weights(ijk,1) = d1*(ijk-1);
        weights(ijk,2*n+1) = sqrt(l_length^2+s_length^2);
    elseif (ijk>1)&&(ijk<n+1)
        weights(ijk,idx_left) = d1;
        weights(ijk,idx_right) = d1;
        weights(ijk,1) = d1*(ijk-1);
    elseif ijk==n+1
        weights(ijk,idx_left) = d1;
        weights(ijk,idx_right) = d2;
        weights(ijk,1) = d1*(ijk-1);
        weights(ijk,3*n+1) = sqrt(l_length^2+s_length^2);
    elseif (ijk>n+1)&&(ijk<2*n+1)
        weights(ijk,idx_left) = d2;
        weights(ijk,idx_right) = d2;
        weights(ijk,n+1) = d2*(ijk-n-1);
    elseif ijk==2*n+1
        weights(ijk,idx_left) = d2;
        weights(ijk,idx_right) = d1;
        weights(ijk,n+1) = d2*(ijk-n-1);
    elseif (ijk>2*n+1)&&(ijk<3*n+1)
        weights(ijk,idx_left) = d1;
        weights(ijk,idx_right) = d1;
        weights(ijk,2*n+1) = d1*(ijk-2*n-1);
    elseif ijk==3*n+1
        weights(ijk,idx_left) = d1;
        weights(ijk,idx_right) = d2;
        weights(ijk,2*n+1) = d1*(ijk-2*n-1);
    elseif ijk>3*n+1
        weights(ijk,idx_left) = d2;
        weights(ijk,idx_right) = d2;
        weights(ijk,3*n+1) = d2*(ijk-3*n-1);              
    end
    
    
    
end



end
