%% Get Connectivity Matrix from Minimum Spanning Tree (MST) of A Connected Graph
% Input:    xx  --  2 x N , N is number of robots
%           R2  --  scale   the square of communication radius R_communication
% Output:   conn_matrix -- M x 2,   M is the number of edges fo MST,
%           entries are the corresponding node number
% Created by Wenhao Luo (whluo12@gmail.com)
% 4/23/2018

function conn_matrix = GetConnMatrix(xx, R2)
N = size(xx,2);
adj=zeros(N,N); % N x N matrix
% adjacency matrix is symmetricle so fill only up triangle and mirror
% around diagonal:

for n1=1:N-1
    r1=xx(1:2,n1); % first node
    for n2=n1+1:N
        r2=xx(1:2,n2); % second node
        dr=r1-r2;
        dr2=dr'*dr; % distance between nodes squared
        if dr2<R2 % if for squres true then sqrt(dr2)<R is true
            adj(n1,n2)=1; % n1 and n2 connected
            adj(n2,n1)=1; % if n2 connected to n1, then n1 connected to n2, matrix adj is symmetricle
        end
    end
end

G_graph = graph(adj);

% determine if the robotic swarm is disconnected at the beginning.
if numel(unique(conncomp(G_graph)))~=1   %~isempty(find(all(logical(adj)==0,2), 1))
    error('robotic swarm not connected!');
end

T_tree = minspantree(G_graph,'Type','tree');
conn_matrix = table2array(T_tree.Edges);
conn_matrix = conn_matrix(:,1:2);  % used to define minimum pairwise connectivity constraint

% figure;plot(G_graph,'Layout','force'); % debug only, to plot comm graph
% figure;plot(T_tree,'Layout','force'); % debug only, to plot span tree

end