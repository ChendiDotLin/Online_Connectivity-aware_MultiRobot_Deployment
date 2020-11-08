%% Get Connectivity Graph and weight assignments for the connectivity graph
% Input:    xx  --  2 x N , N is number of robots
%           R2  --  scale   the square of communication radius R_communication
%           dxi --  2 x N , N is number of robots
% Output:   G_conn_grph --    
%           G_weights ----
% Created by Wenhao Luo (whluo12@gmail.com)
% 9/04/2018

function [G_conn_graph, G_weights, MST_conn_matrix, adj] = GetGraphWeight(xx, R2, dxi, varargin)

parser = inputParser;
addOptional(parser, 'ctrl_flag', []); % how to select break-off swarm

parse(parser, varargin{:});

ctrl_flag = parser.Results.ctrl_flag;

gamma = 1e4;

N = size(xx,2);
adj=zeros(N,N); % N x N matrix
% adjacency matrix is symmetricle so fill only up triangle and mirror
% around diagonal:
s_node = [];
t_node = []; % store vertex lists of connectivity graph
G_weights = [];

for n1=1:N-1
    r1=xx(1:2,n1); % first node
    for n2=n1+1:N
        r2=xx(1:2,n2); % second node
        dr=r1-r2;
        dr2=dr'*dr; % distance between nodes squared
        if dr2<R2 % if for squres true then sqrt(dr2)<R is true
            s_node = [s_node n1];
            t_node = [t_node n2];
            adj(n1,n2)=1; % n1 and n2 connected
            adj(n2,n1)=1; % if n2 connected to n1, then n1 connected to n2, matrix adj is symmetricle
            
            delta = -2*((xx(1:2,n1)-xx(1:2,n2))'*(dxi(1:2,n1)-dxi(1:2,n2)))+gamma*(R2-norm(xx(1:2,n1)-xx(1:2,n2))^2)^3;
            
            if ~isempty(ctrl_flag)
                if ctrl_flag(n1)~=ctrl_flag(n2)
                    delta = delta/1e4;
                end
            
            end
            
            
            G_weights = [G_weights -delta];            
        end
    end
end

G_conn_graph = graph(s_node,t_node,G_weights); %graph(adj);

% determine if the robotic swarm is disconnected at the beginning.
% if numel(unique(conncomp(G_conn_graph)))~=1   %~isempty(find(all(logical(adj)==0,2), 1))
%     error('robotic swarm not connected!');
% end

T_tree = minspantree(G_conn_graph,'Type','tree');
MST_conn_matrix = table2array(T_tree.Edges);
MST_conn_matrix = MST_conn_matrix(:,1:2);  % used to define minimum pairwise connectivity constraint

% conn_matrix = table2array(T_tree.Edges);
% conn_matrix = conn_matrix(:,1:2);  % used to define minimum pairwise connectivity constraint



end