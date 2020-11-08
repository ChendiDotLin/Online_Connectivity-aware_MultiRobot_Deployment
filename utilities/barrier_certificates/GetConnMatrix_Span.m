%% Get Spanning Tree Connectivity Matrix and corresponding controller
%% from A Connected Graph in A Fully Distributed Manner
% Input:    xx  --  2 x N , N is number of robots
%           R2  --  scale   the square of communication radius R_communication
% Output:   conn_matrix -- M x 2,   M is the number of edges fo MST,
%           entries are the corresponding node number
%           ctrl_flag -- N x 1, N is the number of robots, 0 for main
%           controller 0 and 1 for emergent controller 1
% Created by Wenhao Luo (whluo12@gmail.com)
% 6/21/2018

function [conn_matrix, ctrl_flag, adj] = GetConnMatrix_Span(xx, R2, L0, varargin)


parser = inputParser;
addOptional(parser, 'select_flag', 0); % how to select break-off swarm
% 0-maximum, 99-minimum,  any other number directly corresponds to # of
% branch
parse(parser, varargin{:});

select_flag = parser.Results.select_flag;


N = size(xx,2);
adj=zeros(N,N); % N x N matrix
ctrl_flag = zeros(1,N);  % 0 for default controller and 1 for emergent controller (split out)
% adjacency matrix is symmetricle so fill only up triangle and mirror
% around diagonal:
global message_count
message_count = 0;

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

% start initilization of bots
bots(N,1) = struct();
bots = init_bot(bots, adj, L0);
message_queque = cell(N,1);

% initialize communication, send messages to all neighbors
for ijk = 1:N  
    message_queque = msg_send(bots(ijk).message, bots(ijk).neighbor_idx, message_queque);
end

% start communication round

while ~isempty([message_queque{:}])
    for ijk = 1:N
        % each bot start to process asynchornously      
        while ~isempty(message_queque{ijk})
            % process each message
            get_msg = message_queque{ijk}(:,1);
            message_queque{ijk}(:,1)=[]; % clear processed msg
            msg_n = get_msg(1);
            msg_m = get_msg(2);
            msg_L = get_msg(3);
            msg_h = get_msg(4);
            msg_mem_sum = get_msg(5); 
            neighbor_iidx_vec = bots(ijk).neighbor_idx==msg_n; % message decoding
            
            % reset count_bin, member_count and mem_sum for counting number
            % of branch members
            if msg_m~=ijk             
                bots(ijk).count_bin(neighbor_iidx_vec) = 0;
            else
                bots(ijk).count_bin(neighbor_iidx_vec) = 1;  
            end            
            bots(ijk).member_count(neighbor_iidx_vec) = msg_mem_sum;
            bots(ijk).mem_sum = dot(bots(ijk).count_bin, bots(ijk).member_count) + 1; % plus the bot itself
            
            % start standard message processing
            if (bots(ijk).L>msg_L)||((bots(ijk).L==msg_L)&&(bots(ijk).h>msg_h+1))
                bots(ijk).L = msg_L;
                bots(ijk).h = msg_h+1;
                bots(ijk).m = msg_n;
                bots(ijk).message = [ijk bots(ijk).m bots(ijk).L bots(ijk).h bots(ijk).mem_sum]';
                
                message_queque = msg_send(bots(ijk).message, bots(ijk).neighbor_idx, message_queque);%send msg to neighbors
                
            elseif (bots(ijk).L==msg_L)&&(ijk==msg_m)&&(bots(ijk).m~=0)
                bots(ijk).message = [ijk bots(ijk).m bots(ijk).L bots(ijk).h bots(ijk).mem_sum]';
                message_queque = msg_send(bots(ijk).message, bots(ijk).m, message_queque);                
            end      
        end
    end  
end

% Get edge list from converged spanning tree
m_vec = [bots.m];
s_node = 1:N;
s_node = s_node(m_vec>0);
t_node = m_vec(m_vec>0);
conn_matrix = [s_node' t_node'];

%% debug session %%

% G_new = graph(s_node, t_node);% debug only, show graph of the converged spanning tree
% figure;plot(G_graph,'Layout','force');
% figure;plot(G_new,'Layout','force');
%% debug session end

% start to assign controller, namely pick up corresponding branch

mem_sum_stack = [bots(L0).member_count];

% 0-maximum, 99-minimum,  any other number directly corresponds to # of
% branch

if select_flag==0
    [best_num,best_m_iidx] = max(mem_sum_stack);
elseif select_flag==99
    [best_num,best_m_iidx] = min(mem_sum_stack);
else
    best_m_iidx = select_flag;    
end
current_m_set = bots(L0).neighbor_idx(best_m_iidx);  % select idx of 1-hop bot that has largest branch (can be done by root bot)
ctrl_flag(current_m_set) = 1; % first set the controller of the selected 1-hop bot, then start the loop

while ~isempty(current_m_set)
    new_m_set = [];
    for m_idx = 1:numel(current_m_set)
        current_m = current_m_set(m_idx);
        ctrl_flag(m_vec==current_m) = 1; % set all followers' controller
        new_m_set = [new_m_set find(m_vec==current_m)];
    end
    current_m_set = new_m_set;
end

% 
% 
% T_tree = minspantree(G_graph,'Type','tree');
% conn_matrix = table2array(T_tree.Edges);
% conn_matrix = conn_matrix(:,1:2);  % used to define minimum pairwise connectivity constraint

% figure;plot(G_graph,'Layout','force'); % debug only, to plot comm graph
% figure;plot(T_tree,'Layout','force'); % debug only, to plot span tree

end

function bots = init_bot(bots, adj, L0)
% initialization of bots

N = numel(bots);

for ijk =1:N
  
    bots(ijk).m = 0;
    bots(ijk).neighbor_idx = find(adj(ijk,:));
    bots(ijk).count_bin = zeros(1,numel(bots(ijk).neighbor_idx));
    bots(ijk).member_count = zeros(1,numel(bots(ijk).neighbor_idx));
    bots(ijk).L = abs(ijk-L0);
    bots(ijk).h = 0;
    bots(ijk).mem_sum = dot(bots(ijk).count_bin, bots(ijk).member_count) + 1;
    bots(ijk).message = [ijk bots(ijk).m bots(ijk).L bots(ijk).h bots(ijk).mem_sum]';
end

end

function message_queque = msg_send(message, neighbor_idx, message_queque)
% send message to message_queque
% from one bot to some set of other bots
global message_count
num_receiver = numel(neighbor_idx);
for ijk = 1:num_receiver
    message_queque{neighbor_idx(ijk)} = [message_queque{neighbor_idx(ijk)} message];
    message_count = message_count + 1;
end
end