% MVEE and coreset computation with MATLAB
% CASE 2015 paper CH-KY re-implementation
% Created by Wenhao Luo (whluo12@gmail.com)
% July 11 2018

function mvee_algo()

% % start algo

num_bot_stack = 5;
num_bot_set = [10 20 30 40 50];
bot_data = cell(0,1);
num_trial = 10;
r = 0.3; % communication range


% generate valid data

for ijk = 1:num_bot_stack
    ijk
    current_set = [];
    for ijk_t = 1:num_trial
        conn_flag = false;
        while ~conn_flag
            current_data = rand(2, num_bot_set(ijk));
            conn_flag = CheckConn(current_data, r^2);
        end
        current_set(:,:,ijk_t) = current_data;
    end
    bot_data{ijk} = current_set;
end
%
%
% % save('mvee_algo_good_data.mat');
%
% % or load valid data

% load('mvee_algo_good_data.mat'); % all data above will be loaded here.

num_message_set = zeros(num_bot_stack, num_trial); % 5 x 10 store message from each of the 10 trials
num_core_set = zeros(num_bot_stack, num_trial);
area_lowner_set = zeros(num_bot_stack, num_trial);
area_core_set = zeros(num_bot_stack, num_trial);
area_rate_set = zeros(num_bot_stack, num_trial);
valid_flag_set = zeros(num_bot_stack, num_trial);
L0 = 1; % specify the root bot
epsilon = 0.01;

global message_count


for ijk_num_bot = 1:num_bot_stack
    for ijk_t = 1:num_trial
        ijk_num_bot
        ijk_t
        message_count = 0;
        current_pos = bot_data{ijk_num_bot}(:,:,ijk_t);
        N = size(current_pos,2);
        adj = GetAdjMatrix(current_pos, r^2);
        % start initilization of bots
        bots = struct(); % bots(N,1)=struct();
        bots = init_bot(bots, adj, L0);
        message_queque = cell(N,1);
        
        % [~,~,coreset,Ks] = mvee_core(temp2_C,epsilon);

        % initialize computation and communication, send messages to all neighbors
        for ijk = 1:N
            tmp_unique_idx = unique(bots(ijk).CH_idx);
            [~,~,~,tmp_idx_idx] = mvee_core(current_pos(:,tmp_unique_idx),epsilon);            
            bots(ijk).core_idx = tmp_unique_idx(tmp_idx_idx);
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
                    msg_CH_idx = get_msg(5:end);
                   
%                     neighbor_iidx_vec = bots(ijk).neighbor_idx==msg_n; % message decoding
%                     
%                     % reset count_bin, member_count and mem_sum for counting number
%                     % of branch members
%                     if msg_m~=ijk
%                         bots(ijk).count_bin(neighbor_iidx_vec) = 0;
%                     else
%                         bots(ijk).count_bin(neighbor_iidx_vec) = 1;
%                     end
%                     bots(ijk).member_count(neighbor_iidx_vec) = msg_mem_sum;
%                     bots(ijk).mem_sum = dot(bots(ijk).count_bin, bots(ijk).member_count) + 1; % plus the bot itself
                    
                    % start standard message processing
                    if (bots(ijk).L>msg_L)||((bots(ijk).L==msg_L)&&(bots(ijk).h>msg_h+1))
                        bots(ijk).L = msg_L;
                        bots(ijk).h = msg_h+1;
                        bots(ijk).m = msg_n;
                        bots(ijk).CH_idx = msg_CH_idx;                      
                        tmp_unique_idx = unique(bots(ijk).CH_idx);
                        [~,~,~,tmp_idx_idx] = mvee_core(current_pos(:,tmp_unique_idx),epsilon);
                        bots(ijk).core_idx = tmp_unique_idx(tmp_idx_idx);                 
                        
                        bots(ijk).message = [ijk bots(ijk).m bots(ijk).L bots(ijk).h bots(ijk).CH_idx']';
                        
                        message_queque = msg_send(bots(ijk).message, bots(ijk).neighbor_idx, message_queque);%send msg to neighbors
                        
                    elseif (bots(ijk).L==msg_L)&&(bots(ijk).h<msg_h)
                        
                        union_idx = union(unique(bots(ijk).CH_idx), unique(msg_CH_idx));                     
                        
                        if numel(union_idx)<3
                            tmp_CH_idx = union_idx;
                        else
                            tmp_CH_idx_idx = convhull(current_pos(1,union_idx),current_pos(2,union_idx));
                            tmp_CH_idx = union_idx(tmp_CH_idx_idx);                     
                        end            
                        
                        bots(ijk).CH_idx = [tmp_CH_idx' tmp_CH_idx(1)*ones(1,N-numel(tmp_CH_idx))];
                        
                        if bots(ijk).m
                            bots(ijk).message = [ijk bots(ijk).m bots(ijk).L bots(ijk).h bots(ijk).CH_idx]';
                            message_queque = msg_send(bots(ijk).message, bots(ijk).m, message_queque);
                        else
                            
                            tmp_unique_idx = unique(bots(ijk).CH_idx);
                            [~,~,~,tmp_idx_idx] = mvee_core(current_pos(:,tmp_unique_idx),epsilon);
                            bots(ijk).core_idx = tmp_unique_idx(tmp_idx_idx);
                        
                        end
                    end
                end
            end
        end
        
        
        num_message_set(ijk_num_bot, ijk_t) = message_count; % 5 x 10 store message from each of the 10 trials
        
        tmp_CH_idx = convhull(current_pos(1,:),current_pos(2,:));
        
        [~,~,~,tmp_idx_idx] = mvee_core(current_pos(:,tmp_CH_idx),epsilon);
        bots(L0).core_idx = tmp_CH_idx(tmp_idx_idx);
        [A_core_mat,~,~,~] = mvee_core(current_pos(:,bots(L0).core_idx),epsilon);  
        num_core_set(ijk_num_bot, ijk_t) = numel(bots(L0).core_idx);
%         [A_core_mat,~] = lowner(current_pos(:,bots(L0).core_idx),epsilon); 
        [A_all_mat,~] = lowner(current_pos,epsilon);
        area_lowner_set(ijk_num_bot, ijk_t) = det(inv(A_all_mat));
        area_core_set(ijk_num_bot, ijk_t) = det(inv(A_core_mat));
        area_rate_set(ijk_num_bot, ijk_t) = abs(area_lowner_set(ijk_num_bot, ijk_t)-area_core_set(ijk_num_bot, ijk_t))/area_lowner_set(ijk_num_bot, ijk_t);
        if (numel(bots(L0).core_idx)>2)&&(numel(bots(L0).core_idx)<6)
            valid_flag_set(ijk_num_bot, ijk_t)=1;
        end
        
        
    end

end
save('darpa_mvee_data2.mat');

end


function adj = GetAdjMatrix(xx, R2)


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


end


function bots = init_bot(bots, adj, L0)
% initialization of bots

N = size(adj,1);

for ijk =1:N
    
    bots(ijk).m = 0;
    bots(ijk).neighbor_idx = find(adj(ijk,:));
    
    bots(ijk).CH_idx = ijk*ones(1,N);
    bots(ijk).CH_idx(1:numel(bots(ijk).neighbor_idx)) = bots(ijk).neighbor_idx; % initialize convex hull robot to be neighbor robots
    bots(ijk).core_idx = [];
%     bots(ijk).count_bin = zeros(1,numel(bots(ijk).neighbor_idx));
%     bots(ijk).member_count = zeros(1,numel(bots(ijk).neighbor_idx));
    bots(ijk).L = abs(ijk-L0);
    bots(ijk).h = 0;
%     bots(ijk).mem_sum = dot(bots(ijk).count_bin, bots(ijk).member_count) + 1;
    bots(ijk).message = [ijk bots(ijk).m bots(ijk).L bots(ijk).h bots(ijk).CH_idx]';
    
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





