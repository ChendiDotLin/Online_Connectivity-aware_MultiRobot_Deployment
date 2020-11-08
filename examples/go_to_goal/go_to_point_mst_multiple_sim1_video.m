%Initializing the agents to random positions with barrier certificates 
%and data plotting.  This script shows how to initialize robots to a
%particular point
%Paul Glotfelter 
%Modify by Wenhao Luo
%3/24/2016
%9/13/2018

% for sim1 - with 2 biased rendzevous and 2 move-to-goal forming 

% with spanning tree separation


clf;
% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = rb.get_available_agents(); 
N = 40;
num_behavior = 4; % make sure N is dividable by num_behavior
% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
r = rb.set_number_of_agents(N).set_save_data(false).build();

% Initialize x so that we don't run into problems later.  This isn't always
% necessary
x = r.get_poses();
r.step();

% x_offset = [-0.8;-1;0];
% x = x + x_offset;

% load('icra19_sim1_init.mat','x'); % determine the initial positions of robots
% [new_idx,~] = kmeans(x(1:2,:)',num_behavior); % assign membership
load('icra19_sim1_init.mat','x','new_idx');

r.set_poses(x);
adj_flag = false;



% Create a barrier certificate so that the robots don't collide
R2 = 0.05; %0.08;  % 0.25 0.08
timer_switch = 500; % time steps for switching back to primary controller
R2_off = 1.5; % offset scale for determining disconnection: disconnecting when R^2 > R2_off * R2
% L0 = 5; % set the root robot  5
[~,L0] = max(x(1,:)); % find better L0
[conn_matrix, ctrl_flag, adj_mat] = GetConnMatrix_Span(x, R2, L0); % Get Conn Matrix only once
r.set_conn(conn_matrix);
adj_matrix = adj2matrix(adj_mat);
r.set_adj_matrix(adj_matrix);


%% specify control flag

task_vec = [1 -1 1 -0.5 0;...
            1 1 -0.5 -0.3 1]; %[1.2 -1 0.5 0.5;1.2 1 -0.5 1]; %[-1.2 -0.8  1;-1.2 -0.5 -1];

main_target_pos = task_vec(:,1)*ones(1,N); % assign main target for the swarm
target_pos = main_target_pos;

% target_pos(:,ctrl_flag==1) = repmat([-1;1],[1 numel(find(ctrl_flag==1))]); % assign secondary controller
% target_pos(:,ctrl_flag==2) = repmat([0.5;-0.5],[1 numel(find(ctrl_flag==2))]); % assign secondary controller

% pri_pos = [1.2 1.2];
% x_pri = [pri_pos(1)-0.02 pri_pos(1)+0.02 pri_pos(1)+0.02 pri_pos(1)-0.02];
% y_pri = [pri_pos(2)-0.02 pri_pos(2)-0.02 pri_pos(2)+0.02 pri_pos(2)+0.02];
% handle_target_dummy = patch(x_pri,y_pri,'blue');
% pri_pos = [-1 1];
% x_pri = [pri_pos(1)-0.02 pri_pos(1)+0.02 pri_pos(1)+0.02 pri_pos(1)-0.02];
% y_pri = [pri_pos(2)-0.02 pri_pos(2)-0.02 pri_pos(2)+0.02 pri_pos(2)+0.02];
% handle_sec_dummy = patch(x_pri,y_pri,'red');
% pri_pos = [0.5 -0.5];
% x_pri = [pri_pos(1)-0.02 pri_pos(1)+0.02 pri_pos(1)+0.02 pri_pos(1)-0.02];
% y_pri = [pri_pos(2)-0.02 pri_pos(2)-0.02 pri_pos(2)+0.02 pri_pos(2)+0.02];
% handle_third_dummy = patch(x_pri,y_pri,'green');

ctrl_flag = ctrl_flag*0;
member_idx_cell = cell(0,1);
num_member = N/num_behavior;

handle_targets_dummy = cell(0,1);
handle_pri = cell(0,1);
for ijk_h = 1:num_behavior
    cur_flag_idx = find(new_idx==ijk_h);
    ctrl_flag(cur_flag_idx)=ijk_h-1;   % ((ijk_h-1)*num_member+1):(ijk_h*num_member)
    member_idx_cell{ijk_h} = cur_flag_idx; %[((ijk_h-1)*num_member+1):(ijk_h*num_member)];
%     target_pos(:,((ijk_h-1)*num_member+1):(ijk_h*num_member)) = repmat(task_vec(:,ijk_h),[1 numel(member_idx_cell{ijk_h})]); 
    target_pos(:,cur_flag_idx) = repmat(task_vec(:,ijk_h),[1 numel(cur_flag_idx)]); 
    
    pri_pos = task_vec(:,ijk_h);
    x_pri = [pri_pos(1)-0.04 pri_pos(1)+0.04 pri_pos(1)+0.04 pri_pos(1)-0.04];
    y_pri = [pri_pos(2)-0.04 pri_pos(2)-0.04 pri_pos(2)+0.04 pri_pos(2)+0.04];
    
    if ijk_h==2 % added by Wenhao, to change bot color due to ctrl_flag
        color_tar = [1 0 0]; % biased rendzevous
    elseif ijk_h==1
        color_tar = [0 0 1]; % biased rendzevous
    elseif ijk_h==3
        color_tar = [0 1 0]; % circle shape
    elseif ijk_h==4
        color_tar = [1 0 1]; % circle shape
    elseif ijk_h==5
        color_tar = [0 0 0]; % circle shape
    end

    handle_targets_dummy{ijk_h} = patch(x_pri,y_pri,color_tar,'EdgeColor','none','FaceAlpha',0.3);
    font_size = 15;
    handle_pri{ijk_h} = text(pri_pos(1)+0.03,pri_pos(2)+0.08,strcat('Task ',num2str(ijk_h)),'FontSize',font_size);
end



r.set_ctrl(ctrl_flag);

% 
% ctrl_flag(1:floor(N/num_behavior)) = 1;
% ctrl_flag((end-floor(N/num_behavior):end)) = 2;

% member_idx1 = 1:floor(N/num_behavior);
% member_idx0 = (floor(N/num_behavior)+1):(N-floor(N/num_behavior)-1);
% member_idx2 = (N-floor(N/num_behavior):N);

%lap_matrix1 = GetLapMatrix(xx, R2, varargin)

% partition into sub-swarm memberships -- output ctrl_flag.

% implement behavior controller (inputs: robot id, adj matrix, ctrl_vec, varargin (control parameters);  output: ctrl_vec)

%% 


si_barrier_certificate = create_con_barrier_certificate('SafetyRadius', 0.06,'ConnRadius', sqrt(R2),'conn_matrix',conn_matrix);
si_to_uni_dynamics = create_si_to_uni_mapping2();
        
%Get randomized initial conditions in the robotarium arena
initial_conditions = generate_initial_conditions(N, 'Width', r.boundaries(2)-r.boundaries(1)-0.1, 'Height', r.boundaries(4)-r.boundaries(3)-0.1, 'Spacing', 0.2);

% We'll make the rotation error huge so that the initialization checker
% doesn't care about it
args = {'PositionError', 0.01, 'RotationError', 50};
init_checker = create_is_initialized(args{:});

controller_rend = create_si_rendzvous_controller();
controller_move = create_si_position_controller();

timer_count = 1; % set timer for switch motion later
current_pos = target_pos;

%% prepare for computing the performance-related metrics
min_bot_dist_hist = zeros(1500,1);
conn_bot_hist = zeros(1500,1);
pertub_bot_hist = zeros(1500,1);
perf_bot_hist = zeros(1500,1);

% controller = create_si_rendzvous_controller();
r.set_adj_flag(true); % adj_flag, when debug to plot sim1, turn it to true
while (timer_count<1501)%(~init_checker(x, initial_conditions))

    x = r.get_poses();
    
    handle_timestep = text(-0.3,-1.4,strcat('Time Step = ',num2str(timer_count)),'FontSize',20);

    
    bot_dist_tmp = pdist2(x(1:2,:)', x(1:2,:)');
    bot_dist_tmp = triu(bot_dist_tmp, 1);
    min_bot_dist_tmp = min(bot_dist_tmp(find(bot_dist_tmp(:)>0)));
    
    min_bot_dist_hist(timer_count) = min_bot_dist_tmp;

    conn_matrix_tmp = GetConnMatrix(x, 1.5*R2); % simply test whether the connectivity constraint is violated
    
%   dxi = controller(x(1:2, :), initial_conditions(1:2, :));
    if false %timer_count>timer_switch + 1e4
        current_pos = main_target_pos;
        ctrl_flag = ctrl_flag*0;
        r.set_ctrl(ctrl_flag);        
    end
   % implement behavior controller (inputs: robot id, adj matrix, ctrl_vec, varargin (control parameters);  output: ctrl_vec)
     dxi = zeros(2,N); 
     radius_circle = 0.3;
   
     des_pos = zeros(2,N);
     
   for ijk_h = 1:num_behavior
       if ijk_h<3 % biased rendzevous
           lap_matrix = GetLapMatrix(x(:,member_idx_cell{ijk_h}), 1.8*R2);
           dxi_tmp = controller_rend(lap_matrix, x(1:2, member_idx_cell{ijk_h}), current_pos(:,member_idx_cell{ijk_h}));
           dxi(:,member_idx_cell{ijk_h}) = dxi_tmp;
           des_pos(:,member_idx_cell{ijk_h}) = current_pos(:,member_idx_cell{ijk_h});
       else
           Nm = numel(member_idx_cell{ijk_h});
           theta_vec = 1/Nm*2*pi*[1:Nm];
           x_poses = radius_circle*cos(theta_vec);
           y_poses = radius_circle*sin(theta_vec);
           poses = [x_poses;y_poses];
           dxi_tmp = controller_move(x(1:2, member_idx_cell{ijk_h}), current_pos(:,member_idx_cell{ijk_h})+poses);
           dxi(:,member_idx_cell{ijk_h}) = dxi_tmp;
           des_pos(:,member_idx_cell{ijk_h}) = current_pos(:,member_idx_cell{ijk_h})+poses;
       end
       
       
   end
   
   perf_bot_tmp = sum(sqrt(sum((x(1:2,:)-des_pos).^2, 1)))/N;
    
    perf_bot_hist(timer_count) = perf_bot_tmp;
   

%     lap_matrix0 = GetLapMatrix(x(:,member_idx0), 1.5*R2);
%     lap_matrix1 = GetLapMatrix(x(:,member_idx1), 1.5*R2);
%     lap_matrix2 = GetLapMatrix(x(:,member_idx2), 1.5*R2);
    
%     
    
%     dxi = controller(x(1:2, :), current_pos);
    
    [G_conn_graph, G_weights, MST_conn_matrix, adj_tmp] = GetGraphWeight(x, 1.2*R2, dxi, ctrl_flag); 
    r.set_conn(MST_conn_matrix);
    
    adj_matrix = adj2matrix(adj_tmp);
    r.set_adj_matrix(adj_matrix);
    
    
    lap_matrix_tmp = diag(sum(adj_tmp,2))-adj_tmp;
    eig_vec_tmp = eig(lap_matrix_tmp);
    eig_tmp = eig_vec_tmp(2);
    
    conn_bot_hist(timer_count) = eig_tmp;
    
    si_barrier_certificate = create_con_barrier_certificate('SafetyRadius', 0.06,'ConnRadius', sqrt(R2),'conn_matrix',MST_conn_matrix);
    
    dxi_opt = dxi;
    
    dxi = si_barrier_certificate(dxi, x(1:2, :)); 
  
    delta_dxi = dxi-dxi_opt;
    
    pertub_bot_tmp = sum(sum(delta_dxi.^2, 1))/N;
    pertub_bot_hist(timer_count) = pertub_bot_tmp;

    dxu = si_to_uni_dynamics(dxi, x);

    r.set_velocities(1:N, dxu);
    if timer_count==2
       
       r.set_adj_flag(false);
       r.step();
      
    else
        r.set_adj_flag(false);
        r.step();
    end
    
    timer_count = timer_count+1
    
    delete(handle_timestep);
    
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();

