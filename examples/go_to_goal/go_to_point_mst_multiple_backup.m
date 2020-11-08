%Initializing the agents to random positions with barrier certificates 
%and data plotting.  This script shows how to initialize robots to a
%particular point
%Paul Glotfelter 
%Modify by Wenhao Luo
%3/24/2016
%9/4/2018

% with spanning tree separation

% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = rb.get_available_agents(); 
N = 80;

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
r = rb.set_number_of_agents(N).set_save_data(false).build();

% Initialize x so that we don't run into problems later.  This isn't always
% necessary
x = r.get_poses();
r.step();

% Create a barrier certificate so that the robots don't collide
R2 = 0.08;  % 0.25 0.08
timer_switch = 500; % time steps for switching back to primary controller
R2_off = 1.5; % offset scale for determining disconnection: disconnecting when R^2 > R2_off * R2
% L0 = 5; % set the root robot  5
[~,L0] = max(x(1,:)); % find better L0
[conn_matrix, ctrl_flag] = GetConnMatrix_Span(x, R2, L0); % Get Conn Matrix only once
r.set_conn(conn_matrix);

%% specify control flag
num_behavior = 3;
task_vec = [-1.2 -0.8  1;-1.2 -0.5 -1];

ctrl_flag(1:floor(N/num_behavior)) = 1;
ctrl_flag((end-floor(N/num_behavior):end)) = 2;

%lap_matrix1 = GetLapMatrix(xx, R2, varargin)

% partition into sub-swarm memberships -- output ctrl_flag.

% implement behavior controller (inputs: robot id, adj matrix, ctrl_vec, varargin (control parameters);  output: ctrl_vec)



%% 

r.set_ctrl(ctrl_flag);
main_target_pos = 1.2*ones(2,N); % assign main target for the swarm
target_pos = main_target_pos;

target_pos(:,ctrl_flag==1) = repmat([-1.2;1.4],[1 numel(find(ctrl_flag==1))]); % assign secondary controller
target_pos(:,ctrl_flag==2) = repmat([0.8;-1.4],[1 numel(find(ctrl_flag==2))]); % assign secondary controller

pri_pos = [1.2 1.2];
x_pri = [pri_pos(1)-0.02 pri_pos(1)+0.02 pri_pos(1)+0.02 pri_pos(1)-0.02];
y_pri = [pri_pos(2)-0.02 pri_pos(2)-0.02 pri_pos(2)+0.02 pri_pos(2)+0.02];
handle_target_dummy = patch(x_pri,y_pri,'blue');
pri_pos = [-1.2 1.4];
x_pri = [pri_pos(1)-0.02 pri_pos(1)+0.02 pri_pos(1)+0.02 pri_pos(1)-0.02];
y_pri = [pri_pos(2)-0.02 pri_pos(2)-0.02 pri_pos(2)+0.02 pri_pos(2)+0.02];
handle_sec_dummy = patch(x_pri,y_pri,'red');
pri_pos = [0.8 -1.4];
x_pri = [pri_pos(1)-0.02 pri_pos(1)+0.02 pri_pos(1)+0.02 pri_pos(1)-0.02];
y_pri = [pri_pos(2)-0.02 pri_pos(2)-0.02 pri_pos(2)+0.02 pri_pos(2)+0.02];
handle_third_dummy = patch(x_pri,y_pri,'green');


si_barrier_certificate = create_con_barrier_certificate('SafetyRadius', 0.06,'ConnRadius', sqrt(R2),'conn_matrix',conn_matrix);
si_to_uni_dynamics = create_si_to_uni_mapping2();
        
%Get randomized initial conditions in the robotarium arena
initial_conditions = generate_initial_conditions(N, 'Width', r.boundaries(2)-r.boundaries(1)-0.1, 'Height', r.boundaries(4)-r.boundaries(3)-0.1, 'Spacing', 0.2);

% We'll make the rotation error huge so that the initialization checker
% doesn't care about it
args = {'PositionError', 0.01, 'RotationError', 50};
init_checker = create_is_initialized(args{:});
controller = create_si_position_controller();

timer_count = 0; % set timer for switch motion later
current_pos = target_pos;
while(~init_checker(x, initial_conditions))

    x = r.get_poses();
    
    conn_matrix_tmp = GetConnMatrix(x, 1.5*R2); % simply test whether the connectivity constraint is violated
    
%   dxi = controller(x(1:2, :), initial_conditions(1:2, :));
    if timer_count>timer_switch
        current_pos = main_target_pos;
        ctrl_flag = ctrl_flag*0;
        r.set_ctrl(ctrl_flag);        
    end
    
    dxi = controller(x(1:2, :), current_pos);
    
    [G_conn_graph, G_weights, MST_conn_matrix] = GetGraphWeight(x, 1.2*R2, dxi, ctrl_flag); 
    r.set_conn(MST_conn_matrix);
    si_barrier_certificate = create_con_barrier_certificate('SafetyRadius', 0.06,'ConnRadius', sqrt(R2),'conn_matrix',MST_conn_matrix);
    
    dxi = si_barrier_certificate(dxi, x(1:2, :));      
    dxu = si_to_uni_dynamics(dxi, x);

    
    r.set_velocities(1:N, dxu);
    r.step();   
    
    timer_count = timer_count+1
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();

