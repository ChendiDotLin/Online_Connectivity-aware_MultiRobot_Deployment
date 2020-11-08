%Initializing the agents to random positions with barrier certificates 
%and data plotting.  This script shows how to initialize robots to a
%particular point
%Paul Glotfelter 
%Modify by Wenhao Luo
%3/24/2016
%7/09/2018

% with spanning tree separation
% moving secondary target (back and forth motion)
% with rendzvous controller

figure(99);
clf;

% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = rb.get_available_agents(); 
N = 40;

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
r = rb.set_number_of_agents(N).set_save_data(false).build();

% Initialize x so that we don't run into problems later.  This isn't always
% necessary
x = r.get_poses(); % default initialization by Robotarium
% initialize to left bottom arena (customized by Wenhao)
x_offset = [-0.8;-1;0];
x = x + x_offset;
load('init_poses1.mat','x');
r.set_poses(x);


r.step();

% Create a barrier certificate so that the robots don't collide
R2 = 0.1;  % 0.25 0.08 0.04    0.1
pri_pos = [0.5;1]; % primary rendzvous target location  1.2  1.5
sec_pos = [-0.6;0.4]; % secondary rendzvous target location
sec_pos0 = sec_pos;
timer_switch_sec = 300; % time steps for switching to secondary
timer_switch_pri = 1100; % time steps for switching back to primary controller
R2_off = 1.5; % offset scale for determining disconnection: disconnecting when R^2 > R2_off * R2
init_ctrl_flag = zeros(1,N);
L0 = 20; % set the root robot 20
% [~,L0] = max(x(1,:)); % find better L0
[conn_matrix, ctrl_flag] = GetConnMatrix_Span(x, R2, L0, 0); % Get Conn Matrix only once
r.set_conn(conn_matrix);
% r.set_ctrl(ctrl_flag);
main_target_pos = pri_pos.*ones(2,N); % assign main target for the swarm
target_pos = main_target_pos;
target_pos(:,ctrl_flag>0) = repmat(sec_pos,[1 numel(find(ctrl_flag>0))]); % assign secondary controller

si_barrier_certificate = create_con_barrier_certificate('SafetyRadius', 0.03,'ConnRadius', sqrt(R2),'conn_matrix',conn_matrix);
si_to_uni_dynamics = create_si_to_uni_mapping2();
        
%Get randomized initial conditions in the robotarium arena
initial_conditions = generate_initial_conditions(N, 'Width', r.boundaries(2)-r.boundaries(1)-0.1, 'Height', r.boundaries(4)-r.boundaries(3)-0.1, 'Spacing', 0.2);

% We'll make the rotation error huge so that the initialization checker
% doesn't care about it
args = {'PositionError', 0.01, 'RotationError', 50};
init_checker = create_is_initialized(args{:});
controller = create_si_rendzvous_controller();

timer_count = 0; % set timer for switch motion later
% current_pos = target_pos;
current_pos = main_target_pos;
r.set_ctrl(init_ctrl_flag);

figure(99);
font_size = 10;
handle_pri = text(pri_pos(1)-0.4,pri_pos(2)+0.25,'primary target','FontSize',font_size);
handle_exp = text(-0.5,-1.3,'swarm (blue) move to primary target','FontSize',font_size);
n_step = 60; % steps from 0 to pi/2 for the periodical sec target motion
handle_sec = [];
handle_sec_dummy = [];

x_pri = [pri_pos(1)-0.02 pri_pos(1)+0.02 pri_pos(1)+0.02 pri_pos(1)-0.02];
y_pri = [pri_pos(2)-0.02 pri_pos(2)-0.02 pri_pos(2)+0.02 pri_pos(2)+0.02];
handle_pri_dummy = patch(x_pri,y_pri,'red');

while timer_count < 2000 %(~init_checker(x, initial_conditions))

    x = r.get_poses();
    sec_pos = sec_pos0 + [0.05*sin(timer_count*pi/2/n_step);0.14*sin(timer_count*pi/2/n_step)]; % 0.05 0.14
    target_pos(:,ctrl_flag>0) = repmat(sec_pos,[1 numel(find(ctrl_flag>0))]); % assign secondary controller
    
    conn_matrix_tmp = GetConnMatrix(x, R2_off*R2); % simply test whether the connectivity constraint is violated
    
    %   dxi = controller(x(1:2, :), initial_conditions(1:2, :));
    if (timer_count>=timer_switch_sec)&&(timer_count<timer_switch_pri) %timer_count==timer_switch_sec
        delete(handle_sec);
        delete(handle_sec_dummy);
        handle_sec = text(sec_pos(1),sec_pos(2)+0.08,'secondary target','FontSize',font_size);
        x_sec = [sec_pos(1)-0.02 sec_pos(1)+0.02 sec_pos(1)+0.02 sec_pos(1)-0.02];
        y_sec = [sec_pos(2)-0.02 sec_pos(2)-0.02 sec_pos(2)+0.02 sec_pos(2)+0.02];
        handle_sec_dummy = patch(x_sec,y_sec,'green');
        delete(handle_exp);
        handle_exp = text(-1,-1.3,'sub-swarm (red) break off and move to secondary target','FontSize',font_size);        
        current_pos = target_pos;
        r.set_ctrl(ctrl_flag);
    elseif timer_count==timer_switch_pri
        delete(handle_sec);
        delete(handle_sec_dummy);
        delete(handle_exp);
        handle_exp = text(-1,-1.3,'sub-swarm merge back and move to primary target','FontSize',font_size);   
        current_pos = main_target_pos;
        r.set_ctrl(init_ctrl_flag);
    end
    
    lap_matrix = GetLapMatrix(x, R2);
    
    dxi = controller(lap_matrix, x(1:2, :), current_pos);
    
    [G_conn_graph, G_weights, MST_conn_matrix] = GetGraphWeight(x, 1.5*R2, dxi); 
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

