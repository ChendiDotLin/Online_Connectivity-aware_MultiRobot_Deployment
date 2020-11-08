
function [L_vals] = task_reallocation_2targets_go_to_point_mst(allocation_flag)
close all;
%Initializing the agents to random positions with barrier certificates
%and data plotting.  This script shows how to initialize robots to a
%particular point
%Paul Glotfelter
%Modify by Wenhao Luo
%Further modified by Chendi Lin
%3/24/2016
%9/4/2018
%10/7/2019

% with spanning tree separation

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
x = r.get_poses();

r.step();

% Create a barrier certificate so that the robots don't collide
R2 = 0.08;  % 0.25 0.08
timer_switch = Inf; % time steps for switching back to primary controller, never switch back here
R2_off = 1.5; % offset scale for determining disconnection: disconnecting when R^2 > R2_off * R2
% L0 = 5; % set the root robot  5
[~,L0] = max(x(1,:)); % find better L0
[conn_matrix, ctrl_flag] = GetConnMatrix_Span(x, R2, L0); % Get Conn Matrix only once


r.set_conn(conn_matrix);
r.set_ctrl(ctrl_flag);

initialize;


% the control flag is determined by MGM
assigned = zeros(1,N); % store the index of task that the robot is assigned to
[assigned,w] = mgm_no_coalition(c,require,value,assigned,x(1:2,:),targets);
% since the returned assignment start from 1, we substract 1 here
ctrl_flag = assigned-1;
fprintf('ideally, by MGM, the remaining requirement for each task should be [%i,%i] \n', w(1),w(2));
% switch on or off the dynamic task allocation

main_target_pos = ones(2,N); % assign main target for the swarm

target_pos = main_target_pos;
%  target_pos(:,ctrl_flag==-1) = x(1:2,ctrl_flag==-1);
for i = 0:nt-1
    target_pos(:,ctrl_flag==i) = repmat(targets(:,i+1),[1 numel(find(ctrl_flag==i))]); % assign secondary controller
end

task_alloc = allocation_flag;


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

% NOW we need all the legends
plot(-1.4,1.7, '.b', 'MarkerSize',40)
text(-1.3,1.7,"robots with 3 units of capabilities",'FontSize',11);
plot(-1.4,1.55, '.r', 'MarkerSize',40)
text(-1.3,1.55,"robots with 2 units of capabilities",'FontSize',11);

% NOW we draw all the circles
viscircles(targets(:,1)',range,'Color','black');
viscircles(targets(:,2)',range,'Color','black');

h1 = text(0.5,1.75,"value = unknown",'FontSize',11);
h2 = text(-1.4,-0.6,"value = unknown",'FontSize',11);
h4 = text(0.5,1.65,"requirement = unknown",'FontSize',11);
h5 = text(-1.4,-0.7,"requiremen = unknown",'FontSize',11);
L_vals = [];

while(~init_checker(x, initial_conditions))
    
    x = r.get_poses();
    if(timer_count==1000)
        w(3) = 1;
        w_true(3) = 10;
        require_true(3) = 10;
        value_true(3) = 2;
        require(3) = 1;
        value(3) = 100;
        explored(3) = 0;
                nt = nt+1;

        [assigned,w] = mgm_no_coalition(c,w,value,assigned,x(1:2,:),targets);
        ctrl_flag = assigned - 1;
        for i = 0:nt-1
            target_pos(:,ctrl_flag==i) = repmat(targets(:,i+1),[1 numel(find(ctrl_flag==i))]); % assign secondary controller
        end
        current_pos = target_pos;
        
        h3 = text(0.5,0.0,"value = unknown",'FontSize',11);
        h6 = text(0.5,-0.1,"requirement = unknown",'FontSize',11);
        
        viscircles(targets(:,3)',range,'Color','black');
    end
    conn_matrix_tmp = GetConnMatrix(x, 1.5*R2); % simply test whether the connectivity constraint is violated
    
%     [assigned,w] = mgm_no_coalition(c,w,value,assigned,x(1:2,:),targets);
%         ctrl_flag = assigned - 1;
%         for i = 0:nt-1
%             target_pos(:,ctrl_flag==i) = repmat(targets(:,i+1),[1 numel(find(ctrl_flag==i))]); % assign secondary controller
%         end
%         current_pos = target_pos;
    
    %   dxi = controller(x(1:2, :), initial_conditions(1:2, :));
    if timer_count>timer_switch
        current_pos = main_target_pos;
    end
    
    dxi = controller(x(1:2, :), current_pos);
    
    [G_conn_graph, G_weights, MST_conn_matrix] = GetGraphWeight(x, 1.2*R2, dxi);
    r.set_conn(MST_conn_matrix);
    
    [remain,remain_no_value,A3,explored,found] = calculate_remain(N,nt,x,targets,require,c,range,ctrl_flag,value,explored);
    if(found~=0)
        w(found) = require_true(found);
        for kkk = 1:N
            if(assigned(kkk) == found)
                w(found) = w(found) - c(kkk);
            end
        end
        [assigned,w] = mgm_no_coalition(c,w,value,assigned,x(1:2,:),targets);
        ctrl_flag = assigned - 1;
        for i = 0:nt-1
            target_pos(:,ctrl_flag==i) = repmat(targets(:,i+1),[1 numel(find(ctrl_flag==i))]); % assign secondary controller
        end
        current_pos = target_pos;
        % idk how to do it here
        value(found) = value_true(found);
        require(found) = require_true(found);
        if(found==1)
            delete(h1);
            h1 = text(0.5,1.75,strcat("value = ",num2str(value(1))),'FontSize',11);
            delete(h4);
            h4 = text(0.5,1.65,strcat("requirement = ",num2str(require(1))),'FontSize',11);
            
        elseif (found ==2)
            delete(h2);
            h2 = text(-1.4,-0.6,strcat("value = ",num2str(value(2))),'FontSize',11);
            delete(h5);
            h5 = text(-1.4,-0.7,strcat("requirement = ",num2str(require(2))),'FontSize',11);
        elseif (found == 3)
            delete(h3);
            h3 = text(0.5,0.0,strcat("value = ",num2str(value(3))),'FontSize',11);
            delete(h6);
            h6 = text(0.5,-0.1,strcat("requirement = ",num2str(require(3))),'FontSize',11);
        end
    end
    L = sum(remain_no_value.*value);
    
    try
        delete(h7);
        delete(h8);
        delete(h9);
    catch
    end
    h7 = text(0.5,1.55,strcat("remain = ",num2str(remain_no_value(1))),'FontSize',11);
    h8 = text(-1.4,-0.8,strcat("remain = ",num2str(remain_no_value(2))),'FontSize',11);
    
    
    if(nt == 3)
        
        h9 = text(0.5,-0.2,strcat("remain = ",num2str(remain_no_value(3))),'FontSize',11);
    end
    
    if (task_alloc)
        si_barrier_certificate = create_weighted_con_barrier_certificate('SafetyRadius', 0.06,'ConnRadius', sqrt(R2),'conn_matrix',MST_conn_matrix);
        
        dxi = si_barrier_certificate(dxi, x(1:2, :),remain,A3);
    else
        si_barrier_certificate = create_con_barrier_certificate('SafetyRadius', 0.06,'ConnRadius', sqrt(R2),'conn_matrix',MST_conn_matrix);
        
        dxi = si_barrier_certificate(dxi, x(1:2, :));
    end
    aaa = A3*reshape(dxi,2*N,1);
    %     if (aaa>1e-10)
    %         aaa
    % %         pause(1);
    %     end
    dxu = si_to_uni_dynamics(dxi, x);
    
    
    r.set_velocities(1:N, dxu);
    r.step();
    timer_count = timer_count+1
    L_vals(timer_count) = L;
    if (timer_count>=10000)
        break;
    end
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();

