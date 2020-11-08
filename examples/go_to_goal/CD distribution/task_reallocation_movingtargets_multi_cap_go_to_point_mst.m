
function [L_vals] = task_reallocation_movingtargets_multi_cap_go_to_point_mst(allocation_flag,res_num)
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

% here we setup the type of robots and the target position
type_flag = zeros(size(ctrl_flag));
type_flag(1:12) = 0;
type_flag(13:40) = 1;

r.set_type(type_flag); % set different types of robots
num_cap = 4;
c = zeros(num_cap,size(type_flag,2)); % as a prototype, set the first ability to be 3 and second to be 2

% set blue robot
c(1,type_flag==0) = 3;
c(2,type_flag==0) = 1;
c(3,type_flag==0) = 2;
c(4,type_flag==0) = 5;

% set red robot
c(1,type_flag==1) = 2;
c(2,type_flag==1) = 5;
c(3,type_flag==1) = 4;
c(4,type_flag==1) = 0;

% set up the tasks add value and requirements here

value_true = [3,5];
require_true = [30,20;
                15,30;
                18,25;
                15,5];

value = [50,50];
require = [1,1;
           0,0;
           0,0;
           0,0];
       

nt = size(value,2); % number of tasks

targets = [1.2, -1.2,  1.2;
    1.2, -1.2, -0.5];

range = 3e-1;
explored = [0,0];
% the control flag is determined by MGM
assigned = zeros(1,N); % store the index of task that the robot is assigned to
[assigned,w] = mgm_no_coalition(c,require,value,assigned,x(1:2,:),targets);
% since the returned assignment start from 1, we substract 1 here
ctrl_flag = assigned-1;
fprintf('ideally, by MGM, the remaining requirement for each task should be [%i,%i] \n', w(1),w(2));
% switch on or off the dynamic task allocation

main_target_pos = ones(2,N); % assign main target for the swarm

target_pos = main_target_pos;
target_pos(:,assigned==0) = x(1:2,assigned==0);
for i = 1:nt
    target_pos(:,assigned==i) = repmat(targets(:,i),[1 numel(find(assigned==i))]); % assign secondary controller
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
font = 15;
% % NOW we need all the legends
% plot(-1.4,1.7, '.b', 'MarkerSize',40)
% text(-1.3,1.7,"12 robots",'FontSize',font);
% text(-1.3,1.6,"each with 3 units of capability 1",'FontSize',font);
% text(-1.3,1.5,"            	   1 units of capability 2",'FontSize',font);
% plot(-1.4,1.35, '.r', 'MarkerSize',40)
% text(-1.3,1.35,"28 robots",'FontSize',font);
% text(-1.3,1.25,"each with 2 units of capability 1",'FontSize',font);
% text(-1.3,1.15,"              	 5 units of capability 2",'FontSize',font);

% NOW we draw all the circles
circle1 = viscircles(targets(:,1)',range,'Color','black');
circle2 = viscircles(targets(:,2)',range,'Color','black');

% Now we add all the text info
tt11 = "value = unknown";
tt12 = "requirement = unknown";
tt13 = "explored = no";

tt21 = "value = unknown";
tt22 = "requirement = unknown";
tt23 = "explored = no";

tt31 = "value = unknown";
tt32 = "requirement = unknown";
tt33 = "explored = no";


h11 = text(targets(1,1)-0.25 ,targets(2,1)+0.55,tt13,'FontSize',font);
h12 = text(targets(1,1)-0.25 ,targets(2,1)+0.45,tt11,'FontSize',font);

h21 = text(targets(1,2)-0.25 ,targets(2,2)+0.55,tt23,'FontSize',font);
h22 = text(targets(1,2)-0.25 ,targets(2,2)+0.45,tt21,'FontSize',font);

L_vals = [];



while(~init_checker(x, initial_conditions))
%     set(gca,'PaperUnits','inches','PaperPosition',[0 0 4 4],'FontSize',20)


    x = r.get_poses();
    
    % first phase, task one moves towards the center
    if (timer_count < 2500)
        targets(:,1) = targets(:,1)-0.0005;
        delete(circle1);
        circle1 = viscircles(targets(:,1)',range,'Color','black');
        delete(h12);
        delete(h11);
        h11 = text(targets(1,1)-0.25 ,targets(2,1)+0.55,tt13,'FontSize',font);
        h12 = text(targets(1,1)-0.25 ,targets(2,1)+0.45,tt11,'FontSize',font);
    end
    
    % second phase, task one moves out
    if (timer_count >3500 && timer_count<4600)
        targets(2,1) = targets(2,1)+0.0005;
        delete(circle1);
        circle1 = viscircles(targets(:,1)',range,'Color','black');
        delete(h12);
        delete(h11);
        h11 = text(targets(1,1)-0.25 ,targets(2,1)+0.55,tt13,'FontSize',font);
        h12 = text(targets(1,1)-0.25 ,targets(2,1)+0.45,tt11,'FontSize',font);
    end
    
    % the third task appears
    if(timer_count==1000)
        w(:,3) = [1;0;0;0];
        require_true(:,3) = [25;20;15;10];
        value_true(3) = 7;
        require(:,3) = [1;0;0;0];
        value(3) = 50;
        explored(3) = 0;
        nt = nt+1;
        
        
        h31 = text(targets(1,3)-0.25 ,targets(2,3)+0.55,tt33,'fontSize',font);
        h32 = text(targets(1,3)-0.25 ,targets(2,3)+0.45,tt31,'FontSize',font);
        
        viscircles(targets(:,3)',range,'Color','black');
    end
    % now we calculate it at every time step
    [assigned,w] = mgm_no_coalition(c,w,value,assigned,x(1:2,:),targets);
    ctrl_flag = assigned - 1;
    target_pos(:,assigned==0) = x(1:2,assigned==0);
    

    
    for i = 1:nt
        target_pos(:,assigned==i) = repmat(targets(:,i),[1 numel(find(assigned==i))]); % assign secondary controller
    end
    current_pos = target_pos;
    
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
    
    [G_conn_graph, G_weights, MST_conn_matrix] = GetGraphWeight(x, 1.3*R2, dxi);
    r.set_conn(MST_conn_matrix);
    
    [remain,remain_no_value,explored,found] = calculate_remain(N,nt,x,targets,require,c,range,assigned,value,explored);
    
   visualize;
   
    if(found~=0)
        w(:,found) = require_true(:,found);
        for kkk = 1:N
            if(assigned(kkk) == found)
                w(:,found) = w(:,found) - c(:,kkk);
            end
        end
        [assigned,w] = mgm_no_coalition(c,w,value,assigned,x(1:2,:),targets);
        target_pos(:,assigned==0) = x(1:2,assigned==0);
        
        for i = 1:nt
            target_pos(:,assigned==i) = repmat(targets(:,i),[1 numel(find(assigned==i))]); % assign secondary controller
        end
        % idk how to do it here
        value(found) = value_true(found);
        require(:,found) = require_true(:,found);
        if(found==1)
            %             delete(h12);
            tt11 = strcat("value = ",num2str(value(1)));
            %             delete(h13);
%             tt12 = sprintf("requirement = %d, %d", require(1,1), require(2,1));
            tt13 = "explored = yes";
            
        elseif (found ==2)
            delete(h21);
            tt23 = "explored = yes";
            h21 = text(targets(1,2)-0.25 ,targets(2,2)+0.55,tt23,'FontSize',font);
            delete(h22);
            tt21 = sprintf("value = %d",value(2));
            h22 = text(targets(1,2)-0.25 ,targets(2,2)+0.45,tt21,'FontSize',font);

        elseif (found == 3)
            delete(h31);
            tt33 = "explored = yes";
            h31 = text(targets(1,3)-0.25 ,targets(2,3)+0.55,tt33,'FontSize',font);
            delete(h32);
            tt31 = sprintf("value = %d",value(3));
            h32 = text(targets(1,3)-0.25 ,targets(2,3)+0.45,tt31,'FontSize',font);

        end
    end
    
%         if(timer_count==2)
%         print(gcf,'initial_config_moving.png','-dpng','-r360');
%     end
%     if(timer_count==750)
%         print(gcf,'fulfill_both_moving.png','-dpng','-r360');
%     end
%     if(found==1)
%         print(gcf,'explore_first_moving.png','-dpng','-r360');
%     end
%     if(found==2)
%         print(gcf,'explore_second_moving.png','-dpng','-r360');
%     end
%     if(found==3)
%         print(gcf,'explore_third_moving.png','-dpng','-r360');
%     end
%     if(timer_count==3500)
%         print(gcf,'final_config_moving.png','-dpng','-r360');
%     end
    
    if (task_alloc)
        si_barrier_certificate = create_weighted_con_barrier_certificate('SafetyRadius', 0.06,'ConnRadius', sqrt(R2),'conn_matrix',MST_conn_matrix);
        
        dxi = si_barrier_certificate(dxi, x(1:2, :),remain);
    else
        si_barrier_certificate = create_con_barrier_certificate('SafetyRadius', 0.06,'ConnRadius', sqrt(R2),'conn_matrix',MST_conn_matrix);
        
        dxi = si_barrier_certificate(dxi, x(1:2, :));
    end

    dxu = si_to_uni_dynamics(dxi, x);

    
    r.set_velocities(1:N, dxu);
    r.step();
    
    
    timer_count = timer_count+1
    dxi_vals{timer_count} = dxi;

end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();

