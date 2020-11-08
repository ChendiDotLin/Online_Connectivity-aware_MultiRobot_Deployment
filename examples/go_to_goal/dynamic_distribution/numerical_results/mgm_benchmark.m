function [assigned,w] = mgm_benchmark(cap,w,value,assigned)

% Written by Chendi

% cap ~ l*k. each row is a different capability, each column is a robot
% w is the remaining needed coverage, ~ l*j. each row is a different
% capability, each column is a task

% return
% assigned is a list, containing the id of each task, starting from 1
% w is the remaining after solving the problem

% transform the assigned array and capability into right form
num_robots = size(cap,2);
% assigned = zeros(1,num_robots); % store the index of task that the robot is assigned to

num_cap = size(cap,1);

nt = size(w,2);
alpha = 1;

for robots = 1:num_robots
    im_vals = -inf*ones(1,num_robots);
    im_jvals = zeros(1,num_robots);
    for k = 1:num_robots
        improved =[];
        if(assigned(k)==0) % if not assigned, calculate the improvement as this
            loss = 0;
        else
            task_idx = assigned(k);
            loss= sum(max(0,(min(w(:,task_idx) + cap(:,k),cap(:,k)))*value(task_idx)));
            % this line is problematic
            
            
%             loss= sum(min(w(:,task_idx) + cap(:,k),cap(:,k)))*value(task_idx);
%             if (loss<0)
%                 loss = loss/2;
%             end
        end
        for j = 1:nt+1
            %             if(assigned(k)~=0)
            %                 improved(j) = min(cap(k),w(j))-
            % the improvement is determined by the assigned robots now
            if(j ==nt+1)
                
                improved(j) = alpha;
            else

            
            improved(j) = sum(min(cap(:,k),max(zeros(num_cap,1),w(:,j))))*value(j) - loss; % how much a robot can fulfill
            end
            % if improved = 0, we can make it a negative value
            % inversely proportional to task value
            
        end
        
        % for each robot, which task gets the most improvement
        [max_improved,chosen_j] = max(improved);
        im_vals(k) = max_improved;
        im_jvals(k) = chosen_j;
        
        
        
    end
    [max_vals,idx] = max(im_vals); % idx is the robot index
    % if max <= 0 and not assigned, randomly assign
    % if max <= 0 and assigned, dont bother
    %     if(max_vals <= 0)
    %         if(assigned(idx)==0)
    %             taskj = randi(nt);
    %             w(taskj) = w(taskj) - cap(idx);
    %             assigned(idx) = taskj;
    %         end
    %     else
    if(max_vals <= alpha)
        % if all the requirements are fulfilled, assign the task randomly
        % based on the importance of the task
        [assigned,w] = generate_random_idx(assigned,value,w,cap);
        break;
        
    else
        taskj = im_jvals(idx);
        % this is wrong! you need to delete from the original one!!
        % everything is wrong here i feel
        if (assigned(idx)~=0)
        w(:,assigned(idx)) = w(:,assigned(idx)) +cap(:,idx);
        end
        w(:,taskj) = w(:,taskj) - cap(:,idx);
        
        assigned(idx) = taskj;
    end
    %     end
    
end


end

function[assigned,w] = generate_random_idx(assigned,v,w,cap)
P = cumsum(v)/sum(v);
for i = 1:length(assigned)
    if(assigned(i) == 0)
        taskid = find(rand<(P),1,'first');
        assigned(i) = taskid;
        w(:,taskid) = w(:,taskid) - cap(:,i);
    end
end
end