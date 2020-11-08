% c is the capability list, showing the capability of each group i
% each row stands for different capabilities, e.g. coverage, mobility

% r is the number of robots of each group i
% w is the required capability for each task j
% each row stands for different capability requirement


% here we setup the type of robots and the target position
type_flag = zeros(size(ctrl_flag));
type_flag(1:12) = 0;
type_flag(13:40) = 1;

r.set_type(type_flag); % set different types of robots
c = zeros(size(type_flag)); % as a prototype, set the first ability to be 3 and second to be 2
c(:,type_flag==0) = 3;
c(:,type_flag==1) = 2;


% set up the tasks add value and requirements here

value_true = [5,3];
require_true = [30,20];

value = [100,100];
require = [1,1];
nt = size(value,2); % number of tasks

targets = [1.2, -1.2,  1.2;
    1.2, -1.2, -0.5];

range = 3e-1;
explored = [0,0];
