cnt = 1;
times = [];
losses = [];
percents = [];
Ns = [];
for nn = 1:40
N = nn*2;
nt = 3;
% here we setup the type of robots and the target position
type_flag = zeros(1,N);
type_flag(1:nn) = 0;
type_flag(nn+1:N) = 1;

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

% value = [3,5,7];
% require = [30,20,35;
%            45,50,50;
%            28,35,55;
%            15,20,20];


cap = [3,2;
    1,5;
    2,4;
    5,0];

time_vals = [0;0;0];
loss_vals = [0;0;0];
percent_vals = [0;0;0];
num_tests = 200;
for i = 1:num_tests
    value = randi([1 10],1, nt);
    require = randi([10 50],num_cap,nt);
    assigned = zeros(1,N); % store the index of task that the robot is assigned to
    
    robot_types = [nn,nn];
    tic;
    [assigned,w1] = mgm_benchmark(c,require,value,assigned);
    t1 = toc;
    mgm_loss = sum(sum(max(0,w1).*value));
    mgm_percent = sum(sum(max(0,w1)./require));
    
    tic;
    [solution,w2] = numerical_benchmark(cap,require,value,nn);
    t2 = toc;
    midaco_loss = solution.f;
    midaco_percent = sum(sum(max(0,w2)./require));

    tic;
    [x,fval,w3] = ga_benchmark(cap,require,value,nn);
    t3 = toc;
    ga_loss = fval;
    ga_percent = sum(sum(max(0,w3)./require));

    time_vals = time_vals + [t1;t2;t3];
    loss_vals = loss_vals + [mgm_loss;midaco_loss;ga_loss];
    percent_vals = percent_vals + [mgm_percent; midaco_percent; ga_percent];
end
times(:,cnt) = time_vals/num_tests;
losses(:,cnt) = loss_vals/num_tests;
percents(:,cnt) = percent_vals/num_tests;
Ns(cnt) = N;
save('numerical_res.mat','times','losses','percents','Ns');
cnt = cnt+1;
end