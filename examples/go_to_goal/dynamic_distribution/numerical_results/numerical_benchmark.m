%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%
%     This is an example call of MIDACO 6.0
%     -------------------------------------
%
%     MIDACO solves Multi-Objective Mixed-Integer Non-Linear Problems:
%
%
%      Minimize     F_1(X),... F_O(X)  where X(1,...N-NI)   is CONTINUOUS
%                                      and   X(N-NI+1,...N) is DISCRETE
%
%      subject to   G_j(X)  =  0   (j=1,...ME)      equality constraints
%                   G_j(X) >=  0   (j=ME+1,...M)  inequality constraints
%
%      and bounds   XL <= X <= XU
%
%
%     The problem statement of this example is given below. You can use 
%     this example as template to run your own problem. To do so: Replace 
%     the objective functions 'F' (and in case the constraints 'G') given 
%     here with your own problem and follow the below instruction steps.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
function [solution,w] =  numerical_benchmark(cap,require,value,nn)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%   MAIN PROGRAM   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 key = 'MIDACO_LIMITED_VERSION___[CREATIVE_COMMONS_BY-NC-ND_LICENSE]';   
 problem.func = @problem_function; % Call is [f,g] = problem_function(x)        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Step 1: Problem definition     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
% STEP 1.A: Problem dimensions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot_types = 2;
nt = 3;

problem.o  = 1; % Number of objectives
problem.n  = 4; % Number of variables (in total)
problem.ni = problem.n; % Number of integer variables (0 <= ni <= n)
problem.m  = 2; % Number of constraints (in total)
problem.me = 0; % Number of equality constraints (0 <= me <= m)
     
% STEP 1.B: Lower and upper bounds 'xl' & 'xu'  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
problem.xl = 0 * ones(1,problem.n);
problem.xu = 1 * ones(1,problem.n); 
problem.xu(1:2) = nn;
problem.xu(3:4) = nn;


% setup parameters
problem.cap = cap;
problem.require = require;
problem.value = value;
problem.nn = nn;

% STEP 1.C: Starting point 'x'  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
problem.x  = problem.xl; % Here for example: 'x' = lower bounds 'xl'
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Step 2: Choose stopping criteria and printing options    %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% STEP 2.A: Stopping criteria 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
option.maxeval  = 10000;    % Maximum number of function evaluation (e.g. 1000000)
option.maxtime  = 60*60*24; % Maximum time limit in Seconds (e.g. 1 Day = 60*60*24)

% STEP 2.B: Printing options  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
option.printeval  = 10000;  % Print-Frequency for current best solution (e.g. 1000)
option.save2file  = 1;     % Save SCREEN and SOLUTION to TXT-files [ 0=NO/ 1=YES]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Step 3: Choose MIDACO parameters (FOR ADVANCED USERS)    %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

option.param( 1) =  0;  % ACCURACY  
option.param( 2) =  0;  % SEED      
option.param( 3) =  0;  % FSTOP
option.param( 4) =  0;  % ALGOSTOP
option.param( 5) =  0;  % EVALSTOP  
option.param( 6) =  0;  % FOCUS
option.param( 7) =  0;  % ANTS
option.param( 8) =  0;  % KERNEL
option.param( 9) =  0;  % ORACLE    
option.param(10) =  0;  % PARETOMAX   
option.param(11) =  0;  % EPSILON   
option.param(12) =  0;  % BALANCE
option.param(13) =  0;  % CHARACTER  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Step 4: Choose Parallelization Factor   %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

option.parallel = 0;  % Serial: 0 or 1, Parallel: 2,3,4,5,6,7,8...  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%   Call MIDACO solver   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[ solution ] = midaco( problem, option, key);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%   End of Example    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x = solution.x;



% Objective functions F(X)
 w = [require(:,1) - x(1)*cap(:,1) - x(3)*cap(:,2),...
     require(:,2) - x(2)*cap(:,1) - x(4)*cap(:,2),...
     require(:,3) - (nn-x(1)-x(2))*cap(:,1) - (nn-x(3)-x(4))*cap(:,2)];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%   OPTIMIZATION PROBLEM   %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ f, g ] = problem_function( x , cap, require, value, nn)

% cap = [3,2;
%        1,5;
%        2,4;
%        5,0];
%    
% % value = [3,5,7];
% require = [30,20,35;
%            45,50,50;
%            28,35,55;
%            15,20,20];
       
% Objective functions F(X)
  f(1) = (value(1)*sum(max(0,require(:,1) - x(1)*cap(:,1) - x(3)*cap(:,2))) ...
       + value(2)*sum(max(0,require(:,2) - x(2)*cap(:,1) - x(4)*cap(:,2))) ...
       + value(3)*sum(max(0,require(:,3) - (nn-x(1)-x(2))*cap(:,1) - (nn-x(3)-x(4))*cap(:,2))));
  

% Equality constraints G(X) = 0 MUST COME FIRST in g(1:me)


% Inequality constraints G(X) >= 0 MUST COME SECOND in g(me+1:m)
  g(1) = nn - x(1) - x(2);
  g(2) = nn - x(3) - x(4);
  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%   END OF FILE   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
