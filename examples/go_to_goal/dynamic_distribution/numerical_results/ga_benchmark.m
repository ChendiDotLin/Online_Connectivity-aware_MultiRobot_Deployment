function [x,fval,w] =  ga_benchmark(cap,require,value,nn)

robot_types = 2;
nt = 3;

nvars = robot_types*nt;


lb = 0 * ones(1,nvars);
ub = 1 * ones(1,nvars); 
ub(1:3) = nn;
ub(4:6) = nn;

% can make it 4 variables tomorrow.
A = [1,1,1,0,0,0;0,0,0,1,1,1];
b = [nn;nn];

% opts = optimoptions('ga','PlotFcn',@gaplotbestf);
opts = optimoptions('ga','MaxStallGenerations',50,'FunctionTolerance',1e-10,...
    'MaxGenerations',3000,'PlotFcn',@gaplotbestfun);
[x,fval,exitflag] = ga({@problem_function,cap,require,value},nvars,[A;-1*A],[b;-1*b],[],[],...
    lb,ub,[],[1,2,3,4,5,6],opts);


% Objective functions F(X)
 w = [require(:,1) - x(1)*cap(:,1) - x(4)*cap(:,2),...
     require(:,2) - x(2)*cap(:,1) - x(5)*cap(:,2),...
     require(:,3) - x(3)*cap(:,1) - x(6)*cap(:,2)];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%   OPTIMIZATION PROBLEM   %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ f] = problem_function( x , cap, require, value)

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
  f = (value(1)*sum(max(0,require(:,1) - x(1)*cap(:,1) - x(4)*cap(:,2))) ...
       + value(2)*sum(max(0,require(:,2) - x(2)*cap(:,1) - x(5)*cap(:,2))) ...
        + value(3)*sum(max(0,require(:,3) - x(3)*cap(:,1) - x(6)*cap(:,2))));
  

  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%   END OF FILE   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
