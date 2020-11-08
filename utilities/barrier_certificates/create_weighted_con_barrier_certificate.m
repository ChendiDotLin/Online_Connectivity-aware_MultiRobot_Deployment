
%% create_con_barrier_certificate
% Returns a single-integrator barrier certificate function ($f :
% \mathbf{R}^{2 \times N} \times \mathbf{R}^{2 \times N} \to \mathbf{R}^{2
% \times N}$).  This function takes a 2 x N, 2 x N single-integrator
% velocity and state vector, respectively, and returns a single-integrator
% velocity vector that does not induce collisions in the agents while
% maintaining connectivity.
%% Detailed Description 
%%
% * BarrierGain - affects how quickly the agents can approach each other 
% * SafetyRadius - affects the distance the agents maintain 
%% 
% A good rule of thumb is to make SafetyRadius a bit larger than the agent
% itself (0.08 m for the GRITSbot).

%% Implementation
function [ con_barrier_certificate ] = create_con_barrier_certificate(varargin)
        
    parser = inputParser;
    parser.addParameter('BarrierGain', 1e4);
    parser.addParameter('SafetyRadius', 0.1);
    parser.addParameter('ConnRadius', 0.3); % to be tuned
    parser.addParameter('conn_matrix', []); % to be tuned,  M x 2 matrix, where M is number of edges
    parser.addParameter('max_velocity', 120); % to be tuned
    parser.addParameter('admm_flag', 0); % whether to use admm
    
    parse(parser, varargin{:})
    opts = optimoptions(@quadprog,'Display','off');

    gamma = parser.Results.BarrierGain;
    safety_radius = parser.Results.SafetyRadius;
    conn_radius = parser.Results.ConnRadius;
    conn_matrix = parser.Results.conn_matrix;
    max_velocity = parser.Results.max_velocity/sqrt(2); % enforce strict velocity constraint at each axis
    admm_flag = parser.Results.admm_flag;
    
    con_barrier_certificate = @barrier_certificate_f;

    function [ dx ] = barrier_certificate_f(dxi, x, remain)
        %BARRIERCERTIFICATE Wraps single-integrator dynamics in safety barrier
        %certificates
        %   This function accepts single-integrator dynamics and wraps them in
        %   barrier certificates to ensure that collisions and disconnections do not occur.  Note that
        %   this algorithm bounds the magnitude of the generated output to 0.1.
        %
        %   dx = BARRIERCERTIFICATE(dxi, x, safetyRadius)
        %   dx: generated safe, single-integrator inputs
        %   dxi: single-integrator synamics
        %   x: States of the agents
        %   safetyRadius:  Size of the agents (or desired separation distance)       
        %   connRadius: Connectivity radius
        
        N = size(dxi, 2);
        
        if(N < 2)
           dx = dxi;
           return 
        end
        
        x = x(1:2, :);
        
        %Generate constraints for barrier certificates based on the size of
        %the safety radius
        num_constraints = nchoosek(N, 2);
        num_conn_constraints = size(conn_matrix,1);
        A = zeros(num_constraints, 2*N); % for safety
        A2 = zeros(num_conn_constraints, 2*N); % for connectivity
        b = zeros(num_constraints, 1);  % for safety 
        b2 = zeros(num_conn_constraints, 1);  % for connectivity
        count = 1;
        for i = 1:(N-1)
            for j = (i+1):N
                h = norm(x(1:2,i)-x(1:2,j))^2-safety_radius^2;
                A(count, (2*i-1):(2*i)) = -2*(x(:,i)-x(:,j));
                A(count, (2*j-1):(2*j)) =  2*(x(:,i)-x(:,j))';
                b(count) = gamma*h^3;   
                count = count + 1;
            end
        end
        
        if ~isempty(conn_matrix)
            for ijk = 1:num_conn_constraints
                i = conn_matrix(ijk,1);
                j = conn_matrix(ijk,2);
                h2 = conn_radius^2 - norm(x(1:2,i)-x(1:2,j))^2;
                A2(ijk, (2*i-1):(2*i)) = 2*(x(:,i)-x(:,j))';
                A2(ijk, (2*j-1):(2*j)) =  -2*(x(:,i)-x(:,j));
                b2(ijk) = gamma*h2^3;
            end
        end
        
        %Solve QP program generated earlier
        vhat = reshape(dxi,2*N,1);
%         remain = remain/3;
%         remain = remain+1;
        reshaped_remain = reshape([remain;remain],[1,2*N]);
        H = 2*eye(2*N).*reshaped_remain;
        f = -2*vhat.*reshaped_remain';
        
%       vnew = quadprog(sparse(H), double(f), [A;A2], [b;b2], [],[], [], [], dxi, opts); % the second last term changed from [] to dxi, initial point
        
        if ~admm_flag
            opts.ConstraintTolerance = 1e-20;
            vnew = quadprog(sparse(H), double(f), [A;A2], [b;b2], [],[], -max_velocity*ones(2*N,1), max_velocity*ones(2*N,1), dxi, opts); % the second last term changed from [] to dxi, initial point
        else
            
            optts = [];
            optts.method = 'qp-1-over';
            optts.rho = 'optimal';
            % opts.rho = 100000;
            optts.displevel = 0;
            optts.iterplot = 0; % don't display
            optts.tol = 1e-6;
            optts.preconditioning = 0;
            optts.maxiter = 10000;
            
            % optional parameter structure
            p = [];
            
            vnew = qp_admm(sparse(H),double(f), [A;A2], [b;b2],[],[],-max_velocity*ones(2*N,1), max_velocity*ones(2*N,1),p,optts);
            
            
        end
        
        
        %Set robot velocities to new velocities
        dx = reshape(vnew, 2, N);
    end
end

