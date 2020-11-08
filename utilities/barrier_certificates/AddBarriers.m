%% Get updated controls with collision avoidance, barriers, and max_velocity constraint
% Created by Wenhao Luo (whluo12@gmail.com)
% 4/23/2018
% Compatible to Simulator Format

function bots = AddBarriers(bots, varargin) %updates controllers with Barriers.

parser = inputParser;
addOptional(parser, 'conn_matrix', []); % how to select break-off swarm
addOptional(parser, 'SafeRadius', 1200);
addOptional(parser, 'ConnRadius', 1000);
addOptional(parser, 'MaxVelocity', 1000);

parse(parser, varargin{:});

conn_matrix = parser.Results.conn_matrix;
SafeRadius = parser.Results.SafeRadius;
ConnRadius = parser.Results.ConnRadius;
MaxVelocity = parser.Results.MaxVelocity;


states = [bots.state];
xx = [states.p];
dx = [bots.u];

% 
% % generate real-time connectivity matrix from robots' positions
% if isempty(bots(1).conn_matrix)
%     conn_matrix = GetConnMatrix(xx, ConnRadius^2);
% else
%     conn_matrix = bots(1).conn_matrix;    
% end
% 
% conn_matrix = GetConnMatrix(xx, ConnRadius^2);


barrier_cert = create_con_barrier_certificate('SafetyRadius', SafeRadius,...
    'ConnRadius',ConnRadius,'conn_matrix',conn_matrix,'max_velocity',MaxVelocity);
dx = barrier_cert(dx, xx);
N = numel(bots);

for ijk = 1:N
    bots(ijk).u = dx(:,ijk);    
    bots(ijk).conn_matrix = conn_matrix; % not necessary in other simulator
end




end