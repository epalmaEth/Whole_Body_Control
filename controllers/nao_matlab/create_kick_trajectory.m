function kick_trajectory = create_kick_trajectory(ball_position, target_position, kick_foot_position)
% Calculate kick trajectory of the kicking foot based on the ball, target,
% and currentfoor positions (which is the swingback position).

global kick_type

ball_radius = 0.05;
if ~kick_type  % Slow and precise kick
  direction_offset = 0.14; % How far the second control point should be 
else           % Fast kick
  direction_offset = 0.15;
end

% Obtain first control point
P1 = kick_foot_position; % 1st- swingback position which is where kicking leg is at the beginning

% Obtain last control point
% It should be on the surface of the ball, at the point where the robot
% should kick
x_diff = target_position(1) - ball_position(1);
z_diff = target_position(3) - ball_position(3);
r_diff = sqrt(x_diff^2 + z_diff^2);
x_P = (ball_radius*x_diff)/r_diff;  % From similar triangles
z_P = (ball_radius*z_diff)/r_diff;
if ~kick_type
    P4 = [ball_position(1) - x_P, ball_radius+0.2, ball_position(3) - z_P]; % Height of the kicking point is in the middle of the ball - slow kick
elseif kick_type == 1
    P4 = [ball_position(1), ball_radius + 0.20, ball_position(3)];   % fast kick
else
    P4 = [ball_position(1) + 3*x_P, ball_radius + 0.20, ball_position(3) + 3*z_P];   % penalty kick
end

% Obtain tangental control point
% It should be between robot and the ball, at the direction of the kick
x_P = (direction_offset*x_diff)/r_diff;  % From similar triangles
z_P = (direction_offset*z_diff)/r_diff;
if ~kick_type
  P3 = [ball_position(1)-x_P, ball_radius, ball_position(3)-z_P]; 
else
  P3 = [ball_position(1)-x_P, 0.05, ball_position(3)-z_P];
end

if ~kick_type
% Obtain second control point
    P3_P4_x_diff = P4(1)-P3(1);
    P3_P4_z_diff = P4(3)-P3(3);
    P3_P4_r = sqrt(P3_P4_x_diff^2 + P3_P4_z_diff^2);
    P3_P4_angle = atan2(P3_P4_z_diff, P3_P4_x_diff);
    P1_P4_x_diff = P4(1)-P1(1);
    P1_P4_z_diff = P4(3)-P1(3);
    P1_P4_angle = atan2(P1_P4_z_diff, P1_P4_x_diff);
    alpha = P3_P4_angle - P1_P4_angle; % angle between P1P4 and P3P4 vectors
    P1_P2_angle = P1_P4_angle - alpha;
    P1_P2_z_diff = (P3_P4_r)*sin(P1_P2_angle);
    P1_P2_x_diff = (P3_P4_r)*cos(P1_P2_angle);
    P2 = [P1(1) + P1_P2_x_diff, (ball_radius+kick_foot_position(2))/2, P1(3) + P1_P2_z_diff];
end

% Create Bazier curve in 3D
if ~kick_type
    P = [P1', P2', P3', P4'];  %Create a matrix of control points
    t = linspace (0,1,8); % Interpolation of 10 points slow kick;  
elseif kick_type == 1
    P = [P1', P3', P4'];  %Create a matrix of control points
    t = linspace(0,1,6);
else
    P = [P1', P3', P4'];  %Create a matrix of control points
    t = linspace(0,1,5);
end

Q3D = Bezier(P,t);

% figure
% plot3(Q3D(1,:),Q3D(3,:),Q3D(2,:),'b','LineWidth',2),
% hold on
% plot3(P(1,:),P(3,:),P(2,:),'g:','LineWidth',2)        % plot control polygon
% plot3(P(1,:),P(3,:),P(2,:),'ro','LineWidth',2)        % plot control points
% plot3(ball_position(1),ball_position(3),ball_position(2), 'ko','LineWidth',5) 
% plot3(target_position(1), target_position(3),target_position(2), 'go','LineWidth',5)
% xlabel('x'); ylabel('y'); zlabel('z');
% view(3);
% box;

kick_trajectory = Q3D';

end