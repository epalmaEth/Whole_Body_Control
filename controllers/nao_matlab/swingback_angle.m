function angle = swingback_angle(ball_position, kick_foot_position)
% Calculate swingback angle based on the ball and foot positions.
% It should be on the opposite side of the ball 

x_diff = ball_position(1) - kick_foot_position(1);
z_diff = ball_position(3) - kick_foot_position(3);

% Angle between the leg and the ball
alpha = atan2d(z_diff, x_diff);

angle = round((180+alpha)/ 5); % integer division by 5 bcs angle step is 5 degrees

end