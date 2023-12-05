function left_leg = decide_leg(robot_position, ball_position, target_position)
% Decide which leg will be kick leg
%
% @return left_leg -> if true then left leg is kick leg and right is
% support leg

x_diff = ball_position(1) - robot_position(1);
z_diff = ball_position(3) - robot_position(3);

ball_target_offset = target_position(3) - ball_position(3);

% Angle between the robot and the ball
alpha = atan2d(z_diff, x_diff);

if alpha <= -20 && alpha >= -165    % Ball cleary on the left side
    left_leg = 1;
elseif alpha >= 20 && alpha <= 165  % Ball cleary on the right side
    left_leg = 0;
elseif alpha >= -5 && alpha <= 5    % Ball almost in front
    if ball_target_offset > 0
        left_leg = 0;
    else
        left_leg = 1;
    end
elseif alpha >= 170 || alpha <= -170    % Ball almost in back
    if ball_target_offset > 0
        left_leg = 1;
    else
        left_leg = 0;
    end
elseif robot_position(3) <= ball_position(3) && ball_target_offset > 0.2
    left_leg = 1;
elseif robot_position(3) <= ball_position(3) && ball_target_offset <= 0.2
    left_leg = 0;
elseif robot_position(3) >= ball_position(3) && ball_target_offset > -0.2
    left_leg = 1;
elseif robot_position(3) >= ball_position(3) && ball_target_offset <= -0.2
    left_leg = 0;
else
    left_leg = 1;
end


if left_leg
    wb_console_print(sprintf('Left leg kicks'), WB_STDOUT);
else
    wb_console_print(sprintf('Right leg kicks'), WB_STDOUT);
end

end