function [R_upper_leg, R_lower_leg, L_upper_leg, L_lower_leg] = calculate_robot_dimensions()
% Calculate dimesnions of robot (For now for right leg) based on positions
% of joints.
% These are dimensions in Webots. 
% In NAO documentation dimensions are:
%   lower leg -> 102.90 mm
%   upper leg -> 100 mm

global RAnkleRoll_end_point LAnkleRoll_end_point ...
    LHipYawPitch_end_point RHipYawPitch_end_point ...
    LKneePitch_end_point RKneePitch_end_point

%____________Get joint positions in world coordinates____________
RAnkleRoll_position = wb_supervisor_node_get_position(RAnkleRoll_end_point);
LAnkleRoll_position = wb_supervisor_node_get_position(LAnkleRoll_end_point);
RHipYawPitch_position = wb_supervisor_node_get_position(RHipYawPitch_end_point);
LHipYawPitch_position = wb_supervisor_node_get_position(LHipYawPitch_end_point);
RKneePitch_position = wb_supervisor_node_get_position(RKneePitch_end_point);
LKneePitch_position = wb_supervisor_node_get_position(LKneePitch_end_point); 

% ____________Calculating robot dimensions - RIGHT LEG_____________  
R_upper_leg = get_L(RKneePitch_position, RHipYawPitch_position);
R_lower_leg = get_L(RKneePitch_position, RAnkleRoll_position);

% ____________Calculating robot dimensions - LEFT LEG_____________  
L_upper_leg = get_L(LKneePitch_position, LHipYawPitch_position);
L_lower_leg = get_L(LKneePitch_position, LAnkleRoll_position);

end