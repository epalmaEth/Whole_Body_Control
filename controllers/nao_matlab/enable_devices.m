function enable_devices(TIME_STEP)
global robot RShoulderPitch LShoulderPitch RShoulderRoll LShoulderRoll ...
    LHipRoll LHipPitch LKneePitch LAnklePitch LAnkleRoll ...
    RHipRoll RHipPitch RKneePitch RAnklePitch RAnkleRoll ...
    RHipYawPitch LHipYawPitch ...
    RAnkleRoll_end_point LAnkleRoll_end_point RAnklePitch_end_point ...
    LHipYawPitch_end_point RHipYawPitch_end_point ...
    LKneePitch_end_point RKneePitch_end_point ...
    ball target
 
% Define robot
wb_robot_init();
root = wb_supervisor_node_get_root(); % Gives you the scene
children = wb_supervisor_node_get_field(root, 'children');
robot = wb_supervisor_field_get_mf_node(children, 5); % 5th thing in the scene is robot

ball = wb_supervisor_field_get_mf_node(children, 6); % 6th thing in the scene is the ball
target = wb_supervisor_field_get_mf_node(children, 7); % 7th thing in the scene is kicking target

% Robot joints/motors
RShoulderPitch = wb_robot_get_device('RShoulderPitch');
LShoulderPitch = wb_robot_get_device('LShoulderPitch');
RShoulderRoll = wb_robot_get_device('RShoulderRoll');
LShoulderRoll = wb_robot_get_device('LShoulderRoll');

LHipRoll = wb_robot_get_device('LHipRoll');
LHipPitch = wb_robot_get_device('LHipPitch');
LKneePitch = wb_robot_get_device('LKneePitch');
LAnklePitch = wb_robot_get_device('LAnklePitch');
LAnkleRoll = wb_robot_get_device('LAnkleRoll');
LHipYawPitch = wb_robot_get_device('LHipYawPitch');

RHipRoll = wb_robot_get_device('RHipRoll');
RHipPitch = wb_robot_get_device('RHipPitch');
RKneePitch = wb_robot_get_device('RKneePitch');
RAnklePitch = wb_robot_get_device('RAnklePitch');
RAnkleRoll = wb_robot_get_device('RAnkleRoll');
RHipYawPitch = wb_robot_get_device('RHipYawPitch');

% Get R ankle solid - for position of foot
RAnkleRoll_node = wb_supervisor_node_get_from_device(RAnkleRoll);
RAnkleRoll_hinge_joint = wb_supervisor_node_get_parent_node(RAnkleRoll_node); % Type "HingeJoint"
RAnkleRoll_end_point_ref = wb_supervisor_node_get_field(RAnkleRoll_hinge_joint, 'endPoint'); % returns reference to the field.
RAnkleRoll_end_point = wb_supervisor_field_get_sf_node(RAnkleRoll_end_point_ref); % get actually object 

% Get R ankle pitch
RAnklePitch_node = wb_supervisor_node_get_from_device(RAnklePitch);
RAnklePitch_hinge_joint = wb_supervisor_node_get_parent_node(RAnklePitch_node); % Type "HingeJoint"
RAnklePitch_end_point_ref = wb_supervisor_node_get_field(RAnklePitch_hinge_joint, 'endPoint'); % returns reference to the field.
RAnklePitch_end_point = wb_supervisor_field_get_sf_node(RAnklePitch_end_point_ref); % get actually object 

% Get L ankle solid - for position of foot
LAnkleRoll_node = wb_supervisor_node_get_from_device(LAnkleRoll); % Bcs LAnkleroll is just a number
LAnkleRoll_hinge_joint = wb_supervisor_node_get_parent_node(LAnkleRoll_node); % Type "HingeJoint"
LAnkleRoll_end_point_ref = wb_supervisor_node_get_field(LAnkleRoll_hinge_joint, 'endPoint'); % returns reference to the field.
LAnkleRoll_end_point = wb_supervisor_field_get_sf_node(LAnkleRoll_end_point_ref); % get actually object 

% Get R hip joint - for position of hip
RHipYawPitch_node = wb_supervisor_node_get_from_device(RHipYawPitch);
RHipYawPitch_hinge_joint = wb_supervisor_node_get_parent_node(RHipYawPitch_node);
RHipYawPitch_end_point_ref = wb_supervisor_node_get_field(RHipYawPitch_hinge_joint, 'endPoint');
RHipYawPitch_end_point = wb_supervisor_field_get_sf_node(RHipYawPitch_end_point_ref);

% Get L hip joint - for position of hip
LHipYawPitch_node = wb_supervisor_node_get_from_device(LHipYawPitch);
LHipYawPitch_hinge_joint = wb_supervisor_node_get_parent_node(LHipYawPitch_node);
LHipYawPitch_end_point_ref = wb_supervisor_node_get_field(LHipYawPitch_hinge_joint, 'endPoint');
LHipYawPitch_end_point = wb_supervisor_field_get_sf_node(LHipYawPitch_end_point_ref);

% Get R knee - joint position
RKneePitch_node = wb_supervisor_node_get_from_device(RKneePitch);
RKneePitch_hinge_joint = wb_supervisor_node_get_parent_node(RKneePitch_node);
RKneePitch_end_point_ref = wb_supervisor_node_get_field(RKneePitch_hinge_joint, 'endPoint');
RKneePitch_end_point = wb_supervisor_field_get_sf_node(RKneePitch_end_point_ref);

% Get L knee - joint position
LKneePitch_node = wb_supervisor_node_get_from_device(LKneePitch);
LKneePitch_hinge_joint = wb_supervisor_node_get_parent_node(LKneePitch_node);
LKneePitch_end_point_ref = wb_supervisor_node_get_field(LKneePitch_hinge_joint, 'endPoint');
LKneePitch_end_point = wb_supervisor_field_get_sf_node(LKneePitch_end_point_ref);

%______Move arms so that they don't colide with the body_______
wb_motor_set_position(LShoulderRoll, 0.10); 
wb_motor_set_position(RShoulderRoll, -0.10);


% %______Initialize robot in the beginning to stand up straight______
% % Run this once and then save the scene
 wb_motor_set_position(LShoulderPitch, pi/2);
 wb_motor_set_position(RShoulderPitch, pi/2);

% %______Initialize joints_______
% wb_motor_set_position(LHipRoll, 0.0);
% wb_motor_set_position(RHipRoll, 0.0);
% 
% wb_motor_set_position(LHipPitch, 0.0);
% wb_motor_set_position(LAnklePitch, 0.0);
% wb_motor_set_position(LAnkleRoll, 0.0);
% wb_motor_set_position(LKneePitch, 0.0);
% 
% wb_motor_set_position(RHipPitch, 0.0);
% wb_motor_set_position(RAnklePitch, 0.0);
% wb_motor_set_position(RAnkleRoll, 0.0);
% wb_motor_set_position(RKneePitch, 0.0);

end

