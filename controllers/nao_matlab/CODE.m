% CODE


%Set positions of angles
wb_motor_set_position(RHipPitch, -0.5);
wb_motor_set_position(RHipRoll, -0.05);
wb_motor_set_position(RAnklePitch, -0.5);
wb_motor_set_position(RAnkleRoll, -0.05);


%___ Get real angles

ARAnkleRoll_angle = wb_motor_get_target_position (RAnkleRoll);
ARAnklePitch_angle = wb_motor_get_target_position (RAnklePitch);
ARHipRoll_angle = wb_motor_get_target_position (RHipRoll);
ARHipPitch_angle = wb_motor_get_target_position (RHipPitch);

  
%_____Apply force to robot to check stability______
if(steps>10)
%wb_supervisor_node_add_force(robot, [15,0,0], true);
end



%___
RAnkleRoll_2_RHipYawPitch = express_relative_to(LHipYawPitch_position, RHipYawPitch_end_point); % Right ankle porition relative to the right hip


%______Move arms to down position______
wb_motor_set_position(LShoulderPitch, pi/2);
wb_motor_set_position(RShoulderPitch, pi/2);


% Get axis parameter of Hip joint
LHipYawPitch_node = wb_supervisor_node_get_from_device(LHipYawPitch);
LHipYawPitch_hinge_joint = wb_supervisor_node_get_parent_node(LHipYawPitch_node);
LHipYawPitch_joint_parameters_ref = wb_supervisor_node_get_field(LHipYawPitch_hinge_joint, 'jointParameters');
LHipYawPitch_joint_parameters = wb_supervisor_field_get_sf_node(LHipYawPitch_joint_parameters_ref);
type = wb_supervisor_node_get_type_name(LHipYawPitch_joint_parameters);
L_axis_ref = wb_supervisor_node_get_field(LHipYawPitch_joint_parameters, 'axis');
L_axis = wb_supervisor_field_get_sf_vec3f(L_axis_ref);

% Make robot walk just so there is some motion
forwards_motion = wbu_motion_new('../../motions/Forwards.motion');
wbu_motion_set_loop(forwards_motion, true);
wbu_motion_play(forwards_motion);

% Get translation field of HipRoll
LAnkleRoll_children_ref = wb_supervisor_node_get_field(LAnkleRoll_end_point, 'children');
LAnkleRoll_children = wb_supervisor_field_get_sf_node(LAnkleRoll_children_ref);
LEFT_FOOT_SLOT = wb_supervisor_field_get_mf_node(LAnkleRoll_children_ref, 0); % First child of endPoint is LEF_FOOT_SLOT (line 1733)
LAnkle_translation_ref = wb_supervisor_node_get_field(LEFT_FOOT_SLOT, 'translation');
LAnkle_translation = wb_supervisor_field_get_sf_vec3f(LAnkle_translation_ref);

% Get R hip joint - for position of hip - using anchor
RHipYawPitch_node = wb_supervisor_node_get_from_device(RHipYawPitch);
RHipYawPitch_hinge_joint = wb_supervisor_node_get_parent_node(RHipYawPitch_node);
RHipYawPitch_joint_parameters_ref = wb_supervisor_node_get_field(RHipYawPitch_hinge_joint, 'jointParameters');
RHipYawPitch_joint_parameters = wb_supervisor_field_get_sf_node(RHipYawPitch_joint_parameters_ref);
RHip_anchor_ref = wb_supervisor_node_get_field(RHipYawPitch_joint_parameters, 'anchor');
RHip_anchor = wb_supervisor_field_get_sf_vec3f(RHip_anchor_ref);


% OLD APPROACH
com = wb_supervisor_node_get_center_of_mass(robot);

position = wb_supervisor_node_get_position(robot);
rotation = wb_supervisor_node_get_orientation(robot);

RHip_position = position + (RHip_anchor*rotation);
LHip_position = position + (LHip_anchor*rotation);

com_RAnkle = wb_supervisor_node_get_center_of_mass(RAnkleRoll_end_point);
com_LAnkle = wb_supervisor_node_get_center_of_mass(LAnkleRoll_end_point);

    %______PLOTS______
    %
    time = [1:steps] * TIME_STEP / 1000;
    
    figure(1);
    sgtitle('CoM');
    subplot(1,3,1); hold on;
    plot(time,com_x_collection(),'b');  
    plot(time,com_x_collection(),'bo');
    xlabel('time [s]');
    ylabel('X position [m]');

    subplot(1,3,2); hold on;
    plot(time,com_y_collection(),'b');  
    plot(time,com_y_collection(),'bo');
    xlabel('time [s]');
    ylabel('Y position [m]');

    subplot(1,3,3); hold on;
    plot(time,com_z_collection(),'b');  
    plot(time,com_z_collection(),'bo');
    xlabel('time [s]');
    ylabel('Z position [m]');


