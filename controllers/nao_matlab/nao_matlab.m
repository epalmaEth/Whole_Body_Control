%desktop;
%keyboard;

wb_console_print(sprintf('START \n'), WB_STDOUT);
TIME_STEP = 64;

global robot RShoulderPitch LShoulderPitch RShoulderRoll LShoulderRoll ...
    LHipRoll LHipPitch LKneePitch LAnklePitch LAnkleRoll ...
    RHipRoll RHipPitch RKneePitch RAnklePitch RAnkleRoll ...
    RHipYawPitch LHipYawPitch ...
    RAnkleRoll_end_point LAnkleRoll_end_point RAnklePitch_end_point ...
    LHipYawPitch_end_point RHipYawPitch_end_point ...
    LKneePitch_end_point RKneePitch_end_point ...
    ball target kick_type
enable_devices(TIME_STEP);

kick_type = 1;    % 0 -> short and precise kick
                  % 1 -> fast and strong kick
                  % 2 -> penalty kick; the strongest but works only for the
                  % front kick


%______________________Initialize preview controller_______________________
com = wb_supervisor_node_get_center_of_mass(robot);
dt = TIME_STEP*0.001;  % Sample time for the model
N = 50;                % Controller horizon
Q = 10; R = 0.0001;
[A_disc, B_disc, C_disc, Ks, Kx, Gp] = preview_controller_create_model(dt, N, com, Q, R);

%________________________Decide which leg kicks____________________________
position = wb_supervisor_node_get_position(robot);
orientation = wb_supervisor_node_get_orientation(robot);
ball_position = wb_supervisor_node_get_position(ball);
target_position = wb_supervisor_node_get_position(target);
left_leg = decide_leg(position, ball_position, target_position);


%____________________________Reference ZMP_________________________________
% Reference ZMP (px and py) in the supporting foot
if left_leg % Kick with left leg, right leg is support leg
    com_support_ankle = wb_supervisor_node_get_center_of_mass(RAnkleRoll_end_point);
    com_support_ankle_end_point = RAnkleRoll_end_point;
else        % Kick with right leg, left leg is support leg
    com_support_ankle = wb_supervisor_node_get_center_of_mass(LAnkleRoll_end_point); 
    com_support_ankle_end_point = LAnkleRoll_end_point;
end
relative_com_support_ankle = [0 -0.00 0]; %coordinate_transform(com_support_ankle, 0, robot); 
px_ref_value_side = relative_com_support_ankle(1);
py_ref_value_side = relative_com_support_ankle(2);

% Reference ZMP (px and py) in the middle
initial_com = com; 
px_ref_value_middle = initial_com(1);
py_ref_value_middle = initial_com(2);

% Create sufficiently long reference ZMP arrays
px_ref_side = px_ref_value_side .* ones(1, N+2);
py_ref_side = py_ref_value_side .* ones(1, N+2);



% Initial CoM speed and acceleration
x_x = [0; 0; 0];  % 1st param will be changed in the loop 
x_y = [0; 0; 0];
x_x_is = [0 ; 0 ; 0];
x_y_is = [0 ; 0 ; 0];

x_x_old = [0 ; 0 ; 0];
x_y_old = [0 ; 0 ; 0];

% Calculate offset between CoM and torso position


correction_x = 0;
correction_y = 0;
correction_z = 0;
x_z = com(2);
% Positions of support ankle and body orientation should not change; Kick leg p and R are fixed for the shifting of CoM
Rankle_orientation = wb_supervisor_node_get_orientation(RAnkleRoll_end_point);
Lankle_orientation = wb_supervisor_node_get_orientation(LAnkleRoll_end_point);
body_orientation = wb_supervisor_node_get_orientation(robot);

if left_leg
    support_leg_position = wb_supervisor_node_get_position(RAnkleRoll_end_point);
    kick_leg_position = wb_supervisor_node_get_position(LAnkleRoll_end_point); 
else
    support_leg_position = wb_supervisor_node_get_position(LAnkleRoll_end_point);
    kick_leg_position = wb_supervisor_node_get_position(RAnkleRoll_end_point);     
end

% Save kicking leg initial position
kick_foot_position_init = kick_leg_position;
% Set step hight of the kicking leg (2.5 cm of the floor)
kick_foot_position_stepHight = kick_leg_position;
kick_foot_position_stepHight(2) = kick_foot_position_stepHight(2) + 0.05;

%______________Create a circle of feasible kicks of the leg________________
% Hight of swingback is 5.5cm above floor
% Center of the circle is the initial position of the leg (lifted kicking leg near the body)  
r = 0.1 ;   % Radius that the leg can reach: 1 dm; experimentally determined  
circle = create_cropped_circle(r, [kick_foot_position_stepHight(1), kick_foot_position_stepHight(3)], kick_foot_position_stepHight(2)+0.05, com_support_ankle, left_leg); 

flag_change_phase = 0; % initialize flag
%__________________________________________________________________________
% _______________________________main loop_________________________________
%__________________________________________________________________________
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination

g = 9.81;
steps = 0;  % Required for plotting the time


while wb_robot_step(TIME_STEP) ~= -1
    steps = steps + 1;

    %_____________________Time triggered kick phases_______________________
    if(steps == 1)%1
        wb_console_print(sprintf('Phase 1 - Move CoM\n'), WB_STDOUT);
        kick_phase = 1; 
        
    elseif(steps == 20)%20 
        wb_console_print(sprintf('Phase 2 - Lift the leg\n'), WB_STDOUT);
        kick_phase = 2; 
        
        % Save current CoM as a refference for arm controller
        com_ref = wb_supervisor_node_get_center_of_mass(robot); 
        % Lift kicking leg to the step hight
        kick_leg_position = kick_foot_position_stepHight;       
      
    elseif(steps == 35)%35
        wb_console_print(sprintf('Phase 3 - KICK - Swing back\n'), WB_STDOUT);
        kick_phase = 3;               

        % Obtain the swingback angle 
        ball_position = wb_supervisor_node_get_position(ball);
        if left_leg
            kick_foot_position = wb_supervisor_node_get_position(LAnkleRoll_end_point);
        else
            kick_foot_position = wb_supervisor_node_get_position(RAnkleRoll_end_point);
        end
        angle = swingback_angle(ball_position, kick_foot_position);

        % Create matrix of movement from initial to swingback position
        swing_back_trajectory = linear_interpolation(kick_leg_position, circle(angle,:), 10);
        counter_swingback = 1;
        
    %_____________________Flag triggered kick phases_______________________
    elseif kick_phase == 3 && flag_change_phase == 1
        wb_console_print(sprintf('Phase 4 - KICK - Kick the ball\n'), WB_STDOUT);
        kick_phase = 4;
        flag_change_phase = 0;
        
        % Create matrix of movement for the kick
        ball_position = wb_supervisor_node_get_position(ball);
        target_position = wb_supervisor_node_get_position(target);
        if left_leg
            kick_foot_position = wb_supervisor_node_get_position(LAnkleRoll_end_point);
        else
            kick_foot_position = wb_supervisor_node_get_position(RAnkleRoll_end_point);
        end
        kick_trajectory = create_kick_trajectory(ball_position, target_position, kick_foot_position);
        counter_kick = 1;
        
        % Calculate distances between points; relevant for velocity
        for i=2:size(kick_trajectory,1)
            distance = kick_trajectory(i,1)-kick_trajectory(i-1,1);
            %wb_console_print(sprintf('Distance: (%i)\n', distance ), WB_STDOUT);
        end
    
    elseif kick_phase == 4 && flag_change_phase == 1
        wb_console_print(sprintf('Phase 5 - KICK - Return leg\n'), WB_STDOUT);
        kick_phase = 5;
        flag_change_phase = 0;
        
        % Create matrix of movement from the kicking to the initial position
        return_trajectory = linear_interpolation(kick_leg_position, kick_foot_position_stepHight, 10);
        counter_return = 1;
    
    elseif kick_phase == 5 && flag_change_phase == 1
        wb_console_print(sprintf('Phase 6 - Leg to ground\n'), WB_STDOUT);
        kick_phase = 6;
        flag_change_phase = 0;
        
        % Create matrix of movement from the initial position to the ground
        interpolation_points = 10;
        legtoground_trajectory = linear_interpolation(kick_foot_position_stepHight, kick_foot_position_init, interpolation_points);
        LShoulderRoll_trajectory = angle_interpolation(wb_motor_get_target_position(LShoulderRoll), 0.1, interpolation_points);
        RShoulderRoll_trajectory = angle_interpolation(wb_motor_get_target_position(RShoulderRoll), -0.1, interpolation_points);
        LShoulderPitch_trajectory = angle_interpolation(wb_motor_get_target_position(LShoulderPitch), pi/2, interpolation_points);
        RShoulderPitch_trajectory = angle_interpolation(wb_motor_get_target_position(RShoulderPitch), pi/2, interpolation_points);  
        counter_legtoground = 1;

    elseif kick_phase == 6 && flag_change_phase == 1
        wb_console_print(sprintf('Phase 7 - Return CoM\n'), WB_STDOUT);
        kick_phase = 7;
        flag_change_phase = 0;
        
        % Retune the controller
        Q = diag([100,10,1,1]); R = 0.0001; N = 10; 
        [A_disc, B_disc, C_disc, Ks, Kx, Gp] = preview_controller_create_model(dt, N, com, Q, R);
    end
    
    %_____________Apply force to robot to check stability__________________
    if(steps>40 && steps<53)
      wb_console_print(sprintf('NOW!\n'), WB_STDOUT);
      wb_supervisor_node_add_force(robot, [0,-0,0], true);
    end
    
    %if(steps>200 && steps<205)
    %  wb_console_print(sprintf('NOW2!\n'), WB_STDOUT);
    %  wb_supervisor_node_add_force(robot, [0, 10,0], true);
    %end
    
    
  
    %________________________________CoM___________________________________
    % In global 
    com = wb_supervisor_node_get_center_of_mass(robot); 
    
    % Calculating CoM in local coordinates
    position = wb_supervisor_node_get_position(robot);
    orientation = wb_supervisor_node_get_orientation(robot);
    relative_com = coordinate_transform(com,0, com_support_ankle_end_point);
    offset = position - com;
    %___________________Add values to the collections______________________
    com_x_collection(steps) = com(1);
    com_y_collection(steps) = com(2);
    com_z_collection(steps) = com(3);

    relative_com_x_collection(steps) = relative_com(1);
    relative_com_y_collection(steps) = relative_com(2);
    relative_com_z_collection(steps) = relative_com(3);

    x_position_collection(steps) = position(1);
    y_position_collection(steps) = position(2);
    z_position_collection(steps) = position(3);

    %______________Call preview controller for stability___________________
    
    x_x_is(1) = relative_com(1); 
    if steps > 1
        x_x_is(2) = (x_x_is(1) - x_x_old(1)); 
    end
    if steps > 2     
        x_x_is(3)= (x_x_is(2) - x_x_old(2));
    else
        x_x(3)=0;
    end
    x_x_old = x_x_is;

    x_y_is(1) = relative_com(2); 
    if steps > 1
        x_y_is(2) = (x_y_is(1) - x_y_old(1)); 
    end
    if steps > 2     
        x_y_is(3)= (x_y_is(2) - x_y_old(2));
    else
        x_y(3)=0;
    end
    x_y_old = x_y_is;

    
    alpha = 0.5;
    x_x1_collection(steps) = x_x_is(1);
    x_x2_collection(steps) = x_x_is(2);
    x_x3_collection(steps) = x_x_is(3);
    
    x_y1_collection(steps) = x_y_is(1);
    x_y2_collection(steps) = x_y_is(2);
    x_y3_collection(steps) = x_y_is(3);
    
     observer_error_x = x_x_is - x_x; 
     observer_error_y = x_y_is - x_y;
     observer_error_z = relative_com(3) - x_z;
     correction_x = correction_x - observer_error_x(1) * (alpha*0.9+0.1);
     correction_y = correction_y - observer_error_y(1) * alpha;
     correction_z = correction_z  -observer_error_z * 0.1;
% 
     observer_error_x_collections(steps) = observer_error_x(1);
     observer_error_y_collections(steps) = observer_error_y(1);

    
    x_x = x_x_is*alpha + x_x*(1-alpha);
    x_y = x_y_is * alpha + x_y *(1-alpha);
    if kick_phase < 7 % In first 6 phases, the ZMP is in the support foot
        [x_x, x_y, p_x, p_y] = preview_controller(A_disc, B_disc, C_disc, Ks, Kx, Gp, px_ref_side, py_ref_side, x_x, x_y, N);%, observer_error_x, observer_error_y);
       	p_x_ref_collection(steps) = px_ref_value_side;
    	p_y_ref_collection(steps) = py_ref_value_side;
    	x_z = relative_com(3);
    else              % In 7th phase, ZMP is in the middle
    	ref_value_middle = coordinate_transform(initial_com,0 ,com_support_ankle_end_point);
    	px_ref_middle = ref_value_middle(1) .* ones(1, N+2);
	    py_ref_middle = ref_value_middle(2) .* ones(1, N+2);

        [x_x, x_y, p_x, p_y] = preview_controller(A_disc, B_disc, C_disc, Ks, Kx, Gp, px_ref_middle, py_ref_middle, x_x, x_y, N);%, observer_error_x, observer_error_y);
        x_z = ref_value_middle(3)*0.2 +relative_com(3)*0.8;
    	p_x_ref_collection(steps) = px_ref_middle(1);
    	p_y_ref_collection(steps) = py_ref_middle(1);
    end
    
    % Add ZMP to the collections
    desired_x_x_collection(steps) = x_x(1);
    desired_x_y_collection(steps) = x_y(1);
    
    p_x_collection(steps) = p_x;
    p_y_collection(steps) = p_y;
    

    % Handle CoM output
    % x_x and x_y are x and y axis of desired CoM in robot coordinates
    desired_relative_com = [x_x(1)+correction_x, x_y(1) + correction_y, x_z+correction_z]; % Consider hight of CoM unchanged
    desired_com = coordinate_transform(desired_relative_com, com_support_ankle_end_point,0);
    % Calculate desired robot position based on the desired CoM
    desired_body_position = desired_com + offset;
    
    desired_com_x_collection(steps) = desired_com(1);
    desired_com_y_collection(steps) = desired_com(2);
    desired_com_z_collection(steps) = desired_com(3);
    
    
    %_________________Set positiond dependent on the phase_________________
    % Set desired position of the kicking foot, as long as the final position is not reached
    if kick_phase == 3 
        kick_leg_position = swing_back_trajectory(counter_swingback,:);
        counter_swingback = counter_swingback + 1;
        if counter_swingback > length(swing_back_trajectory) % if the final position is reached, proceed to the next phase
            flag_change_phase = 1;
        end
   
    elseif kick_phase == 4
        kick_leg_position = kick_trajectory(counter_kick,:);
        counter_kick = counter_kick + 1;
        if counter_kick > length(kick_trajectory) % if the final position is reached, proceed to the next phase
            flag_change_phase = 1;
        end 
        %wb_motor_set_velocity(LHipPitch, 4.48);
        %wb_motor_set_velocity(LKneePitch, 6.00);
        %wb_supervisor_node_set_velocity(LAnkleRoll_end_point, [-1,-1,-1,-1,-1,-1]);
        
    elseif kick_phase == 5
        counter_return = counter_return + 1;
        if counter_return > length(return_trajectory) % if the final position is reached, proceed to the next phase
            if counter_return > length(return_trajectory) + 1 % Wait for the stabilization
                flag_change_phase = 1;
            end
        else
            kick_leg_position = return_trajectory(counter_return,:);
        end
        wb_motor_set_velocity(LHipPitch, 6.40);
        wb_motor_set_velocity(LKneePitch, 6.40);
        
    elseif kick_phase == 6
        counter_legtoground = counter_legtoground + 1;
        if counter_legtoground > length(legtoground_trajectory)
            if counter_legtoground > length(legtoground_trajectory) + 1 % Wait for the stabilization
                flag_change_phase = 1;
            end
        else
            kick_leg_position = legtoground_trajectory(counter_legtoground,:);
            wb_motor_set_position(RShoulderRoll, RShoulderRoll_trajectory(counter_legtoground));
            wb_motor_set_position(RShoulderPitch, RShoulderPitch_trajectory(counter_legtoground));
            wb_motor_set_position(LShoulderRoll, LShoulderRoll_trajectory(counter_legtoground));
            wb_motor_set_position(LShoulderPitch, LShoulderPitch_trajectory(counter_legtoground));
        end
    end  
    
    %_____________Call inverse kinematics for the lower body_______________
    if left_leg
        [RkneePitch, RfootPitch, RfootRoll, RhipPitch, RhipRoll] = inverse_kinematics(1, desired_body_position, body_orientation, support_leg_position, Rankle_orientation);
        [LkneePitch, LfootPitch, LfootRoll, LhipPitch, LhipRoll] = inverse_kinematics(0, desired_body_position, body_orientation, kick_leg_position, Lankle_orientation);
    else
        [RkneePitch, RfootPitch, RfootRoll, RhipPitch, RhipRoll] = inverse_kinematics(1, desired_body_position, body_orientation, kick_leg_position, Rankle_orientation);
        [LkneePitch, LfootPitch, LfootRoll, LhipPitch, LhipRoll] = inverse_kinematics(0, desired_body_position, body_orientation, support_leg_position, Lankle_orientation);
    end
    %_______Call proportional controller for the arms when on one leg______
    if kick_phase > 1 && kick_phase < 6 %Use arms when on one leg
        % When robot is on one leg, table-cart assumption is broken and we activate arms to compensate for CoM change
        [RshoulderRoll, LshoulderRoll, RshoulderPitch, LshoulderPitch] = arm_controller(com, wb_supervisor_node_get_center_of_mass(com_support_ankle_end_point), left_leg);
        wb_motor_set_position(RShoulderRoll, RshoulderRoll);
        wb_motor_set_position(LShoulderRoll, LshoulderRoll);
        wb_motor_set_position(RShoulderPitch, RshoulderPitch);
        wb_motor_set_position(LShoulderPitch, LshoulderPitch);
    end
    

    %__________________Apply desired leg joint angles______________________
    wb_motor_set_position(LHipPitch, LhipPitch);
    wb_motor_set_position(LHipRoll, LhipRoll);
    wb_motor_set_position(LAnklePitch, LfootPitch);
    wb_motor_set_position(LAnkleRoll, LfootRoll);
    wb_motor_set_position(LKneePitch, LkneePitch);
    
    wb_motor_set_position(RHipPitch, RhipPitch);
    wb_motor_set_position(RHipRoll, RhipRoll);
    wb_motor_set_position(RAnklePitch, RfootPitch);
    wb_motor_set_position(RAnkleRoll, RfootRoll);
    wb_motor_set_position(RKneePitch, RkneePitch);
    
    % Save values for plotting
    AkneePitch(steps) = RkneePitch;
    AhipPitch(steps) = RhipPitch;
    AhipRoll(steps) = RhipRoll;
    AanklePitch(steps) = RfootPitch;
    AankleRoll(steps) = RfootRoll;
    
    %LHipPitch_velocity_collection(steps) = wb_motor_get_velocity(LHipPitch);
    velocity = wb_supervisor_node_get_velocity(LAnkleRoll_end_point);
    left_foot_velocity_collection_x(steps) = velocity(1);
    left_foot_velocity_collection_z(steps) = velocity(2);
    left_foot_velocity_collection_y(steps) = velocity(3);
    
    left_foot_velocity_collection_rotx(steps) = velocity(4);
    left_foot_velocity_collection_rotz(steps) = velocity(5);
    left_foot_velocity_collection_roty(steps) = velocity(6);
    
    
    if kick_phase == 3
       % wb_console_print(sprintf('Velocity swingback foot: (%i)\n', velocity(1) ), WB_STDOUT);
    end
    
    
    if kick_phase == 4 && flag_change_phase == 0
       % wb_console_print(sprintf('Velocity kick foot: (%i)\n', velocity(1) ), WB_STDOUT);
    end
  
    %_____________Print to console_________________________________________
    %wb_console_print(sprintf('Position:\n'), WB_STDOUT);
    %wb_console_print(sprintf(' - CoM worlds frame: (%i %i %i)\n', com(1), com(2), com(3)), WB_STDOUT);
    %wb_console_print(sprintf(' - CoM robots frame: (%i %i %i)\n', relative_com(1), relative_com(2), relative_com(3)), WB_STDOUT);
    
    %wb_console_print(sprintf(' - Velocity CoM: (%i %i)\n', x_x(2), x_y(2)), WB_STDOUT);
    %wb_console_print(sprintf(' - Acceleration: (%i %i)\n', x_x(3), x_y(3)), WB_STDOUT);

    %_____________PLOTS____________________________________________________
    %
    plot_collections;

end

