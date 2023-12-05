function [RshoulderRoll, LshoulderRoll, RshoulderPitch, LshoulderPitch] = arm_controller(com, com_ref, left_leg)

global RShoulderPitch LShoulderPitch RShoulderRoll LShoulderRoll

RShoulderPitch_angle = wb_motor_get_target_position (RShoulderPitch);
LShoulderPitch_angle = wb_motor_get_target_position (LShoulderPitch);

RShoulderRoll_angle = wb_motor_get_target_position (RShoulderRoll);
LShoulderRoll_angle = wb_motor_get_target_position (LShoulderRoll);

% Initialize angles if there should be no change
RshoulderRoll = RShoulderRoll_angle; 
RshoulderPitch = RShoulderPitch_angle;
LshoulderRoll = LShoulderRoll_angle; 
LshoulderPitch = LShoulderPitch_angle;

[com_x, ~, com_z] = get_xyz(com);
[com_x_ref, ~, com_z_ref] = get_xyz(com_ref);


% Proportioanl controller for z axis in Webots (movement left-right) 
Kp = 3;
e_z = com_z - com_z_ref;

if e_z < 0   % Use opposite arm to stabilize in z (y) direction
    if e_z < -0.001  %React if there is a 1 mm offset
        RshoulderRoll = RShoulderRoll_angle + Kp*e_z;
        %wb_console_print(sprintf('Roll: %i\n', e_z), WB_STDOUT);
        if RshoulderRoll < -1.32645    % Handle limit of shoulder roll - based on documentation and warnings
            RshoulderRoll = -1.32645;
        end
    end
else
    if e_z > 0.001  
        LshoulderRoll = LShoulderRoll_angle + Kp*e_z;
        %wb_console_print(sprintf('Roll: %i\n', e_z), WB_STDOUT);
        if LshoulderRoll > 1.32645    % Handle limit of shoulder roll - based on documentation and warnings
            LshoulderRoll = 1.32645;
        end
    end
end

% Proportioanl controller for x axis in Webots (movement front-back) 
Kp = 1;
e_x = com_x - com_x_ref;

if e_x > 0.001 ||  e_x < -0.001  %React if there is a 1 mm offset
    RshoulderPitch = RShoulderPitch_angle + Kp*e_x;
    LshoulderPitch = LShoulderPitch_angle + Kp*e_x;
    if RshoulderPitch > 2.08567    % Handle limit of shoulder pitch - based on documentation and warnings
        RshoulderPitch = 2.08567;
    end
    if LshoulderPitch > 2.08567  
        LshoulderPitch = 2.08567;
    end
    if RshoulderPitch < -1.32645    % Handle limit of shoulder pitch
        RshoulderPitch = -1.32645;
    end
    if LshoulderPitch < -1.32645   
        LshoulderPitch = -1.32645;
    end
    %wb_console_print(sprintf('Pitch: %i\n', e_x), WB_STDOUT);
end

end
