function [kneePitch, footPitch, footRoll, hipPitch, hipRoll] = inverse_kinematics(right_leg, body_position, body_orientation, ankle_position, ankle_orientation)
% Following the code release and the book
% Info about the robot from http://doc.aldebaran.com/2-1/family/index.html
% Works with world coordinates
%
% @param right_leg -> If true then this is IK for the right leg, if false then it is for the left

 
% Define robot dimensions
upper_leg = 0.1; % 100 mm from aldebaran website
lower_leg = 0.1029; %102.90 mm
if right_leg == 1  % Distance from torso to the right or left hip. Offset in y axis differs
    torso_hip_offset = [0; -0.05; -0.085; 1]; % Offset from the robot torso is the predefined distance
else
    torso_hip_offset = [0; 0.05; -0.085; 1];
end

% Get transformation matrix of the torso
transformation_matrix_robot = [body_orientation', body_position'; 0 0 0 1];

% Get hip position
hip_position =  transformation_matrix_robot * torso_hip_offset;  
hip_position = hip_position(1:3)'; % Get rid of last 1

% Get distance between ankle and hip
L1 = get_L(ankle_position, hip_position);
% Get relative pose of the hip node with respect to the ankle node 
Ankle2RHip = ankle_orientation * (hip_position - ankle_position)'; 
%L1 = norm(Ankle2RHip); % Alternative way

%______________Calculating knee angle______________________________________
outer_knee_pitch = (upper_leg^2 + lower_leg^2 - L1^2)/(2.0*upper_leg*lower_leg);
if outer_knee_pitch >= 1
    kneePitch = pi;
elseif outer_knee_pitch <= -1
    kneePitch = 0.0;
else
    kneePitch = pi - acos(outer_knee_pitch);
end

%______________Calculating ankle angles____________________________________
leg_triangle = (lower_leg^2 + L1^2 - upper_leg^2)/(2.0*lower_leg*L1) ;
if leg_triangle >= 1
    footPitch1 = 0.0;
elseif leg_triangle <= -1
    footPitch1 = pi;
else
    footPitch1 = acos( leg_triangle );
end

x = Ankle2RHip(1);
y = Ankle2RHip(2);
z = Ankle2RHip(3);

footPitch2 = atan2( x, sqrt(y^2 + z^2) );
footPitch =  -footPitch1 - footPitch2;

footRoll = atan2(y,z); 

%______________Calculating hip angles______________________________________
% Create matrix using the kinematic chain
Rot_x_footRoll = [1,0,0; 
    0, cos(-footRoll), -sin(-footRoll);
    0, sin(-footRoll), cos(-footRoll)];

Rot_y_footPitch_kneePitch = [cos(-footPitch-kneePitch), 0, sin(-footPitch-kneePitch);
    0, 1, 0;
    -sin(-footPitch-kneePitch), 0, cos(-footPitch-kneePitch)];

% R = body_orientation * ankle_orientation' * Rot_x_footRoll * Rot_y_footPitch_kneePitch ;
                                       % hipZ*hipX*hipY
R = body_orientation' * ankle_orientation * Rot_x_footRoll' * Rot_y_footPitch_kneePitch;

%hipYaw = atan2(-R(1,2),R(2,2));   % Extract hip yaw
% hipPitch = atan2( -R(3,1), R(3,3));  
% hipRoll = asin(R(3,2));% - pi/4;
hipPitch = asin(R(1, 3));
hipRoll = -asin(R(3, 2));

%______________Check if angles are feasible________________________________
% From website but also corrected when the code was throwing warning of limit
if right_leg == 1
    if hipRoll < -0.738274
        hipRoll = -0.738274;
    elseif hipRoll > 0.449597
        hipRoll = 0.449597;
    end
    
    if hipPitch < -1.77378
        hipPitch = -1.77378;
    elseif hipPitch > 0.48398
        hipPitch = 0.48398;
    end
    
    if kneePitch < -0.0923279
        kneePitch = -0.0923279;
    elseif kneePitch > 2.11255
        kneePitch = 2.11255;
    end
    
    if footPitch < -1.1863
        footPitch = -1.1863;
    elseif footPitch > 0.932006
        footPitch = 0.932006;
    end
    
    if footRoll < -0.768992
        footRoll = -0.768992;
    elseif footRoll > 0.397935
        footRoll = 0.397935;
    end
    
else
    if hipRoll < -0.379435
        hipRoll = -0.379435;
    elseif hipRoll > 0.79046
        hipRoll = 0.79046;
    end
    
    if hipPitch < -1.77378
        hipPitch = -1.77378;
    elseif hipPitch > 0.48398
        hipPitch = 0.48398;
    end
    
    if kneePitch < -0.0923279
        kneePitch = -0.0923279;
    elseif kneePitch > 2.11255
        kneePitch = 2.11255;
    end
    
    if footPitch < -1.18944
        footPitch = -1.18944;
    elseif footPitch > 0.922581
        footPitch = 0.922581;
    end
    
    if footRoll < -0.397880
        footRoll = -0.397880;
    elseif footRoll > 0.769001
        footRoll = 0.769001;
    end
end
    
end
