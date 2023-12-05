function [footPitch, footRoll, hipPitch, hipRoll] = IK(com, ankle, L1, L2)
% @param com, ankle -> Positions in world coordinates




[com_x, com_y, com_z] = get_xyz(com);
[ankle_x, ankle_y, ankle_z] = get_xyz(ankle);

r = sqrt((com_y - ankle_y)^2 + (com_z - ankle_z)^2);

alpha = (L1^2 + L2^2 - r^2)/(2*L1*L2);
if alpha >= 1
    alpha = 0.0;
elseif alpha <= -1
    alpha = pi;
else
    alpha = acos( alpha );
end

hipRoll = alpha - pi;

beta = (r^2 + L1^2 - L2^2)/(2*L1*r);
if beta >= 1
    beta = 0.0;
elseif beta <= -1
    beta = pi;
else
    beta = acos( beta );
end

a1 = abs(com_y-ankle_y) ;
a2 = abs(com_z-ankle_z) ;
gamma = atan2(a1, a2);

footRoll = gamma - beta;

%____________________PITCH________________________
r = sqrt((com_y - ankle_y)^2 + (com_x - ankle_x)^2);

alpha = (L1^2 + L2^2 - r^2)/(2*L1*L2);
if alpha >= 1
    alpha = 0.0;
elseif alpha <= -1
    alpha = pi;
else
    alpha = acos( alpha );
end

hipPitch = alpha - pi;

beta = (r^2 + L1^2 - L2^2)/(2*L1*r);
if beta >= 1
    beta = 0.0;
elseif beta <= -1
    beta = pi;
else
    beta = acos( beta );
end

a1 = abs(com_y-ankle_y);
a2 = abs(com_x-ankle_x);
gamma = atan2(a1, a2);

footPitch = gamma - beta;



end