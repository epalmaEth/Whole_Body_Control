function L1 = get_L(ankle, hip)
% Calculate lentgh from ankle to hip
%
% @param ankle, hip -> x,z,y coordinates of ankle/hip position

[ankle_x, ankle_y, ankle_z] = get_xyz(ankle);
[hip_x, hip_y, hip_z] = get_xyz(hip);

% Pythagoras theorem in 3D
L1 = sqrt((ankle_x - hip_x)^2 + (ankle_y - hip_y)^2 + (ankle_z - hip_z)^2);

end
