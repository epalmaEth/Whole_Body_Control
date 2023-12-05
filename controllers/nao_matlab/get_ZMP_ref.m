% Obtain ZMP in the initial position for the ZMP reference

%______________ZMP______________
% Second derivative of x 
grad = gradient(gradient(relative_com_x_collection)); 
x_second = grad(end);  
% Second derivative of y
grad = gradient(gradient(relative_com_y_collection));   
y_second = grad(end);
% Second derivative of z
grad = gradient(gradient(relative_com_z_collection));    
z_second = grad(end);

px(steps) = relative_com_x_collection(end) - (relative_com_z_collection(end))*x_second/(g);
py(steps) = relative_com_y_collection(end) - (relative_com_z_collection(end))*y_second/(g);
relative_ZMP = [px(steps), py(steps), -position(2)];  % Third coordinate is hight of ZMP, which should be just on the floor

%Go back to world coordinates
ZMP = world_coordinates(relative_ZMP);
ZMP(2) = 0; % Y coordinate is the hight and ZMP should be on the floor. Calculations have small offsets so just set this to 0

x_ZMP_collection(steps) = ZMP(1);
y_ZMP_collection(steps) = ZMP(2);
z_ZMP_collection(steps) = ZMP(3);