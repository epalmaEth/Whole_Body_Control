function P = create_cropped_circle(r, center, height, com_support_foot, left_leg)
% Function creates a circle of feasible swingback positions
%
% @return P -> Matrix of all points in the circle
% @param r, center -> radius and diameter of the circle in x-z axis
% @param height -> circle height in y axis (above the floor)
% @com_rankle -> center position of the support foot

% Create the circle
th = (0:5:360)' ;  % 5deg is a step
x = center(1)+r*cosd(th) ;
z = center(2)+r*sind(th) ; 
y = height.*ones(size(th)) ;  % Ball dimensions is 0.1 m so the hight where the kick should be is 0.05m. 

% Cropp the circle
if left_leg
    allowed_z_movement = com_support_foot(3)-0.07;
    for th = 20:5:165 % For these angles kicking leg would collide with the support leg
      %Calculation from similar triangles
      z(th/5) = allowed_z_movement;
    end
    
else
    allowed_z_movement = com_support_foot(3)+0.07;
    for th = 195:5:340 % For these angles kicking leg would collide with the support leg
      %Calculation from similar triangles
      z(th/5) = allowed_z_movement;
    end
end

P = [x y z] ;

end