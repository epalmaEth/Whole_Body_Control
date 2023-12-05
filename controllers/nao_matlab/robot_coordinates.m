function relative_point = robot_coordinates(point)
% Calculates position of certain point in local (robot) coordinates
%
% @param position, rotation -> of the reference (robot)
% @param point -> what we want to see in local coordinates
% @return relative_point -> point in new coordinates
global robot
position = wb_supervisor_node_get_position(robot);
orientation = wb_supervisor_node_get_orientation(robot);

delta = [point(1)-position(1), point(2)-position(2), point(3)-position(3)];
relative_point = zeros(1,3); %initialization

for i = 0:1:2
s = 0;
for j = 0:1:2
  k = 3 * j;
  s = s + orientation(k+i+1) * delta(j+1);
end
relative_point(i+1) = s;
end

end