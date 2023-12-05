function point_world = world_coordinates(point)
% Goes from the robot to the world coordinates

global robot

position = wb_supervisor_node_get_position(robot);
orientation = wb_supervisor_node_get_orientation(robot);

point_world = point*orientation + position;

end