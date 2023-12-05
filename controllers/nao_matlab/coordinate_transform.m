function destination_position = coordinate_transform(point, origin, destination)
% Goes from the origin to the destination coordinates
if origin == 0
	origin_position = [0 0 0];
	origin_orientation = eye(3);
else 
	origin_position = wb_supervisor_node_get_position(origin);
	origin_orientation = wb_supervisor_node_get_orientation(origin);
end	 
if destination == 0
	destination_position = [0 0 0];
	destination_orientation = eye(3);
else 
	destination_position = wb_supervisor_node_get_position(destination);
	destination_orientation = wb_supervisor_node_get_orientation(destination);
end

world_position = point*origin_orientation + origin_position;

destination_position = (world_position-destination_position)*inv(destination_orientation) ;

end
