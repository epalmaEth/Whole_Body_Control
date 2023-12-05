function new_position = express_relative_to(position, relative)
% Express a point given in world coordinates in a coordinate system relative to the 
% 'relative' param
% @param relative -> node of the endPoint
% Equivalent to Rotation*position + translation

transformation_matrix = get_transformation_matrix(relative);
new_position = inv(transformation_matrix) * [position, 1]';
new_position = new_position(1:3)';

end