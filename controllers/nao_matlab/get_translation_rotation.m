function [translation, rotation] = get_translation_rotation(endPoint)
% Returns translation and rotation values 

translation = wb_supervisor_node_get_position(endPoint);
rotation = wb_supervisor_node_get_orientation(endPoint);
end