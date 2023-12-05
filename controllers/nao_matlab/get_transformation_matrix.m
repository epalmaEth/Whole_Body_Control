function transformation_matrix = get_transformation_matrix(endPoint)
% Caluclate transformation matrix using translation and rotation of the
% field: T [R, t; 0, 1]

[translation, rotation] = get_translation_rotation(endPoint);
transformation_matrix = [rotation, translation'; 0 0 0 1];

end