% MATLAB controller for Webots
% File:          my_first_controller.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 64;

wb_console_print(sprintf('Hello World!\n'), WB_STDOUT);

wb_robot_init();
  
root = wb_supervisor_node_get_root();
children = wb_supervisor_node_get_field(root, 'children');
robot = wb_supervisor_field_get_mf_node(children, 6);


forwards_motion = wbu_motion_new('../../motions/Forwards.motion');
wbu_motion_set_loop(forwards_motion, true);
wbu_motion_play(forwards_motion);

% get and enable devices, e.g.:
%  camera = wb_robot_get_device('camera');
%  wb_camera_enable(camera, TIME_STEP);
%  motor = wb_robot_get_device('motor');

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%
while wb_robot_step(TIME_STEP) ~= -1
  % wb_console_print(sprintf('Hello World!\n'), WB_STDOUT);

  % read the sensors, e.g.:
  %  rgb = wb_camera_get_image(camera);

  % Process here sensor data, images, etc.

  % send actuator commands, e.g.:
  %  wb_motor_set_postion(motor, 10.0);

  % if your code plots some graphics, it needs to flushed like this:
  drawnow;

end

% cleanup code goes here: write data to files, etc.
