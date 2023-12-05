function [x_x, x_y, p_x, p_y] = preview_controller(A_disc, B_disc, C_disc, Ks, Kx, Gp, ref_x, ref_y, x_x, x_y, N)%, observer_error_x, observer_error_y)
% Preview controller tracks refernece ZMP, with control input which
% constist of integral action to track error, state feedback (LQR) and
% preview gain for tracking future reference.
%
% @param A,B,C_dis -> Model
% @param Ks, Kx, Gp -> Controller gains
% @param ref_x,y -> Array of reference ZMP, x and y coordinates
% @param x_x,y -> State, x and y coordinates
% @return zmp_x,y -> ZMP of the robot,x and y coordinates
% @return x_x,y -> Desired CoM of the robot,x and y coordinates

% ___________Controller___________
p_x = C_disc * x_x;
p_y = C_disc * x_y;    

e_x = (ref_x(1) - p_x);    % Reference x axis zmp- real x axis zmp
e_y = (ref_y(1) - p_y);

gp_sum_x = 0;
gp_sum_y = 0;
j = 1;
for n=0:N
    gp_sum_x = gp_sum_x + Gp(j) * ref_x(1+j);
    gp_sum_y = gp_sum_y + Gp(j) * ref_y(1+j);
    j = j+1;
end
% Calculate optimal inputs

u_x = -Ks *e_x- Kx * x_x - gp_sum_x;
u_y = -Ks *e_y - Kx * x_y - gp_sum_y;

% Update state
Lx = diag([0.2 0.2 0.2]);
Ly = diag([0.2 0.2 0.2]);

x_x = A_disc * x_x + B_disc * u_x;% + Lx* observer_error_x;
x_y = A_disc * x_y + B_disc * u_y;% + Ly * observer_error_y;   
end
