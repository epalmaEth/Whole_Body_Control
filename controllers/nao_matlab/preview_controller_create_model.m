function [A_disc, B_disc, C_disc, Ks, Kx, Gp] = preview_controller_create_model(dt, N, com, Q, R)
% @param dt -> Sample time
% @param N -> Horizon
% @return A,B,C_dis -> Model
% @return Ks, Kx, Gp -> Controller gains

% Some constants
zc = com(2);  % Height of CoM
g = 9.81;

% COSTS
% For stability using both legs in the middle: Q = [1,0,0;0,0,0;0,0,0]; R = 0.1
% For stability using both legs on right leg: Q = [10,0,0;0,10,0;0,0,10]; R = 0.0001
Q = Q.*[1 0 0 0; 
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

% Continuous time model
A = [0,1,0;
     0,0,1;
     0,0,0];
B = [0;0;1];
C = [1 0 -zc/g];
D = 0;
system_cont = ss(A, B, C, D);      

% Discretized model: x(k+1) = A x(k) + B u(k); p(k) = C x(k)      
system_disc = c2d(system_cont, dt);
[A_disc, B_disc, C_disc, ~] = ssdata(system_disc);

% Improved model to get rid of C
A_tilde = [1, C_disc*(A_disc);
           zeros(3,1), A_disc];
B_tilde = [C_disc*B_disc; B_disc];
% C_tilde = [1,0,0,0];

% Solve ARE
[K, P] = dlqr(A_tilde, B_tilde, Q, R);

Ks = K(1);     % Integral gain; intergal action tracks error
Kx = K(2:end); % State gain; for state feedback

% Calculate preview gain Gp
Gp = zeros(1,N+1);
Gp(1,1) = -1*Ks;

Ac = A_tilde - B_tilde * K;
I = [1;0;0;0];
X_tilde = -Ac'*P*I ;  %First x tilde

for i = 2:(N+1)
    Gp(1,i) = (R+B_tilde'*P*B_tilde)^(-1)*B_tilde'*X_tilde;
    X_tilde = Ac'*X_tilde;
end


end
