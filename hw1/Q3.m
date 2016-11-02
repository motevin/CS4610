% TODO: You write this function!
% input: f -> an 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%                     effector position to reach <position>
%                     (orientation is to be ignored)
function q = Q3(f,qInit,posGoal)
q_nt = [1.5708, -0.3900, 0, 0.7850, 0, 0.7850, 0, -0.5000, 0.5000];
q = qInit;
p_eff = [0; 0; 0; 1];
x = f.fkine(q) * p_eff;
x_eff = x(1:3); % initial end-effector location

alpha = 0.01;
error = norm(posGoal - x_eff);
while (error > 0.01)
    delta_x = alpha * (posGoal - x_eff);
    delta_nt = alpha * (q_nt - q);
    
    J = f.jacob0(q, 'trans');
    Ji = pinv(J);
    J0 = Ji * J;
    N = eye(size(J0)) - J0;
    
    delta_q = (Ji * delta_x) + (N * delta_nt');
    q = q + delta_q';

    x = f.fkine(q) * p_eff;
    x_eff = x(1:3);

    error = norm(posGoal - x_eff);
    %disp(error);
end
end
    
    
