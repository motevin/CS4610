% TODO: You write this function!
% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%                     effector position to reach <position>
%                     (orientation is to be ignored)
function q = Q2(f,qInit,posGoal)
q = qInit;
p_eff = [0; 0; 0; 1];
x = f.fkine(q) * p_eff;
x_eff = x(1:3); % initial end-effector location

alpha = 0.01;
error = norm(posGoal - x_eff);
while (error > 0.001)
    delta_x = alpha * (posGoal - x_eff);
    J = f.jacob0(q, 'trans');
    
    delta_q = pinv(J) * delta_x;
    q = q + delta_q';
    
    x = f.fkine(q) * p_eff;
    x_eff = x(1:3);
    
    error = norm(posGoal - x_eff);
    %disp(error);
end
end


