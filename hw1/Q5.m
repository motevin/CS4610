% TODO: You write this function!
% input: f1 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        f2 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        qInit -> 1x11 vector denoting current joint configuration.
%                 First six joints are the arm joints. Joints 8,9 are
%                 finger joints for f1. Joints 10,11 are finger joints
%                 for f2.
%        f1Target, f2Target -> 3x1 vectors denoting the target positions
%                              each of the two fingers.
% output: q -> 1x11 vector of joint angles that cause the fingers to
%              reach the desired positions simultaneously.
%              (orientation is to be ignored)
function q = Q5(f1,f2,qInit,f1Target,f2Target)
q_nt = [0, -0.7800, 0, 1.5700, 0, 3.1416, 0, -1.0000, 1.0000, 1.0000, -1.0000];
T_goal1 = transl(f1Target);
T_goal2 = transl(f2Target);

f0 = [0 0; 0 0; 0 0; 0 0; 0 0; 0 0];

alpha = 0.01;
q = qInit;

error = 1; % arbitrary init error
while (error > 0.01)
    % split q
    q1 = [q(:, 1:7) q(:,  8: 9)];
    q2 = [q(:, 1:7) q(:, 10:11)];
    
    % get delta x
    T_x1 = f1.fkine(q1);
    T_x2 = f2.fkine(q2);
    delta_x1 = tr2delta(T_x1, T_goal1);
    delta_x2 = tr2delta(T_x2, T_goal2);
    delta_x = alpha * [delta_x1; delta_x2];
    
    % stacked jacobian
    J1 = f1.jacob0(q1);
    J1 = [J1(:, 1:9) f0];
    J2 = f2.jacob0(q2);
    J2 = [J2(:, 1:7) f0 J2(:, 8:9)];
    J = [J1; J2];
    Ji = pinv(J);

    % nullspace term
    J0 = Ji * J;
    N = eye(size(J0)) - J0;
    delta_nt = alpha * (q_nt - q);
    
    delta_q = (pinv(J) * delta_x) + (N * delta_nt');
    q = q + delta_q';
    
    norm_err = [norm(transl(T_goal1) - transl(T_x1)) norm(transl(T_goal2) - transl(T_x2))];
    error = mean(norm_err);
    %disp(error);
end

    
