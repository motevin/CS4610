% 
% This function takes two joint configurations and the parameters of the
% obstacle as input and calculates whether a collision free path exists
% between them.
% 
% input: q1, q2 -> start and end configuration, respectively. Both are 1x4
%                  vectors.
%        sphereCenter -> 3x1 position of center of sphere
%        r -> radius of sphere
%        rob -> SerialLink class that implements the robot
% output: collision -> binary number that denotes whether this
%                      configuration is in collision or not.
function collision = Q1(rob,q1,q2,sphereCenter,r)
    % init
    collision = false;
    sample_pts = zeros(10, 4);
    
    % points to sample  
    for i=1:length(q1)
        sample_pts(:,i) = linspace(q1(i), q2(i), 10)';       
    end;
    
    % check for collisions
    for i=1:10
        collision = robotCollision(rob, sample_pts(i,:), sphereCenter, r);
        if(collision)
            break;
        end;
    end;
end

