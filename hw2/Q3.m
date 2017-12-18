% Smooth path given in qMilestones
% input: qMilestones -> nx4 vector of n milestones. 
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> qMilestones -> 4xm vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You should output a number of
%                    milestones m<=n.
function qMilestonesSmoothed = Q3(rob,qMilestones,sphereCenter,sphereRadius)
    qMilestonesSmoothed = [];
    [num_milestones, ~] = size(qMilestones);
    
    % init first iteration
    q_curr = qMilestones(1, :);
    qMilestonesSmoothed = [qMilestonesSmoothed; q_curr];
    i = 1;
    
    while(i < num_milestones)
        q_curr = qMilestones(i, :);
       
        % try from last milestone backwards
        for j=num_milestones:-1:i
            q_goal = qMilestones(j,:);
            collision = Q1(rob, q_curr, q_goal, sphereCenter, sphereRadius);
            if (not(collision))
                i = j;
                qMilestonesSmoothed = [qMilestonesSmoothed; q_goal];
                break;
            end;
        end;
    end;
end
