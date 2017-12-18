% Calculate a path from qStart to xGoal
% input: qStart -> 1x4 joint vector describing starting configuration of
%                   arm
%        xGoal -> 3x1 position describing desired position of end effector
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> qMilestones -> 4xn vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You may output any number of
%                    milestones. The first milestone should be qStart. The
%                    last milestone should place the end effector at xGoal.
function qMilestones = Q2(rob,sphereCenter,sphereRadius,qStart,xGoal)
    qMilestones = [];
    stepsize = 1;
    tree = [qStart]; % List of joint positions
    parent = [0]; % Pointer to this node's parent index in the tree
    
    % Goal Configuration
    T = transl(xGoal);
    qGoal = ikine(rob, T, qStart, [1 1 1 1 0 0],'pinv','alpha', 0.5);
    
    while(true)
        % Get Random Config
        if (rand<0.1)
            q_rand = qGoal;
        else
            q_rand = rand(size(qGoal)) * 2 * pi;
        end;
        
        % Nearest Node
        [q_near_idx, dist] = getNearestNeighbor(tree, q_rand);
        q_near = tree(q_near_idx,:);
        
        % Extend
        if (dist > stepsize)
            segments = max(10, fix(dist/stepsize));
            q_next = getNextQ(q_near, q_rand, segments);
        else
            q_next = q_rand;
        end;
        collision = Q1(rob, q_near, q_next, sphereCenter, sphereRadius);
        if(not(collision))
            tree = [tree; q_next];
            parent = [parent; q_near_idx];
        end;
        
        % Break if we've reached goal
        if(qGoal == q_next)
            break;
        end;
    end;
    
    % Get configurations from traversing tree backwards
    [j, ~] = size(tree);
    while(j ~= 0)
        qMilestones = [tree(j,:); qMilestones];
        j = parent(j);
    end;
end

% Get nearest node to q in N (list of nodes)
function [min_idx_so_far, min_dist_so_far] = getNearestNeighbor(N, q)
    min_dist_so_far = norm(N(1, :) - q);    
    min_idx_so_far = 1;

    [l, ~] = size(N);
    for k=2:l
        dist = norm(N(k, :) - q);
        if(dist < min_dist_so_far)
            min_idx_so_far = k;
            min_dist_so_far = dist;
        end;
    end;
end

% Get next configuration to extend to towards goal
% Generates a path between q1 & q2 and returns next point in path
% from hw2 q1
function q = getNextQ(q1, q2, segments)
    sample_pts = zeros(segments, 4);
    for i=1:length(q1)
        sample_pts(:,i) = linspace(q1(i), q2(i), segments)';       
    end;
    q = sample_pts(2, :);
end