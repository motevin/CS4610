% Localize a sphere in the point cloud. Given a point cloud as input, this
% function should locate the position and radius of a sphere.
% input: ptCloud -> a pointCloud object that contains the point cloud (see
%                   Matlab documentation)
% output: center -> 3x1 vector denoting sphere center
%         radius -> scalar radius of sphere
function [center,radius] = Q1(ptCloud)
    points = ptCloud.Location;
    point_count = ptCloud.Count;
    inlier_threshold = 0.001;
    max_inliners = 0;
    
    for i=1:1000
        % sample points for candidate sphere model
        sample = points(randperm(point_count,4),:);

        X = sample(:,1);
        Y = sample(:,2);
        Z = sample(:,3);
        
        % Set up matrix for solving system of equations using the 4 sampled
        % points
        % Ax = B
        A = [X, Y, Z, ones(size(X))];
        x = -(X.^2 + Y.^2 + Z.^2);
        
        % solution of x^2 + y^2 + z^2 - ax - by - cz + d = 0
        B = A \ x;
        
        % center = -[a/2, b/2, c/2]
        candidate_center = -B(1:3) ./ 2;
        
        % r^2 = (a/2)^2 + (b/2)^2 + (c/2)^2 - d
        candidate_radius = sqrt(sum(candidate_center .^ 2) - B(4));

        % Evaluate fit using inliers
        C = repmat(candidate_center', size(points, 1), 1);
        dist = sqrt(sum((points - C) .^ 2, 2));
        inlier_count = sum(abs(dist - candidate_radius) < inlier_threshold);
        
        if(inlier_count > max_inliners) % best score so far
            center = candidate_center;
            radius = candidate_radius;
            max_inliners = inlier_count;
        end;
    end;
end
