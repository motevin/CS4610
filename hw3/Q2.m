% Localize a cylinder in the point cloud. Given a point cloud as input, this
% function should locate the position and orientation, and radius of the
% cylinder.
% input: ptCloud -> a pointCloud object that contains the point cloud (see
%                   Matlab documentation)
% output: center -> 3x1 vector denoting cylinder center
%         axis -> 3x1 unit vector pointing along cylinder axis
%         radius -> scalar radius of cylinder
function [center,axis,radius] = Q2(ptCloud)
    normals = pcnormals(ptCloud,20);
    point_count = ptCloud.Count;
    inlier_threshold = 0.01;
    ransac=[];

    %Radius Bounds
    r_floor=0.05;
    r_ceil=0.10;

    for runCount = 1:10000;
        % sample points
        sample1 = randi(point_count);
        sample2 = randi(point_count);

        sample1_pts = ptCloud.Location(sample1,:);
        sample2_pts = ptCloud.Location(sample2,:);

        sample1_norm = normals(sample1,:);
        sample2_norm = normals(sample2,:);

        sample1_unit = sample1_norm / norm(sample1_norm);
        sample2_unit = sample2_norm / norm(sample2_norm);

        % Candidate vals
        candidate_radius = r_floor + (r_ceil - r_floor) * rand; 
        candidate_ctr = sample1_pts - sample1_unit*candidate_radius;
        orientation = cross(sample1_norm,sample2_norm);
        candidate_axis = orientation / norm(orientation);

        % (I - aaT)
        proj = (eye(3) - candidate_axis' * candidate_axis);

        proj_plane = proj * ptCloud.Location';
        ctr1_proj = proj * candidate_ctr';

        dist = sqrt(sum((proj_plane - repmat(ctr1_proj,1, size(proj_plane,2))) .^2 , 1) );
        inliers = (dist > (candidate_radius - inlier_threshold)) & (dist < (candidate_radius + inlier_threshold));

        ransac(runCount,:) = [candidate_ctr, candidate_radius, candidate_axis, sum(inliers)];
    end
    % best so far
    [~, max_inlier] = max(ransac(:,8));

    center=ransac(max_inlier,1:3)'
    radius=ransac(max_inlier,4)
    axis=ransac(max_inlier,5:7)'

end

