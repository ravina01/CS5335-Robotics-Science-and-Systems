% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = M5(robot, path, link_radius, sphere_centers, sphere_radii)

iter = 1;
old_j=1;
smoothed_path(1,:) = path(1,:);
n= size(path,1);
for i = 1:size(path,1)
    for j = i+1:size(path,1)
        if ~check_edge(robot, smoothed_path(iter,:), path(j, :), link_radius, sphere_centers, sphere_radii)
            old_j = j;
            
            break;
        end
        iter = iter+1;
        smoothed_path(iter,:) = path(old_j,:);
    end

%     if iter ~=i
%         smoothed_path = [smoothed_path; path(iter,:)];
%         i = iter;
%     end
end
smoothed_path = [smoothed_path; path(n,:)];

end

