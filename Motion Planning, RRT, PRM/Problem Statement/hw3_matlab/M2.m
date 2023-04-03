% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)


function [samples, adjacency] = M2(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_centers, sphere_radii)

% initialize
samples = zeros(num_samples, 4);
adjacency = zeros(num_samples, num_samples);


for i = 1:num_samples
    
    %chcek if within joint-limits, this piece is same as - M0
    samples(i,:) = (q_max - q_min).*rand(1,4) + q_min;

    while check_collision(robot, samples(i,:), link_radius, sphere_centers, sphere_radii)
        samples(i,:) = (q_max - q_min).*rand(1,4) + q_min;
    end
end


%if size(adjacency, 1) < num_samples
%        adjacency(num_samples, num_samples) = 0;
%end

for i = 1:num_samples
    % Using knnserach inbuilt functions to get neighbors within K = 10 and
    % respective distances.
    [neighbors, distances] = knnsearch(samples, samples(i,:), 'K', num_neighbors + 1);
    
    
    % loop over neighbors obtained using knnsearch
    for j = 2:length(neighbors)
        neighbor = neighbors(j);
        distance = distances(j);
		
        % added this condition as I was getting out of bound error.	
	  %if i <= size(samples, 1)

        % check if the edge between the sample configuration and neighbor is collision-free

        if ~check_edge(robot, samples(i,:), samples(neighbor,:), link_radius, sphere_centers, sphere_radii)
            adjacency(i, neighbor) = distance;
            %as adjacency matrix is symmetric, hence assigning the same
            %value at exactly opposite location
            adjacency(neighbor, i) = distance;
        else
            adjacency(i, neighbor) = 0;
            adjacency(neighbor, i) = 0;
        end

    end
end

end