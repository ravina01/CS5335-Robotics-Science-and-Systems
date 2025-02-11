% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
% Output: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise

function cspace = C2(robot, obstacles, q_grid)


for i = 1:length(q_grid)
    for j = 1 : length(q_grid)

        L1 = trotz(q_grid(i));
        L1(1,4) =  robot.pivot1(1,1);
        L1(2,4) =  robot.pivot1(2,1);
        
        new_pivot2 = [robot.pivot2(1) robot.pivot2(2) 0 1];
        new_pivot2 = L1 * new_pivot2';
        L2 = L1 * trotz(q_grid(j));
        L2(1,4) = new_pivot2(1,1);
        L2(2,4) = new_pivot2(2,1);
  
        new_poly_1 = zeros(2, 4);
        new_poly_2 = zeros(2, 4);
    
        for x = 1 : 4
            
            T1 = [robot.link1(1,x) robot.link1(2,x) 0 1];
            polyCordinates_1 = L1*T1';
            new_poly_1(1,x) = polyCordinates_1(1,1);
            new_poly_1(2,x) = polyCordinates_1(2,1);
    
            T2 = [robot.link2(1,x) robot.link2(2,x) 0 1];
            polyCordinates_2 = L2*T2';
            new_poly_2(1,x) = polyCordinates_2(1,1);
            new_poly_2(2,x) = polyCordinates_2(2,1);
        end

        % Plot the links
        P1 = polyshape(new_poly_1(1,:), new_poly_1(2,:));
        P2 = polyshape(new_poly_2(1,:), new_poly_2(2,:));
        
        link1_intersect = 0;
        link2_intersect = 0;
        for iter = 1 : length(obstacles)
            link1_intersect = link1_intersect + area(intersect(obstacles(iter), P1));
            link2_intersect = link2_intersect + area(intersect(obstacles(iter), P2));
            
        end
        if(link1_intersect + link2_intersect == 0)
            cspace(i,j) = 0;
        else
            cspace(i,j) = 1;
        end
    end
end

end