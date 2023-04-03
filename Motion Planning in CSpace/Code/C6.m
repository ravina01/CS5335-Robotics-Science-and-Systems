% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_path -> Mx2 matrix containing a collision-free path from
%                  q_start to q_goal. Each row in q_path is a robot
%                  configuration. The first row should be q_start,
%                  the final row should be q_goal.
% Output: num_collisions -> Number of swept-volume collisions encountered
%                           between consecutive configurations in q_path

function num_collisions = C6(robot, obstacles, q_path)

num_collisions = 0;
for q = 1 : length(q_path)-1
    l1 = q_path(q,1);
    r1 = q_path(q,2);
    l2 = q_path(q+1,1);
    r2 = q_path(q+1,2);

%check if rotated links convulve
     
            %Frame 1
            T1 = trotz(l1);
            T1(1,4) =  robot.pivot1(1,1);
            T1(2,4) =  robot.pivot1(2,1);
            
            %Frame 2
            link = [robot.pivot2(1) robot.pivot2(2) 0 1];
            link = T1*link';
        
            T2 = T1*trotz(r1);
            T2(1,4) =  link(1,1);
            T2(2,4) =  link(2,1) ;
          
            %Frame 3
            T3 = trotz(l2);
            T3(1,4) =  robot.pivot1(1,1);
            T3(2,4) =  robot.pivot1(2,1) ;
            
            %Frame 4
            link = [robot.pivot2(1) robot.pivot2(2) 0 1];
            link = T3*link';
        
            T4 = T1*trotz(r2);
            T4(1,4) =  link(1,1);
            T4(2,4) =  link(2,1) ;
          
            for i = 1:4
               
                    %Link 1 : Bottom Link
                    b = [robot.link1(1,i), robot.link1(2,i), 0, 1];
                    c = T1*(b');
                    link1_at0(1,i) = c(1,1);
                    link1_at0(2,i) = c(2,1);
        
                    %Link 2 : Upper Link
                    b = [robot.link2(1,i), robot.link2(2,i), 0, 1];
                    c = T2*(b');
                    link2_at0(1,i) = c(1,1);
                    link2_at0(2,i) = c(2,1);

                    %Link 3 : Bottom Link
                    b = [robot.link1(1,i), robot.link1(2,i), 0, 1];
                    c = T3*(b');
                    link3_at0(1,i) = c(1,1);
                    link3_at0(2,i) = c(2,1);
        
                    %Link 4 : Upper Link
                    b = [robot.link2(1,i), robot.link2(2,i), 0, 1];
                    c = T4*(b');
                    link4_at0(1,i) = c(1,1);
                    link4_at0(2,i) = c(2,1);
            end
       
            L1 = polyshape(link1_at0(1,:), link1_at0(2,:));
            L2 = polyshape(link2_at0(1,:), link2_at0(2,:));
            L3 = polyshape(link3_at0(1,:), link3_at0(2,:));
            L4 = polyshape(link4_at0(1,:), link4_at0(2,:));
                        
            U1 = union(L2,L4);
            k1 = convhull(U1);
            U2 = union(L1,L3);
            k2 = convhull(U2);

            for i = 1:length(obstacles)
                
                %Another approach to solev this by taking area
                %P = [link2_at0,link4_at0];
                %P=P'
                %[k,V] = convhull(P);
                %conv_poly = polyshape(P(k,1),P(k,2));
                %conv_area = area(intersect(conv_poly, obstacles(i)));
                %conv_area = intersect(conv_poly, obstacles(i));
                %if(conv_area > 0)
                if(overlaps(k1, obstacles(i)) || overlaps(k2, obstacles(i)))
                    num_collisions = num_collisions+1;
                    plot(polyshape(link1_at0(1,:), link1_at0(2,:)), 'FaceColor', 'r');
                    plot(polyshape(link2_at0(1,:), link2_at0(2,:)), 'FaceColor', 'b');
                    plot(polyshape(link3_at0(1,:), link3_at0(2,:)), 'FaceColor', 'r');
                    plot(polyshape(link4_at0(1,:), link4_at0(2,:)), 'FaceColor', 'b');
                end
            end    
end
end