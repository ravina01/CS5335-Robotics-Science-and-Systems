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
 
    N= rand(num_samples); %N random values, 0 to 1
%     N = N';
%     N =sort(N);
    num_collision = 0;
    i=1;
%     samples(1,1:4)= [0 -pi/4 0 -pi/4];
%     samples(2,1:4)= [0 -3 0 -3];
%     i=3;
n=1;
    while(i<=num_samples)
        consider_q = q_min(1:4) + ((q_max(1:4)-q_min(1:4)).*N(n,1:4));
        n= n+1;
        if(n>100)
            n=1;
        end
      if check_collision(robot, consider_q, link_radius, sphere_centers, sphere_radii,100)
                num_collision = num_collision + 1;
      else
          samples(i,1:4)= consider_q;
          i = i+1;
         
      end
       
    end
   
    lc=num_samples
    for a = 1:lc
        
        source = samples(a,:);
         r = robot.fkine(source).t;
    
         b = r(:,1)';
         EE_pos(a,:) = b;

         combined_ang_ee(a,1:4) = samples(a,:) ;
         combined_ang_ee(a,5:7) = EE_pos(a,:);
    end
    
    
    c=combined_ang_ee
    
adjacency = eye(lc);
    r =0.5;
    for a = 1:lc
        xyz = combined_ang_ee(a,5:7);
        n_break =0;
        
        for b= 1:lc
        if((n_break < num_neighbors) || (b<lc)) && (a~=b) && (adjacency(a,b)==0) && (adjacency(b,a)==0)

            abc = combined_ang_ee(b,5:7);
            d =  sqrt(((xyz(1)-abc(1))^2) + ((xyz(2)-abc(2))^2) + ((xyz(3)-abc(3))^2));

            if(d>0) && (d<r)
                source = combined_ang_ee(a,1:4);
                neighbor = combined_ang_ee(b,1:4);
                %col_samples = M1(source, neighbor, 10)
                num_collision =0;
                col_samples = M1(source, neighbor, 10);

              for s=1:length(col_samples)
                if check_collision(robot, col_samples(s,:), link_radius, sphere_centers, sphere_radii)
                num_collision = num_collision + 1
                end
              end
                
                if(num_collision == 0)
                     adjacency(a,b)= d;
                     adjacency(b,a)= d;
                     n_break = n_break + 1;
                end
            end

        end

    end

     
   end
  a=adjacency

end