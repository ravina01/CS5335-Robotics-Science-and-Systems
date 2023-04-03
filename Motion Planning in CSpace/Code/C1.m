% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to plot the robot at

function C1(robot, q)
    % The following code plots the robot in configuration q = [0; 0].
    % You should remove the following code and replace it with code that
    % plots the robot links and pivots at the provided input configuration.
    
    % Translate frame origins
    origin1_at0 = robot.pivot1;
    origin2_at0 = origin1_at0 + robot.pivot2;
    % Compute link polygon corners
    link1_at0 = robot.link1 + origin1_at0;
    link2_at0 = robot.link2 + origin2_at0;

    %Rotate Link 1 about z Axis
    %Transl = [origin1_at0(1) origin1_at0(2) 0 1]
    L1 = trotz(q(1));
    L1(1,4) =  robot.pivot1(1,1);
    L1(2,4) =  robot.pivot1(2,1);
    
    new_pivot2 = [robot.pivot2(1) robot.pivot2(2) 0 1];
    new_pivot2 = L1 * new_pivot2';
    L2 = L1 * trotz(q(2));
    L2(1,4) = new_pivot2(1,1);
    L2(2,4) = new_pivot2(2,1);

    new_poly_1 = zeros(2, 4);
    new_poly_2 = zeros(2, 4);

    for i = 1 : 4
        T1 = [robot.link1(1,i) robot.link1(2,i) 0 1];
        polyCordinates_1 = L1*T1';
        new_poly_1(1,i) = polyCordinates_1(1,1);
        new_poly_1(2,i) = polyCordinates_1(2,1);

        T2 = [robot.link2(1,i) robot.link2(2,i) 0 1];
        polyCordinates_2 = L2*T2';
        new_poly_2(1,i) = polyCordinates_2(1,1);
        new_poly_2(2,i) = polyCordinates_2(2,1);
    end
    
    % Plot the links
    plot(polyshape(new_poly_1(1,:), new_poly_1(2,:)), 'FaceColor', 'r');
    plot(polyshape(new_poly_2(1,:), new_poly_2(2,:)), 'FaceColor', 'b');

    % Plot the pivot points
    plot(origin1_at0(1), origin1_at0(2), 'k.', 'MarkerSize', 10);
    plot(new_pivot2(1), new_pivot2(2), 'k.', 'MarkerSize', 10);
  
end
    
    
    
