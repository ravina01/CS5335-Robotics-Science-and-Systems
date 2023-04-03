% Input: q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples to sample
% Output: qs -> num_samples x 4 matrix of joint angles,
%               all within joint limits

function qs = M1(q_min, q_max, num_samples)
% it detects 22 collisions. 
q_min % -1.5708   -3.1416         0   -3.1416
q_max % 1.5708         0         0         0
num_samples % 100

    random_angles = rand(4,num_samples); %N uniformly distributed random values, 0 to 1
    %(q_max(1:4)-q_min(1:4)) % 3.1416    3.1416         0    3.1416
    for i=1:length(random_angles)
        qs(i,1:4)= q_min(1:4) + ((q_max(1:4)-q_min(1:4)).*random_angles(i)); 
    end
end