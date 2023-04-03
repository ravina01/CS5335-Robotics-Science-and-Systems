% Input: q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples to sample
% Output: qs -> num_samples x 4 matrix of joint angles,
%               all within joint limits

function qs = M1(q_min, q_max, num_samples)

% q_min =  [0.3120   -0.8701         0   -2.1734]
% q_max = [0.6431   -1.6260         0   -3.0149 ]
    N = rand(4,num_samples); %N random values, 0 to 1
%     N =sort(N);
%     N=N';
    %N= (N-0.5)*2 % between -1 and 1
    for i=1:length(N)
     qs(i,1:4)= q_min(1:4) + ((q_max(1:4)-q_min(1:4)).*N(i));     
    end
end