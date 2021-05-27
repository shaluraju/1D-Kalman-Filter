
clc, clear 
%reply = input('Please run the code section wise; press any number to continue: ');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Some parameters you can use to set up your solution of SimLab1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
length_hallway = 10;
sensor_range = 0.5;

landmarks = [2; 5; 8];
num_landmarks = length(landmarks);

stdev_odometry = 0.1; % m/s
stdev_range = 0.01; % m

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot's "Ground Truth" Trajectory when it travels the hallway 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

delta_t = 0.1; % time is discretized with a time-step of 0.1 seconds
v_robot = 0.1; % robot travels at a constant speed of 0.1 m/s

t_terminal = length_hallway/v_robot; % time that we reach the end of hall

t_vector = 0:delta_t:t_terminal; % time vector for robot's trajectory
x_vector = 0:v_robot*delta_t:length_hallway; % robot's true position in time
v_vector = ones(1,length(t_vector)).*v_robot;  % robot's true velocity in time

num_states = length(x_vector); % number of discrete-time states in trajectory

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gausian error in Odometry
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mean = 0;   % mean
t = t_vector;
%%
% Deliverable 3
clc

% Storing calculated values in these vectors for plotting
XX = zeros(1,1001);
tt = zeros(1,1001);
xx_predicted = zeros(2,1001);
xx = zeros(2,1001);
PP = zeros(2,2002);
yy = zeros(2,1001);
AE = zeros(1,1000);

for i = 1:1000
    X = 0;
    dt = 0.1; % time step
    u = 0.1; %speed/control = 0.1m/s
    
    %Kalman Filter variables
    x_measured = [u; X];      %state vector
    A = [1 0; 0.1 1];    %state transition matrix
    P_Measured = [1 0; 0 0.0001];    %std_dev*std_dev = 0.1*0.1
    Q = [1*10^-8, 0; 0, 0];      %process noise covariance matrix
    R = 0.01;       %Sensor noise covariance matrix
    H_k = [1 0; 0 1];
    w = 0.01*randn(1001); %Q
    w(:,1) =0;
    
    for t=1:1001
        %simulating the System
        X = X + u*dt + w(t); % New Current State
        Z_K = [0.1; X]; % Measurements
        
        %Prediction Step
        x_predicted = A*x_measured;         %predicting the new state (mean)
        P_Predicted = A * P_Measured * A' + Q;    %predicting the new uncertainity (covariance)
        
        %Update Step
        K = P_Predicted*H_k'*inv(H_k*P_Predicted*H_k' + R);
        x_measured = x_predicted + K*(Z_K -  H_k*x_predicted); %final corrected state
        P_Measured = P_Predicted - K * H_k* P_Predicted;       %uncertainity in corrected state
        
        %Saving the outputs
        xx_predicted(:,t) = x_predicted;
        xx(:,t) = x_measured;
        PP(:,t*2+1:t*2+2) = P_Measured;
        XX(t) = X;
        tt(t) = t;
        yy(:,t) = Z_K;        
    end
    AE(i,:) = abs(x_vector(t)-xx_predicted(2,t));
end

MAE = sum(AE,1)/i;
figure, plot(tt,XX,tt,xx(2,:))
title(' 1D Kalman Filter'), xlabel('Time'), ylabel('Position')
legend('Ground Truth','Measured State')
figure, plot(MAE)
