% script that runs the quad test

% parametes of the quad
Ixx = 0.2;
Iyy =  0.2; 
Izz = 0.5;

%simulation parameters
Td = 0.1;
sim_time = 1000;

%this runs the model
sim('quadrotor_model');


% this runs data visualization in 3D - comment if you only run the
% simulation without 3D visualization

data.x = [zeros(1,length(roll)) ; zeros(1,length(roll)); zeros(1,length(roll))];
data.theta = [roll(:,2)'; pitch(:,2)'; yaw(:,2)'];
data.vel = [zeros(1,length(roll)) ; zeros(1,length(roll)); zeros(1,length(roll)) ];
data.angvel = [roll_rate(:,2)' ; pitch_rate(:,2)' ; yaw_rate(:,2)'];
data.t =roll(:,1)';
data.input =[zeros(1,length(roll)) ; zeros(1,length(roll)); zeros(1,length(roll));zeros(1,length(roll)) ];
data.dt = 0.1;
  
visualize_test(data);


 