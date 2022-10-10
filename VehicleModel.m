function X = VehicleModel(v,w,previous_pose)
% Mimic real world respond
% Input argument
% v : foward speed
% w : rotation speed
% previous_pose : [X, Y, Theta]'
% 
% Output argument
% X : resulting pose

% Enviroment uncertainty parameters
c1=0.05;
c2=0.05;
c3=0.05;
c4=0.05;
del_t = 1;

% Disturb the actual input
r = [
    (c1*abs(v)+c2*abs(w))^2     0;
    0                           (c3*abs(v)+c4*abs(w))^2];
ran = mvnrnd([v w],r);
v_actual = ran(1);
w_actual = ran(2);

% Physical actual output
change_pose = [
    v_actual*del_t*cos(previous_pose(3)+w_actual*del_t);
    v_actual*del_t*sin(previous_pose(3)+w_actual*del_t);
    w_actual*del_t;
    ];
X = previous_pose + change_pose;
end