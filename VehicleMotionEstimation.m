function X = VehicleMotionEstimation(v,w,previous_pose)
% Mimic simulation world respond
% Input argument
% v : foward speed
% w : rotation speed
% previous_pose : [X, Y, Theta]'
% 
% Output argument
% X : resulting pose

del_t = 1;

v_actual = v;
w_actual = w;
% Physical actual output
change_pose = [
    v_actual*del_t*cos(previous_pose(3)+w_actual*del_t);
    v_actual*del_t*sin(previous_pose(3)+w_actual*del_t);
    w_actual*del_t;
    ];
X = previous_pose + change_pose;
end