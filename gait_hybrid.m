close, clear, clc
%% initialize kinematics parameters
l1 = 0.213;
l2 = 0.208;

l_rear = 0.211;
r_wheel = 0.11;

hip_height = 0.3096;

step_length = 0.6;
step_height_bezier = 0.3;

step_points = 300;
phase = 180;
stance_points = 500;
%% initialize swing foot trajectory
swing_foot_traj = bezier.eval([-step_length/2, -hip_height;...
    0, -hip_height + step_height_bezier;...
    step_length/2, -hip_height], step_points);

%% initialize key positions
stand_foot = [0, -hip_height];
% xf_stand = stand_foot(1);
zf_stand = stand_foot(2);

xf_stand = linspace(0, 0, stance_points)';

hip1 = [linspace(-step_length/4, step_length/4, stance_points)', linspace(0, 0, stance_points)'];
hip2 = [linspace(-step_length/4, step_length/4, step_points)', linspace(0, 0, step_points)'];
%% generate joint position trajectory
theta1_swing_traj = zeros(length(swing_foot_traj), 1);
theta2_swing_traj = zeros(length(swing_foot_traj), 1);

theta1_stand_traj = zeros(stance_points, 1);
theta2_stand_traj = zeros(stance_points, 1);

for i = 1:length(swing_foot_traj)
    xh = hip2(i, 1);
    zh = hip2(i, 2);
    xf_swing = swing_foot_traj(i, 1);
    zf_swing = swing_foot_traj(i, 2);
    
    [theta1_swing, theta2_swing] = IK(l1, l2, 0, 0, xf_swing, zf_swing);
    
    theta1_swing_traj(i) = theta1_swing;
    theta2_swing_traj(i) = theta2_swing;

    
    plot([xh, xh + l1 * sin(theta1_swing)], [zh, zh - l1 * cos(theta1_swing)]), hold on
    plot([xh + l1 * sin(theta1_swing), xh + l1 * sin(theta1_swing) + l2 * sin(theta1_swing+theta2_swing)], [zh - l1 * cos(theta1_swing), zh - l1 * cos(theta1_swing) - l2 * cos(theta1_swing+theta2_swing)])
   
    axis equal
    grid on
end
for i = 1:stance_points
    xh_stand = hip1(i, 1);
    zh_stand = hip1(i, 2);
    
    [theta1_stand, theta2_stand] = IK(l1, l2, xh_stand, zh_stand, xf_stand(i), zf_stand);
    
    theta1_stand_traj(i) = theta1_stand;
    theta2_stand_traj(i) = theta2_stand;
    
    plot([xh_stand, xh_stand + l1 * sin(theta1_stand)], [zh_stand, zh_stand - l1 * cos(theta1_stand)]), hold on
    plot([xh_stand + l1 * sin(theta1_stand), xh_stand + l1 * sin(theta1_stand) + l2 * sin(theta1_stand+theta2_stand)], [zh_stand - l1 * cos(theta1_stand), zh_stand - l1 * cos(theta1_stand) - l2 * cos(theta1_stand+theta2_stand)])
    
    axis equal
    grid on
end

%% initializing trajectory

theta1_full = cat(1,theta1_swing_traj, theta1_stand_traj);
theta2_full = cat(1,theta2_swing_traj, theta2_stand_traj);
% p = (2*step_points)/360;
% shift = round(p*phase);

left_thigh = theta1_full;
left_calf = theta2_full;

% right_thigh = circshift(theta1_full,shift);
% right_calf = circshift(theta2_full,shift);

right_thigh = cat(1,theta1_stand_traj, theta1_swing_traj);
right_calf =  cat(1,theta2_stand_traj, theta2_swing_traj);


%% hebi init
family = 'modules';

names_leg_ctrl = {'left_hip', 'left_thigh', 'left_calf', 'left_thigh_wheel', ...
         'right_hip', 'right_thigh', 'right_calf', 'right_thigh_wheel'};
group_leg_ctrl = HebiLookup.newGroupFromNames(family, names_leg_ctrl);

names_wheel_ctrl = {'left_wheel', 'right_wheel'};
group_wheel_ctrl = HebiLookup.newGroupFromNames(family, names_wheel_ctrl);

leg_cmd = CommandStruct();
wheel_cmd = CommandStruct();

%% walk control
pause_time = 0.0025;

body_vel = (step_length/2)/(pause_time * (stance_points + step_points));
wheel_omega = body_vel / r_wheel;


while true
    for i = 2:(step_points + stance_points)
        
        leg_cmd.position = [-0.2, -left_thigh(i), left_calf(i), pi/5 ...
                    0.2, right_thigh(i), -right_calf(i), -pi/5];
        wheel_cmd.velocity = [wheel_omega, -wheel_omega];

        group_leg_ctrl.send(leg_cmd);
        group_wheel_ctrl.send(wheel_cmd);

        pause(pause_time);
    end
end
