close, clear, clc

%% initialize kinematics parameters
l1 = 0.213;
l2 = 0.208;

l_rear = 0.211;
r_wheel = 0.11;

hip_height = 0.3096;

step_length = 0.4;
step_height_bezier = 0.30;

%% initialize swing foot trajectory
swing_foot_traj = bezier.eval([-step_length/2, -hip_height;...
    0, -hip_height + step_height_bezier;...
    step_length/2, -hip_height], 300);

%% initialize key positions
stand_foot = [0, -hip_height];
% xf_stand = stand_foot(1);
zf_stand = stand_foot(2);
xf_stand = linspace(0, 0, length(swing_foot_traj))';
hip = [linspace(-step_length/4, step_length/4, length(swing_foot_traj))', linspace(0, 0, length(swing_foot_traj))'];

%% generate joint position trajectory
theta1_swing_traj = zeros(length(swing_foot_traj), 1);
theta2_swing_traj = zeros(length(swing_foot_traj), 1);

theta1_stand_traj = zeros(length(swing_foot_traj), 1);
theta2_stand_traj = zeros(length(swing_foot_traj), 1);

for i = 1:length(swing_foot_traj)
    xh = hip(i, 1);
    zh = hip(i, 2);
    
    xf_swing = swing_foot_traj(i, 1);
    zf_swing = swing_foot_traj(i, 2);
    
    [theta1_swing, theta2_swing] = IK(l1, l2, xh, zh, xf_swing, zf_swing);
    [theta1_stand, theta2_stand] = IK(l1, l2, xh, zh, xf_stand(i), zf_stand);
    
    theta1_swing_traj(i) = theta1_swing;
    theta2_swing_traj(i) = theta2_swing;
    theta1_stand_traj(i) = theta1_stand;
    theta2_stand_traj(i) = theta2_stand;
    
    plot([xh, xh + l1 * sin(theta1_swing)], [zh, zh - l1 * cos(theta1_swing)]), hold on
    plot([xh + l1 * sin(theta1_swing), xh + l1 * sin(theta1_swing) + l2 * sin(theta1_swing+theta2_swing)], [zh - l1 * cos(theta1_swing), zh - l1 * cos(theta1_swing) - l2 * cos(theta1_swing+theta2_swing)])
    
    plot([xh, xh + l1 * sin(theta1_stand)], [zh, zh - l1 * cos(theta1_stand)]), hold on
    plot([xh + l1 * sin(theta1_stand), xh + l1 * sin(theta1_stand) + l2 * sin(theta1_stand+theta2_stand)], [zh - l1 * cos(theta1_stand), zh - l1 * cos(theta1_stand) - l2 * cos(theta1_stand+theta2_stand)])
    
    axis equal
    grid on
end

% figure(2);
% plot(theta1_swing_traj), hold on;
% plot(theta1_stand_traj);
% 
% figure(3);
% plot(theta2_swing_traj), hold on;
% plot(theta2_stand_traj);

%% hebi init
family = 'modules';

names_right_leg_ctrl = {'left_hip', 'left_thigh', 'left_calf', 'left_thigh_wheel'};
names_left_leg_ctrl = {'right_hip', 'right_thigh', 'right_calf', 'right_thigh_wheel'};
group_right_leg_ctrl = HebiLookup.newGroupFromNames(family, names_right_leg_ctrl);
group_left_leg_ctrl = HebiLookup.newGroupFromNames(family, names_left_leg_ctrl);
names_wheel_ctrl = {'left_wheel', 'right_wheel'};
group_wheel_ctrl = HebiLookup.newGroupFromNames(family, names_wheel_ctrl);

pause_time = 0.002;

stance_leg_cmd = CommandStruct();
swing_leg_cmd = CommandStruct();
wheel_cmd = CommandStruct();

%% walk control
stand_leg = 'r';

body_vel = step_length / 2 / (pause_time * length(swing_foot_traj));
wheel_omega = body_vel / r_wheel;

while true
    if stand_leg == 'r'
        for i = 2:length(swing_foot_traj)
            left_thigh = theta1_stand_traj(i);
            left_calf = theta2_stand_traj(i);

            right_thigh = theta1_swing_traj(i);
            right_calf = theta2_swing_traj(i);

            stance_leg_cmd.position = [-0.2, -left_thigh, left_calf, pi/6];
            swing_leg_cmd.position = [0.2, right_thigh, -right_calf, -pi/6];
            wheel_cmd.velocity = [wheel_omega, -wheel_omega];

            group_left_leg_ctrl.send(swing_leg_cmd);
            group_wheel_ctrl.send(wheel_cmd);
            group_right_leg_ctrl.send(stance_leg_cmd);
            pause(pause_time);
        
        
        end
        stand_leg = 'l';
    elseif stand_leg == 'l'
        for i = 2:length(swing_foot_traj)
            left_thigh = theta1_swing_traj(i);
            left_calf = theta2_swing_traj(i);

            right_thigh = theta1_stand_traj(i);
            right_calf = theta2_stand_traj(i);
            
            swing_leg_cmd.position = [-0.2, -left_thigh, left_calf, pi/6];
            stance_leg_cmd.position = [0.2, right_thigh, -right_calf, -pi/6];
            wheel_cmd.velocity = [wheel_omega, -wheel_omega];

            group_right_leg_ctrl.send(swing_leg_cmd);
            group_wheel_ctrl.send(wheel_cmd);
            group_left_leg_ctrl.send(stance_leg_cmd);
            pause(pause_time);
        end
        stand_leg = 'r';
    end
end