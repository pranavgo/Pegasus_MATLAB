l1 = 0.213;
l2 = 0.211;
hip_height = 0.2899;
step_length = 0.2;
step_height_bezier = 0.1;

swing_foot = bezier.eval([-step_length/2, -hip_height; 0, -hip_height + step_height_bezier; step_length/2, -hip_height]);
stand_foot = [0, -hip_height];
xf_stand = stand_foot(1);
zf_stand = stand_foot(2);
hip = [linspace(-step_length/4, step_length/4, length(swing_foot))', linspace(0, 0, length(swing_foot))'];
theta1_swing_array = [];
for i = 1:length(swing_foot)
    xh = hip(i, 1);
    zh = hip(i, 2);
    
    xf_swing = swing_foot(i, 1);
    zf_swing = swing_foot(i, 2);
    [theta1_swing, theta2_swing] = IK(l1, l2, xh, zh, xf_swing, zf_swing);
    [theta1_stand, theta2_stand] = IK(l1, l2, xh, zh, xf_stand, zf_stand);
    theta1_swing_array = [theta1_swing_array, theta1_swing];
    
    plot([xh, xh + l1 * sin(theta1_swing)], [zh, zh - l1 * cos(theta1_swing)]), hold on
    plot([xh + l1 * sin(theta1_swing), xh + l1 * sin(theta1_swing) + l2 * sin(theta1_swing+theta2_swing)], [zh - l1 * cos(theta1_swing), zh - l1 * cos(theta1_swing) - l2 * cos(theta1_swing+theta2_swing)])
    
    plot([xh, xh + l1 * sin(theta1_stand)], [zh, zh - l1 * cos(theta1_stand)]), hold on
    plot([xh + l1 * sin(theta1_stand), xh + l1 * sin(theta1_stand) + l2 * sin(theta1_stand+theta2_stand)], [zh - l1 * cos(theta1_stand), zh - l1 * cos(theta1_stand) - l2 * cos(theta1_stand+theta2_stand)])
    
    axis equal
    grid on
end
