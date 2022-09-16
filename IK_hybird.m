l1 = 0.213;
l2 = 0.211;
hip_height = -0.2899;
step_length = 0.06;

th = linspace( pi, 0, 100);
R = step_length/2;  %or whatever radius you want
x = R*cos(th) + 0;
z = R*sin(th) - 0.2899;
x(end+1) = 0;
z(end + 1) = - 0.2899;
theta1_array =[];
theta2_array = [];
    for i = 1:length(x)
        [theta1, theta2] = IK(l1, l2, 0, 0, x(i), z(i));
        theta1_array = [theta1_array,theta1];
        theta2_array = [theta2_array,theta2];
        plot(theta1,i), hold on
        plot (theta2,1), hold on
        axis equal
        grid on
    end