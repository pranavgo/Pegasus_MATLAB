family = 'modules';
names_hip = {'right_hip','left_hip'};
% names_thigh = {'right_thigh','left_thigh'};
% names_calf = {'right_calf' ,'left_calf'};
names_rleg = {'right_thigh','right_calf'};
names_lleg = {'left_thigh' ,'left_calf'};
names_thigh_wheel = {'right_thigh_wheel' ,'left_thigh_wheel'};
names_wheel = {'right_wheel','left_wheel'};
group_hip = HebiLookup.newGroupFromNames(family, names_hip);
group_rleg = HebiLookup.newGroupFromNames(family, names_rleg);
group_lleg = HebiLookup.newGroupFromNames(family, names_lleg);
group_thigh_wheel = HebiLookup.newGroupFromNames(family, names_thigh_wheel);
group_wheel = HebiLookup.newGroupFromNames(family, names_wheel);

l1 = 0.213;
l2 = 0.208;
l3 = 0.321;
step_length = 0.1;

th = linspace( pi, 0, 200);
R = step_length/2;  %or whatever radius you want
x = R*cos(th) - 0.01;
z = R*sin(th) - 0.31006; %32.3*cos(15)

w = (step_length)/0.22; %radius of the wheel is 11cms
cmd = CommandStruct();
vel_cmd = CommandStruct();
while true
    for i = 1:length(x)        
        cmd.position = [0 0];
        group_hip.send(cmd);
        cmd.position = [-pi/4 pi/4]; % 20 degrees(5 degrres more to keep the com backwards)
        group_thigh_wheel.send(cmd);
        
        vel_cmd.velocity = [-w, w];
        group_wheel.send(vel_cmd);
        
        [theta1, theta2] = IK(l1, l2, 0, 0, x(i), z(i));
        cmd.position = [-theta1, -theta2];
        group_rleg.send(cmd);
        pause(0.005);
    end   
    
    for i = 1:length(x)
        cmd.position = [0 0];
        group_hip.send(cmd);
        cmd.position = [-pi/4 pi/4]; % 20 degrees
        group_thigh_wheel.send(cmd);
        
        vel_cmd.velocity = [-w, w];
        group_wheel.send(vel_cmd);

        [theta1, theta2] = IK(l1, l2, 0, 0, x(i), z(i));
        cmd.position = [theta1, theta2];
        group_lleg.send(cmd);
        pause(0.005);
    end    
end