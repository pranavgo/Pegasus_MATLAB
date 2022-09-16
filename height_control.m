l1 = 0.213;
l2 = 0.208;

l_rear = 0.211;
r_wheel = 0.11;

hip_height = 0.3096;
base = 0.1525;
step_length = 0.5;
step_height_bezier = 0.2;
PI = 3.14159;
family = 'modules';
previous_error = 0;

names_leg_ctrl = {'left_hip', 'left_thigh', 'left_calf', 'left_thigh_wheel', ...
         'right_hip', 'right_thigh', 'right_calf', 'right_thigh_wheel'};
group_leg_ctrl = HebiLookup.newGroupFromNames(family, names_leg_ctrl);

% names_wheel_ctrl = {'left_wheel', 'right_wheel'};
% group_wheel_ctrl = HebiLookup.newGroupFromNames(family, names_wheel_ctrl);

leg_cmd = CommandStruct();
wheel_cmd = CommandStruct();

[theta1, theta2] = IK(l1,l2, 0, 0, 0, -hip_height);
back_left = pi/5;
back_right = - pi/5;
pitch_array = [];
leg_cmd.position = [-0.15, -theta1, theta2, back_left...
                 0.15, theta1, -theta2, back_right];
group_leg_ctrl.send(leg_cmd)               
group_leg_ctrl.send(leg_cmd)    
group_leg_ctrl.send(leg_cmd)    
group_leg_ctrl.send(leg_cmd)    
group_leg_ctrl.send(leg_cmd)    
group_leg_ctrl.send(leg_cmd) 
group_leg_ctrl.send(leg_cmd)    
group_leg_ctrl.send(leg_cmd) 

ifbk = group_leg_ctrl.getNextFeedbackFull();
qa = [ ifbk.orientationW(4), ...
      ifbk.orientationX(4), ...
      ifbk.orientationY(4), ...
      ifbk.orientationZ(4) ];
  
qb = [ ifbk.orientationW(8), ...
      ifbk.orientationX(8), ...
      ifbk.orientationY(8), ...
      ifbk.orientationZ(8) ];
[yawa,pitcha,rolla] = quat2angle(qa);
[yawb,pitchb,rollb] = quat2angle(qb);

pitcha = rad2deg(pitcha);
pitchb = rad2deg(pitchb);

angle = (pitcha+pitchb)/2;
while true
    fbk = group_leg_ctrl.getNextFeedbackFull();
    qa = [ fbk.orientationW(4), ...
          fbk.orientationX(4), ...
          fbk.orientationY(4), ...
          fbk.orientationZ(4) ];

    qb = [ fbk.orientationW(8), ...
          fbk.orientationX(8), ...
          fbk.orientationY(8), ...
          fbk.orientationZ(8) ];

   rawza = fbk.accelZ(1);
   
   [yawa,pitcha,rolla] = quat2angle(qa);
   [yawb,pitchb,rollb] = quat2angle(qb);   
   
   pitcha = rad2deg(pitcha);
   pitchb = rad2deg(pitchb);   
   
   pitch_array = [pitch_array pitcha];
   angle1 = (pitcha+pitchb)/2;
  
   error = angle1 - angle;
   dt_error = previous_error - error;
   
   if rawza > 0 
       error = -error;
   else
       error = error;
   end
   
   disp(error)
   if abs(error) < 45 && abs(dt_error)>3
        dheight = tand(error)*base;
        height = hip_height - dheight;
        [theta1, theta2] = IK(l1, l2, 0, 0, 0, height);
        back_left = pi/5 - deg2rad(error);
        back_right = -pi/5 + deg2rad(error);
        leg_cmd.position = [-0.15, -theta1, theta2, back_left ...
                        0.15, theta1, -theta2, back_right];
        group_leg_ctrl.send(leg_cmd);
        pause(0.05);
        prevous_error = error;
   else
       continue
   end
end

% for i=1:50
%     [theta1, theta2] = IK(l1, l2, 0, 0, 0, -hip_height);
%     back_left = pi/5;
%     back_right = -pi/5;
%     leg_cmd.position = [-0.2, -theta1, theta2, back_left ...
%                     0.2, theta1, -theta2, back_right];
%     fbk = group_leg_ctrl.getNextFeedback();
%     if abs(fbk.position - leg_cmd.position) > [0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001]
%         group_leg_ctrl.send(leg_cmd);
%         pause(0.01);
%     end
%     group_leg_ctrl.send(leg_cmd);
%     height  = -hip_height + i*0.001;
%     [theta1, theta2] = IK(l1, l2, 0, 0, 0, height);
%     back_left = pi/5 - i*0.01;
%     back_right = -pi/5 + i*0.01;
%     leg_cmd.position = [-0.2, -theta1, theta2, back_left ...
%                     0.2, theta1, -theta2, back_right];
%     fbk = group_leg_ctrl.getNextFeedback();
%     if abs(fbk.position - leg_cmd.position) > [0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001]
%         group_leg_ctrl.send(leg_cmd);
%         pause(0.01);
%     else
%         pause(2);
%         continue
%     end
%     [theta1, theta2] = IK(l1, l2, 0, 0, 0, -hip_height);
%     
%     back_left = pi/5;
%     back_right = -pi/5;
%     leg_cmd.position = [-0.2, -theta1, theta2, back_left ...
%                     0.2, theta1, -theta2, back_right];
%     fbk = group_leg_ctrl.getNextFeedback();
%     if abs(fbk.position - leg_cmd.position) > [0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001]
%         group_leg_ctrl.send(leg_cmd);
%         pause(0.01);
%     end
%     
%     height  = -hip_height - i*0.001;
%     [theta1, theta2] = IK(l1, l2, 0, 0, 0.01, height);
%     [theta1x, theta2x] = IK(l1, l2, 0, 0, -0.01, height);
%     back_left = pi/5 + i*0.01;
%     back_right = -pi/5 - i*0.01;
%     leg_cmd.position = [-0.2, -theta1, theta2, back_left ...
%                     0.2, theta1x, -theta2x, back_right];
%     fbk = group_leg_ctrl.getNextFeedback();
%     if abs(fbk.position - leg_cmd.position) > [0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001]
%         group_leg_ctrl.send(leg_cmd);
%         pause(0.01);
%     else
%         pause(2);
%         continue
%     end
% end