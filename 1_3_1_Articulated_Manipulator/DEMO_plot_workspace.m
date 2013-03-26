% setup figure
[h, tableHandle] = Fig_1_9;   
hold on

% create a handle to a 'trail' that will show where the robot has been
hTrail = plot3(0,0,0,'k.');
axis([-10 10 -10 10 -2 15])
view(110,20)

% loop through each joint range of motion
for theta3 = linspace(-45,90,5);
    for theta2 = linspace(0,90,5);
        for theta1 = linspace(0,360,15)
            % update robot
            T_0_n = Fig_1_9_dof_update(h,deg2rad(theta1),deg2rad(theta2),deg2rad(theta3),tableHandle);
            
            % update trail
            set(hTrail,'XData',[get(hTrail,'XData') T_0_n(1,4)]);
            set(hTrail,'YData',[get(hTrail,'YData') T_0_n(2,4)]);
            set(hTrail,'ZData',[get(hTrail,'ZData') T_0_n(3,4)]);
            drawnow
        end
    end
end

% return to home
T_0_n = Fig_1_9_dof_update(h,pi/2,0,0,tableHandle);
view(90,0)
return

%%
delete(hTrail)
%%

theta1 = 90;
theta2 = 0;
theta3 = 0;
T_0_n = Fig_1_9_dof_update(h,deg2rad(theta1),deg2rad(theta2),deg2rad(theta3));
view(90,0)

