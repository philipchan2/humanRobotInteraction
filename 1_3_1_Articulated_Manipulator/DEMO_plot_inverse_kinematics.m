% whether to use the HW solution for IK
useIK_3_17 = 1;

% setup figure
figure(1)
clf
[h, tableHandle] = Fig_1_9;
hold on

% create a handle to a 'trail' that will show where the robot has been
hTrail = plot3(0,0,0,'k.');
axis([-10 10 -10 10 -2 15])
view(110,20)

% loop through each joint range of motion
for z = 3;
    for y = linspace(10,-10,10);
        for x = linspace(-5,4,20);
            % update robot
            p_0 = [x; y; z];
            
            if useIK_3_17
                theta = IK_3_17(p_0,[]); % no orientation specified
            else
                theta = IK(p_0);
            end
            
            T_0_n = Fig_1_9_dof_update(h,theta(1),theta(2),theta(3),tableHandle);
            
            % update trail
            set(hTrail,'XData',[get(hTrail,'XData') T_0_n(1,4)]);
            set(hTrail,'YData',[get(hTrail,'YData') T_0_n(2,4)]);
            set(hTrail,'ZData',[get(hTrail,'ZData') T_0_n(3,4)]);
            drawnow
        end
    end
end
