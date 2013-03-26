% setup figure
figure(1);
clf;
[h, tableHandle] = Fig_1_9;
hold on
view(90,0)

% define three joint angles and update plot
%% enter different theta values to move robot
theta1 = 45;
theta2 = 50;
theta3 = 45;
EndEffectorFrame = Fig_1_9_dof_update(h,deg2rad(theta1),deg2rad(theta2),deg2rad(theta3),tableHandle);

