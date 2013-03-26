function T_0_n = Fig_1_9_dof_update(h,theta1,theta2,theta3,tableHandle)
% this gives access to the free parameters in our robot, all other
% parameters are constant (e.g. lengths)
%
% returns the transformation to the last link

A = get_kinematics(theta1,theta2,theta3);

set(h(2),'Matrix',A(:,:,1));
set(h(3),'Matrix',A(:,:,2));
set(h(4),'Matrix',A(:,:,3));

%% update the displays
% compute the tool origin
toolTransform = A(:,:,1)*A(:,:,2)*A(:,:,3);
data{1,1} = toolTransform(1,4);
data{2,1} = toolTransform(2,4);
data{3,1} = toolTransform(3,4);

% alarm for collision detection
% if the end effector is below the base frame z - 0, then beep
if toolTransform(3,4)<=0
    beep; % notify, but don't stop the VIE from moving
end

set(tableHandle(1),'data', data); % set the table origin display
data{1,1} = mod(rad2deg(theta1), 360);
data{2,1} = rad2deg(theta2);
data{3,1} = rad2deg(theta3);
set(tableHandle(2),'data', data); % set the joint angle display
% update the joint angle sliders
set(tableHandle(3),'value', data{1,1}); % set the joint angle display
set(tableHandle(4),'value', data{2,1}); % set the joint angle display
set(tableHandle(5),'value', data{3,1}); % set the joint angle display


%%


T_0_n = A(:,:,1)*A(:,:,2)*A(:,:,3);
