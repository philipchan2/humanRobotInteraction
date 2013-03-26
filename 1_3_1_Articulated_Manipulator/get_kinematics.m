function A = get_kinematics(theta1,theta2,theta3)

A = repmat(eye(4),[1 1 3]);

useHWhd_transform = 1; % whether to use the DH transform
if useHWhd_transform
    % Ai = DH_transformationHW(linkLength,linkTwist,linkOffset,jointAngle)
    A(:,:,1) = DH_transformationHW(0,pi/2,5,theta1);
    A(:,:,2) = DH_transformationHW(4,0,0,theta2);
    A(:,:,3) = DH_transformationHW(4,0,0,theta3);
else
    % Ai = DH_transformation(linkLength,linkTwist,linkOffset,jointAngle)
    A(:,:,1) = DH_transformation(0,pi/2,5,theta1)
    A(:,:,2) = DH_transformation(4,0,0,theta2)
    A(:,:,3) = DH_transformation(4,0,0,theta3)
end
