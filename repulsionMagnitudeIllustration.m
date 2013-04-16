%illustration of Magnitude of Velocity Blend for Avoidance
clear;
repelDist = 0:500; % domain for illustration
maximumRepelDistance = 350;
closestAllowedApproach = 200;
timeStep = 10; % magnitude of the control velocity command

i = repelDist > maximumRepelDistance;

repelMag = (maximumRepelDistance-repelDist).^2*timeStep/(maximumRepelDistance-closestAllowedApproach)^2;
repelMag(i) = 0; % set to zero

figure;hold on;
title('Repulsion Magnitude')
xlabel('Repulsion Distance')
plot(repelDist, repelMag, 'm', 'linewidth', 3);

plot([closestAllowedApproach closestAllowedApproach 0 ],[0 timeStep timeStep], 'k-');


