function hTriad = f_plot_triad(F,scale)
% create an x,y,z triad with r,g,b colors
ax = gca;

xyz_color = {'r-','g-','b-'};

Xaxis = scale.*[0 1;0 0; 0 0];
Yaxis = scale.*[0 0;0 1; 0 0];
Zaxis = scale.*[0 0;0 0; 0 1];

holdState = ishold(ax);
hold(ax,'on');

hTriad = hgtransform('Parent',ax);

plot3(hTriad,Xaxis(1,:),Xaxis(2,:),Xaxis(3,:),xyz_color{1},'LineWidth',2);
plot3(hTriad,Yaxis(1,:),Yaxis(2,:),Yaxis(3,:),xyz_color{2},'LineWidth',2);
plot3(hTriad,Zaxis(1,:),Zaxis(2,:),Zaxis(3,:),xyz_color{3},'LineWidth',2);

set(hTriad,'Matrix',F);

% Restore hold state
if holdState
    hold(ax,'on');
else
    hold(ax,'off');
end
