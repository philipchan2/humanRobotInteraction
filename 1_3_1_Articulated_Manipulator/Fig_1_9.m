function [h tableHandle]= Fig_1_9

% assign constants X=1 Y=2 Z=3
[X,Y,Z] = deal(1,2,3);

% create a set of points for a 1x1x2 cylinder
scale = [1; 1; 2;];
[xData,yData,zData] = cylinder(1);
xData = xData.*scale(X);
yData = yData.*scale(Y);
zData = zData.*scale(Z) - scale(Z)./2;
fvc = surf2patch(xData,yData,zData);

% Create all the plot objects we'll need (cylinders, coordinate frames,
% text labels)

% first draw the cylinders
clf
p(1) = patch(fvc);
p(2) = patch(fvc);
p(3) = patch(fvc);

% next draw the coordinate frames
t(1) = f_plot_triad(eye(4),2);  % GCS
t(2) = f_plot_triad(eye(4),2);  % x0y0z0
t(3) = f_plot_triad(eye(4),2);  % x1y1z1
t(4) = f_plot_triad(eye(4),2);  % x2y2z2
t(5) = f_plot_triad(eye(4),2);  % x3y3z3

% next draw the text
cs_text(1) = text(1,1,0,'A0');
cs_text(2) = text(1,1,0,'A1');
cs_text(3) = text(1,1,0,'A2');
cs_text(4) = text(1,1,0,'A3');

% create MATLAB transformation containers and assign hierarchy
h(1) = hgtransform('Parent',gca);
h(2) = hgtransform('Parent',h(1));
h(3) = hgtransform('Parent',h(2));
h(4) = hgtransform('Parent',h(3));

% assign parents to all the plot objects we made
set(p(1),'Parent',h(1));
set(p(2),'Parent',h(2));
set(p(3),'Parent',h(3));

set(t(1),'Parent',h(1));
set(t(2),'Parent',h(2));
set(t(3),'Parent',h(3));
set(t(4),'Parent',h(4));

set(cs_text(1),'Parent',h(1));
set(cs_text(2),'Parent',h(2));
set(cs_text(3),'Parent',h(3));
set(cs_text(4),'Parent',h(4));

% assign some different colors
set(p(1),'FaceColor','cyan')
set(p(2),'FaceColor','green')
set(p(3),'FaceColor','red')

% setup the figure how we need it
daspect([1 1 1])
axis([-2 10 -2 10 -2 10])
view(110,25)
camlight
xlabel('X-axis')
ylabel('Y-axis')
zlabel('Z-axis')

%% Add displays
% skrink the current axes to make room for the displays
% make room at the bottom
displayBottomPortion = .2;
set(gca,'outerposition', [0 displayBottomPortion 1 1-displayBottomPortion]);

% create a table at the bottom
toolHorizontalPortion = .2;

% Tool origin
rnames = {'X' 'Y' 'Z'};
tableHandle(1) = uitable('RowName',rnames,...
    'columnName', {'Tool'},...
    'units', 'normalized',...
    'Position',[0 0 toolHorizontalPortion displayBottomPortion] ...
    );

% current joint angles
rnames = { '1' '2' '3'};
tableHandle(2) = uitable('RowName',rnames,...
    'columnName', {'Joint Angles'},...
    'units', 'normalized',...
    'Position',[toolHorizontalPortion 0 toolHorizontalPortion displayBottomPortion] ...
    );

% set the slider titles

uicontrol('style', 'text', ...
    'units', 'normalized',...
    'string', 'Joint 1',...
    'Position',[2*toolHorizontalPortion displayBottomPortion*2/3 toolHorizontalPortion displayBottomPortion/3] ...
    );
uicontrol('style', 'text', ...
    'units', 'normalized',...
    'string', 'Joint 2',...
    'Position',[2*toolHorizontalPortion displayBottomPortion*1/3 toolHorizontalPortion displayBottomPortion/3] ...
    );
uicontrol('style', 'text', ...
    'units', 'normalized',...
    'string', 'Joint 3',...
    'Position',[2*toolHorizontalPortion displayBottomPortion*0/3 toolHorizontalPortion displayBottomPortion/3] ...
    );


%% add slider to control for joint angles
tableHandle(3) = uicontrol('style', 'slider', ...
    'units', 'normalized',...
    'min', 0,'max', 360,...
    'Position',[3*toolHorizontalPortion displayBottomPortion*2/3 1-3*toolHorizontalPortion displayBottomPortion/3] ...
);
tableHandle(4) = uicontrol('style', 'slider', ...
    'units', 'normalized',...
    'min', -180,'max', 180,...
    'Position',[3*toolHorizontalPortion displayBottomPortion*1/3 1-3*toolHorizontalPortion displayBottomPortion/3] ...
    );
tableHandle(5) = uicontrol('style', 'slider', ...
    'units', 'normalized',...
    'min', -180,'max', 180,...
    'Position',[3*toolHorizontalPortion displayBottomPortion*0/3 1-3*toolHorizontalPortion displayBottomPortion/3] ...
    );
% set callbacks
set(tableHandle(3),'callback', {@updateFigFromSlider, h, tableHandle});
set(tableHandle(4),'callback', {@updateFigFromSlider, h, tableHandle});
set(tableHandle(5),'callback', {@updateFigFromSlider, h, tableHandle});

set(gcf, 'toolbar', 'figure'); % reappear the toolbar
end
%%  slider call backs
function updateFigFromSlider(hObj,event,h, tableHandle) %#ok<INUSL>
    % Called when the Joint 1 slider is updated 
    % Updates the figure
    theta1 = get(tableHandle(3),'Value');
    theta2 = get(tableHandle(4),'Value');
    theta3 = get(tableHandle(5),'Value');
    
    T_0_n = Fig_1_9_dof_update(h,deg2rad(theta1),deg2rad(theta2),deg2rad(theta3),tableHandle);

end

