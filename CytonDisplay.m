classdef CytonDisplay < hgsetget
    % Class for displaying Cyton Robot
    %
    % 2011-04-13 Armiger Created
    properties
    end
    properties (Access = private)
        hCyton;  % Handle to Parent Cyton Robot
        hg; % handle graphics
        hTimer;
    end
    methods
        function obj = CytonDisplay(hCyton)
            obj.hCyton = hCyton;
        end
        function start(obj)
            if strcmpi(obj.hTimer.Running,'off')
                start(obj.hTimer);
            end
        end
        function stop(obj)
            if ~isempty(obj.hTimer) && strcmpi(obj.hTimer.Running,'on')
                stop(obj.hTimer);
            end
        end
        function close(obj)
            stop(obj);
            if ishandle(obj.hg.Figure)
                delete(obj.hg.Figure);
            end
        end
        function initialize(obj,hParentAxes)
            % This function initializes the display on the axes provided
            % otherwise creates a new figure for display
            import Presentation.CytonI.*
            
            % Load surfaces
            patchData = obj.quickLoadPatchData();
            
            if nargin < 2
                % Draw Objects
                obj.hg.Figure = UiTools.create_figure('CytonVIE','CytonVIE');
                set(obj.hg.Figure,'CloseRequestFcn',@(src,evt)close(obj));
                %set(obj.hg.Figure,'MenuBar','figure');
                set(obj.hg.Figure,'ToolBar','figure');
                
                hParentAxes = axes('Parent',obj.hg.Figure);
                
            end
            
            obj.hg.hAxes = hParentAxes;
            
            [obj.hg.Transforms obj.hg.Patches obj.hg.Triads] = drawCyton(obj.hg.hAxes,patchData);
            
            axis(obj.hg.hAxes,'equal');
            
            axis(obj.hg.hAxes,[-300 300 -300 300 -100 600])
            view(obj.hg.hAxes,15,15)
            
            obj.hg.hLight = light('Parent',obj.hg.hAxes);
            camlight(obj.hg.hLight,'left');
            
            obj.hg.hTarget = hgtransform('Parent',obj.hg.hAxes);
            sphereRadius = 15;
            obj.hg.hSphere = Utils.plotSphere([0 0 0]',sphereRadius,'Parent',obj.hg.hTarget);
            obj.hg.hTriad = Utils.plot_triad(obj.hg.hTarget,eye(4),100);
            
            % create two bird points
            obj.hg.Bird1 = hgtransform('Parent',obj.hg.hAxes);
            obj.hg.hBird1 = Utils.plotSphere([0 0 0]',10, 'Parent',obj.hg.Bird1, 'facecolor', 'm', 'edgecolor','none');
            obj.hg.Bird2 = hgtransform('Parent',obj.hg.hAxes);
            obj.hg.hBird2 = Utils.plotSphere([0 0 0]',10, 'Parent',obj.hg.Bird2, 'facecolor', 'm', 'edgecolor','none');
            
%             obj.hg.hCylinder = hgtransform('Parent',obj.hg.hAxes);
%             [X,Y,Z] = cylinder;
%             surf('Parent',obj.hg.hCylinder,X,Y,Z); % create a cylinder
            
            obj.hTimer = UiTools.create_timer(mfilename,@(src,evt)updateFigure(obj));
            obj.hTimer.Period = 0.1;
            
            obj.hg.pbRun = uicontrol(...
                'Parent',obj.hg.Figure,...
                'Style','togglebutton',...
                'String','Run',...
                'Value', 1);
            
            cb = @(src,evt)get(obj.hg.pbRun,'Value') ...
                && start(obj.hCyton.hPlant) ...
                || stop(obj.hCyton.hPlant);
            set(obj.hg.pbRun,'Callback',cb)
            
        end
        function showTriads(obj,Value)
            % Turn on/off Triad display
            if ~ishandle(obj.hg.Figure)
                return
            end
            
            if Value
                set(obj.hg.Triads,'Visible','on');
            else
                set(obj.hg.Triads,'Visible','off');
            end
        end
        function showSurfaces(obj,Value)
            % Turn on/off Surface display
            if ~ishandle(obj.hg.Figure)
                return
            end
            
            if Value
                set(obj.hg.Patches,'Visible','on');
            else
                set(obj.hg.Patches,'Visible','off');
            end
        end
        function showTarget(obj,Value)
            % Turn on/off Target display
            if ~ishandle(obj.hg.Figure)
                return
            end
            
            if Value
                set(allchild(obj.hg.hTarget),'Visible','on');
            else
                set(allchild(obj.hg.hTarget),'Visible','off');
            end
            
        end
        function setTarget(obj,M)
            % Create a target and triad at the specified location in the
            % virtual environment
            %
            % Usage:
            %   setTarget(obj,[-100 150 280]);
            %   
            %   setTarget(obj,makehgtform('yrotate',0.3,'translate',[100 100 100]))
            
            if isequal(size(M(:)),[3 1])
                % convert vector to matrix
                M = makehgtform('translate',M);
            end
            
            if ishandle(obj.hg.Figure)
                set(obj.hg.hTarget,'Matrix',M);
            end
        end
                function setBird1(obj,M)
            % Create a sphere at the specified location in the
            % virtual environment
            %
            % Usage:
            %   setTarget(obj,[-100 150 280]);
            %   
            %   setTarget(obj,makehgtform('yrotate',0.3,'translate',[100 100 100]))
            
            if isequal(size(M(:)),[3 1])
                % convert vector to matrix
                M = makehgtform('translate',M);
            end
            
            if ishandle(obj.hg.Figure)
                set(obj.hg.Bird1,'Matrix',M);
            end
                end
                        function setBird2(obj,M)
            % Create a sphere at the specified location in the
            % virtual environment
            %
            % Usage:
            %   setTarget(obj,[-100 150 280]);
            %   
            %   setTarget(obj,makehgtform('yrotate',0.3,'translate',[100 100 100]))
            
            if isequal(size(M(:)),[3 1])
                % convert vector to matrix
                M = makehgtform('translate',M);
            end
            
            if ishandle(obj.hg.Figure)
                set(obj.hg.Bird2,'Matrix',M);
            end
        end
%         function setCylinder(obj,pos1, pos2)
%             % Create a cylinder given two end points
%             % pos1 is length(3)
%             if ishandle(obj.hg.Figure)
%                 [x y z] = cylinder; % get the unit cylinder
% 
%                 % transform and scale to the segment between pos1 and 2
%                 segment = pos1 - pos2;
%                 % scale to length
%                 z = z*norm(segment);
%                 x = x*10;
%                 y = y*10;
%                 
%                 % rotate
%                 unitVec = segment/norm(segment);
%                 A = makehgtform(...
%                     'zrotate', atan2(unitVec(2),unitVec(1)),...
%                     'yrotate', atan2(sqrt(norm(unitVec(1:2))),unitVec(3)));
%                                 A = makehgtform(...
%                     'xrotate', pi/2);
%                 for i = 1:numel(x)
%                     temp = A*[
%                         1 0 0 x(i)
%                         0 1 0 y(i)
%                         0 0 1 z(i)
%                         0 0 0 1];
%                     x(i) = temp(1,4);
%                     y(i) = temp(2,4);
%                     z(i) = temp(3,4);
%                     
%                 end
%                 % translate
%                 x_ = x + pos2(1);
%                 y_ = y + pos2(2);
%                 z_ = z + pos2(3);
%                 
%                 set(obj.hg.hCylinder,'xdata', x);
%                 set(obj.hg.hCylinder,'ydata', y);
%                 set(obj.hg.hCylinder,'zdata', z);
%             end
%         end
        function hSphere = setTargetSphereRadius(obj,r)
            if ~ishandle(obj.hg.Figure)
                obj.hg.hSphere = [];
                return
            end
            
            assert(ishandle(obj.hg.hTarget),'Error target missing');
            
            if ishandle(obj.hg.hSphere)
                delete(obj.hg.hSphere);
                obj.hg.hSphere = [];
            end
            
            sphereRadius = r;
            obj.hg.hSphere = Presentation.CytonI.Utils.plotSphere([0 0 0]',sphereRadius,'Parent',obj.hg.hTarget);
            hSphere = obj.hg.hSphere;

        end
        function updateFigure(obj)
            % Set the graphic hgtransform objects to reflect the kinematics
            
            if ~ishandle(obj.hg.Figure)
                return
            end
            
            hgT = obj.hg.Transforms;
            
            A = obj.hCyton.hControls.getJointFrames();
            
            for i = 1:length(A)
                set(hgT(i),'Matrix',A(:,:,i));
            end
            
            if obj.hCyton.hPlant.isRunning
                title(obj.hg.hAxes,'Plant Running...');
            else
                title(obj.hg.hAxes,'Plant Stopped');
            end
            
        end
    end
    methods (Static = true)
        function patchData = quickLoadPatchData()
            % Check for the existance of a binary Matlab -mat file, if so
            % load it.  If not, load the STL data and save the patch data
            % at the end
            import Presentation.CytonI.*
            matFile = fullfile(fileparts(which(mfilename)),'CytonIPatchData.mat');
            
            if exist(matFile,'file')
                load(matFile);
                return
            else
                patchData = CytonDisplay.loadPatchData();
                disp('Saving Patch Data for Faster Load');
                save(matFile,'patchData');
            end
        end
        function patchData = loadPatchData()
            error('Obsolete.  Use quickLoadPatchData()')
            
            % Load Cyton Data
            % The struct patchData contains the faces and vertices of the patch data
            
            % read STL files
            dirName = fullfile(fileparts(which(mfilename)),'Cyton_STLb');
            
            a = dir(fullfile(dirName,'*.STL'));
            if length(a) < 1
                fprintf(2,'No STL files found in %s',dirName);
                return
            end
            
            mdlDat = struct('faces',{},'vertices',{});
            patchData = struct('faces',{},'vertices',{});
            partName = cell(size(a));
            for i = 1:length(a)
                partName{i} = a(i).name;
                [stlMesh msg] = Utils.readStl(fullfile(dirName,partName{i}));
                if isempty(stlMesh)
                    error(msg);
                end
                
                % extract patch faces and vertices
                p = Utils.stlMesh_to_patch(stlMesh);
                patchReductionFactor = 0.5; % reduces #faces by 50%
                mdlDat(i) = reducepatch(p,patchReductionFactor);
            end
            
            % create kinematic graphics
            % Transform each segment into it's D-H local coordinate system
            
            T_0_n = CytonControls.getDHParams();
            
            for i = 1:length(T_0_n)
                patchData(i) = mdlDat(i);
                
                R = T_0_n(1:3,1:3,i);
                p = T_0_n(1:3,4,i);
                b = patchData(i).vertices';
                d = bsxfun(@minus,R'*b,R'*p); %% Same as d = R * b + p, but works for hPatches = [3 n]
                patchData(i).vertices = d';
            end
        end
    end
end
