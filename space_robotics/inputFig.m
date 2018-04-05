classdef inputFig < handle
    
    properties (Constant)
        boxHeight = 0.05;
        boxWidth  = 0.07;
        sliderWidth = 0.5;
        sliderHeight = 0.03;
        horMargin = 0.01;
        verMargin = 0.01;
        fontSize  = 12;
        
        posMinMax = [-2000 2000;...
                     -2000 2000;...
                     -2000 2000];
        jointMinMax = [-180 180;...
                       -180 180;...
                       -180 180;...
                       -180 180;...
                       -180 180;...
                       -180 180];
    end
    
    properties
        % Graphics handles
        fig;
        posInput = gobjects(3,1);
        rotTabGroup;
        tabRotMat;
        tabEulZyx;
        matInput = gobjects(3,3);
        zyxInput = gobjects(3,1);
        configSelect;
        goButton;
        posSlider = gobjects(3,1);
        posSliderBox = gobjects(3,1);
        jointSlider = gobjects(6,1);
        jointSliderBox = gobjects(6,1);
        
        % Other objects
        robotKin;
        udps;
        outFigure;
    end
    
    methods
        function self = inputFig(robotKin, udps, outFigure)
            self.robotKin = robotKin;
            self.udps = udps;
            self.outFigure = outFigure;
        end
        
        function createFigure(self)
            % close possible other instances of same figure
            hFigTemp = findobj(...
                'Type', 'Figure',...
                'Name', 'Robot Inputs');
            if ~isempty(hFigTemp)
                close(hFigTemp)
            end
            
            % current values for end effector position P and rotation C
            P = self.robotKin.getPos(6);
            C = self.robotKin.getRot(6);
            % current values for joint parameters
            q = self.robotKin.q;
            
            % create input figure
            self.fig = figure(...
                'Units',            'Normalized',...
                'OuterPosition',    [0.5, 0, 0.5, 0.5],...
                'Name',             'Robot Inputs',...
                'ToolBar',          'none',...
                'MenuBar',          'none',...
                'NumberTitle',      'off');
            
            %% Position input
%             uicontrol(...
%                 'Parent',           self.fig,...
%                 'Style',            'text',...
%                 'String',           'Position',...
%                 'Fontsize',         self.fontSize,...
%                 'Units',            'Normalized',...
%                 'Position',         [0, 1-self.boxHeight, 2*self.boxWidth+self.horMargin, self.boxHeight]);
            posStrings = {'x [mm]', 'y [mm]', 'z [mm]'};
            for ii = 1:3
                uicontrol(...
                    'Parent',           self.fig,...
                    'Style',            'text',...
                    'String',           posStrings{ii},...
                    'Fontsize',         self.fontSize,...
                    'Units',            'Normalized',...
                    'Position',         [0,...
                                         1-(ii+1)*self.boxHeight-ii*self.verMargin,...
                                         self.boxWidth,...
                                         self.boxHeight]);
                self.posInput(ii) = uicontrol(...
                    'Parent',           self.fig,...
                    'Style',            'edit',...
                    'String',           sprintf('%.2f', P(ii)),...
                    'Fontsize',         self.fontSize,...
                    'Units',            'Normalized',...
                    'Position',         [self.boxWidth+self.horMargin,...
                                         1-(ii+1)*self.boxHeight-ii*self.verMargin,...
                                         self.boxWidth,...
                                         self.boxHeight]);
            end
            lastPos = self.posInput(3).Position;
            
            %% Rotation input
            self.rotTabGroup = uitabgroup(...
                'Parent',               self.fig,...
                'TabLocation',          'Top',...
                'Position',             [lastPos(1)+lastPos(3)+self.horMargin,...
                                         lastPos(2)-0.008,...
                                         3*self.boxWidth,...
                                         1-lastPos(2)]);
            % Rotation matrix input
            self.tabRotMat = uitab(...
                'Parent',               self.rotTabGroup,...
                'Title',                'matrix');
            for ii = 1:3
                for jj = 1:3
                    self.matInput(ii,jj) = uicontrol(...
                        'Parent',           self.tabRotMat,...
                        'Style',            'edit',...
                        'String',           sprintf('%.2f', C(ii,jj)),...
                        'Fontsize',         self.fontSize,...
                        'Units',            'normalized',...
                        'Position',         [(jj-1)/3, 1-ii/3, 1/3, 1/3]);
                end
            end
            % Euler XYZ Input
            self.tabEulZyx = uitab(...
                'Parent',           self.rotTabGroup,...
                'Title',            'ZYX');
            rotString = {'Z [deg]', 'Y [deg]', 'X [deg]'};
            for ii = 1:3
                uicontrol(...
                    'Parent',           self.tabEulZyx,...
                    'Style',            'text',...
                    'String',           rotString{ii},...
                    'Fontsize',         self.fontSize,...
                    'Units',            'normalized',...
                    'Position',         [0, 1-ii/3, 1/2, 1/3]);
                self.zyxInput(ii) = uicontrol(...
                    'Parent',           self.tabEulZyx,...
                    'Style',            'edit',...
                    'String',           '0',...
                    'Fontsize',         self.fontSize,...
                    'Units',            'normalized',...
                    'Position',         [1/2, 1-ii/3, 1/2, 1/3]);
            end
            lastPos = self.rotTabGroup.Position;
            
            %% Configuration selector & 'GO-button'
            self.configSelect = uicontrol(...
                'Parent',           self.fig,...
                'Style',            'popupmenu',...
                'String',           cellfun(@num2str, num2cell(1:8),'UniformOutput',false),...
                'Value',            1,...
                'Fontsize',         self.fontSize,...
                'TooltipString',    'Joint configuration',...
                'Units',            'normalized',...
                'Position',         [lastPos(1)+lastPos(3)+self.horMargin,...
                                     1-2*self.boxHeight-self.verMargin,...
                                     self.boxWidth,...
                                     self.boxHeight]);
            self.goButton = uicontrol(...
                'Parent',           self.fig,...
                'Style',            'pushbutton',...
                'String',           'Go',...
                'Fontsize',         self.fontSize,...
                'Units',            'normalized',...
                'Position',         [lastPos(1)+lastPos(3)+self.horMargin,...
                                     1-4*self.boxHeight-3*self.verMargin,...
                                     self.boxWidth,...
                                     self.boxHeight],...
                'Callback',         @self.executeMove);
            
            %% Cartesian position sliders + number boxes
                
            for ii = 1:3
                self.posSlider(ii) = uicontrol(...
                    'Parent',           self.fig,...
                    'Style',            'slider',...
                    'Fontsize',         self.fontSize,...
                    'Units',            'normalized',...
                    'Position',         [self.horMargin,...
                                         lastPos(2)-(ii+1)*self.boxHeight-ii*self.verMargin,...
                                         self.sliderWidth,...
                                         self.boxHeight],...
                    'Min',              self.posMinMax(ii,1),...
                    'Max',              self.posMinMax(ii,2),...
                    'SliderStep',       [0.001 0.05],...
                    'Value',            P(ii));
                self.posSliderBox(ii) = uicontrol(...
                    'Parent',           self.fig,...
                    'Style',            'text',...
                    'String',           sprintf('%.2f', P(ii)),...
                    'Fontsize',         self.fontSize,...
                    'Units',            'normalized',...
                    'Position',         [2*self.horMargin + self.sliderWidth,...
                                         lastPos(2)-(ii+1)*self.boxHeight-ii*self.verMargin,...
                                         self.boxWidth,...
                                         self.boxHeight]);
                % add listener to update numeric value in posSlider
                % continuously, not only after release of mouse.
                addlistener(self.posSlider(ii) ,'Value', 'PostSet', ...
                    @(hObj, evtData) self.updateSliderValue(hObj, evtData, self.posSliderBox(ii), 'pos'));
            end
            lastPos = self.posSlider(3).Position;
            

            %% Joint angle sliders + number boxes
            
            for ii = 1:6
                self.jointSlider(ii) = uicontrol(...
                    'Parent',           self.fig,...
                    'Style',            'slider',...
                    'Fontsize',         self.fontSize,...
                    'Units',            'normalized',...
                    'Position',         [self.horMargin,...
                                         lastPos(2)-(ii+1)*self.boxHeight-ii*self.verMargin,...
                                         self.sliderWidth,...
                                         self.boxHeight],...
                    'Min',              self.jointMinMax(ii,1),...
                    'Max',              self.jointMinMax(ii,2),...
                    'SliderStep',       [0.001 0.05],...
                    'Value',            q(ii)*180/pi);
                self.jointSliderBox(ii) = uicontrol(...
                    'Parent',           self.fig,...
                    'Style',            'text',...
                    'String',           sprintf('%.2f', q(ii)*180/pi),...
                    'Fontsize',         self.fontSize,...
                    'Units',            'normalized',...
                    'Position',         [2*self.horMargin + self.sliderWidth,...
                                         lastPos(2)-(ii+1)*self.boxHeight-ii*self.verMargin,...
                                         self.boxWidth,...
                                         self.boxHeight]);
                % add listener to update numeric value in posSlider
                % continuously, not only after release of mouse.
                addlistener(self.jointSlider(ii) ,'Value', 'PostSet', ...
                    @(hObj, evtData) self.updateSliderValue(hObj, evtData, self.jointSliderBox(ii), 'joint'));
            end
            
            
            
        end
    end
    
    methods(Access=private)
        
        %% Callback from go-button, 
        % checks input validity and calls for move to location, if valid.
        function executeMove(self, ~, ~)
            tolerance = 100*eps;
            % get input data
            switch self.rotTabGroup.SelectedTab.Title
                case 'matrix'
                    rotMatVal = reshape(cellfun(@str2num, {self.matInput.String}), [3,3]);
                case 'ZYX'
                    angleVal  = cellfun(@str2num, {self.zyxInput.String})';
                    angleVal  = angleVal*pi/180;
                    if ~any(isnan(angleVal))
                        rotMatVal = rotMat(angleVal(1), 'z') * rotMat(angleVal(2), 'y') * rotMat(angleVal(3), 'x');
                    else
                        rotMatVal = nan*eye(3);
                    end
            end
            
            % get position values, source depends on origin of callback
            posVal    = cellfun(@str2num, {self.posInput.String})';
            
            % check if all inputs were numeric
            if any(any(isnan(rotMatVal))) || any(isnan(posVal))
                errordlg('Location inputs must be numeric', 'Invalid input');
                return
            end
            % check if rotation matrix is valid and right handed
            if isalmost(norm(rotMatVal), 1, tolerance) && ...
               isalmost(cross(rotMatVal(:,1), rotMatVal(:,2)), rotMatVal(:,3), tolerance)
               self.moveToLoc(posVal, rotMatVal);
            else
                errordlg('Rotation matrix not normalized or right-handed', 'Invalid input')
                return
            end
        end
        
        
        %% Move to new location
        function moveToLoc(self, P, C)
            qNext = self.robotKin.inverseKinematics(P, C, self.configSelect.Value);
            if isempty(qNext) % if qNext is empty, move is not possible
                return
            end
            %{
            TODO joint-space interpolation from current q to qNext
                return 3 Mx6 matrices of consecutive joint pos, vel & accel
                plot joint vel, pos and accel in output fig A
            TODO go through matrix at set time intervals
                update output position figure at each time step in output
                fig B
                send joint pos at each time step to spanviewer
                stop loop at end of move
            %}
            
            % TODO remove below line (causes immediate update, used for
            % testing)
            self.updateQ(qNext);
        end
        
        
        
        %% slider listener for semi-continuous updating of current value
        function updateSliderValue(self, ~, eventdata, hValBox, type)
            hValBox.String = sprintf('%.2f', eventdata.AffectedObject.Value);
            switch type
                case 'pos'
                    qNext = self.robotKin.inverseKinematics(...
                        cell2mat({self.posSlider.Value})', ...
                        self.robotKin.getRot(6), ...
                        self.configSelect.Value);
                    if isempty(qNext) % if qNext is empty, move is not possible
                        return
                    end
                case 'joint'
                    qNext = cell2mat({self.jointSlider.Value})*pi/180;
            end
            self.updateQ(qNext);
        end
        
        %% Update joint parameters & send to outputs
        function updateQ(self, qNext)
            self.udps(qNext);
            self.robotKin.updateParams(qNext);
            self.outFigure.updatePos(self.robotKin.getPos(6))
        end
    end
end

