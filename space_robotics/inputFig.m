classdef inputFig < handle
    
    properties (Constant)
        boxHeight = 0.05;
        boxWidth  = 0.07;
        sliderWidth = 0.5;
        sliderHeight = 0.03;
        horMargin = 0.01;
        verMargin = 0.01;
        fontSize  = 12;
        t_step = 0.016;
        vAlphabet = 10;
        PointSequence = [750,750,750;
                         -750,750,750;
                         -750,-750,750;
                         750,-750,750;
                         750,750,-750;
                         -750,750,-750;
                         -750,-750,-750;
                         750,-750,-750;];
        RotationSequence = zeros(8,3);
        
        posMinMax = [-2000 2000;...
                     -2000 2000;...
                     -2000 3000];
    end
    
    properties (Access=private)
        jointMinMax;
        qMatrix; % matrix containing next move joint angles
        n;       % current index for qMatrix
        nMax;    % last index in qMatrix
        
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
        routineButton;
        posSlider = gobjects(3,1);
        posSliderBox = gobjects(3,1);
        jointSlider = gobjects(6,1);
        jointSliderBox = gobjects(6,1);
        textBox;
        writeButton;
        ps3Button;
        
        % Other objects
        robotKin;
        udps;
        outFigure;
        timerObj;
        fontRender;
        timerObjPs3;
        rtoA;
        
        % listener objects for sliders
        jointListener
        posListener
    end
    
    methods
        %% Constructor
        function self = inputFig(robotKin, udps, outFigure)
            self.robotKin = robotKin;
            self.udps = udps;
            self.outFigure = outFigure;
            self.jointMinMax = [robotKin.qMin', robotKin.qMax']*180/pi;
            self.timerObj = timer(...
                'ExecutionMode',        'fixedRate',...
                'Period',               self.t_step,...
                'TimerFcn',             @self.timerCallback,...
                'StopFcn',              @self.timerStopFunction...
                );
            self.fontRender = fontRender();
            
            
            self.timerObjPs3 = timer(...
                'ExecutionMode',        'fixedRate',...
                'Period',               0.05,...
                'TimerFcn',             @self.ps3Callback,...
                'StopFcn',              @self.timerStopFunction...
                );
            % start and immediately pause simulink model used to get inputs
            set_param('ps3Sim', 'SimulationCommand', 'start', 'SimulationCommand', 'pause')
            % get runtime object for controller output block
            self.rtoA = get_param('ps3Sim/locationOut', 'RuntimeObject');
        end
        
        %% Create initial figure and uicontrol
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
            lastPos = self.goButton.Position;
            
            %% Quadrant/Octant routine
            
            self.routineButton = uicontrol(...
                'Parent',           self.fig,...
                'Style',            'pushbutton',...
                'String',           'Quad-Routine',...
                'Fontsize',         self.fontSize,...
                'Units',            'normalized',...
                'Position',         [lastPos(1)+lastPos(3)+self.horMargin,...
                                     1-4*self.boxHeight-3*self.verMargin,...
                                     3.5*self.boxWidth,...
                                     3.5*self.boxHeight],...
                'Callback',         @self.routineMove);
            lastPos = self.routineButton.Position;
            
            %% Text input for writing with arm
            self.textBox = uicontrol(...
                    'Parent',           self.fig,...
                    'Style',            'edit',...
                    'String',           'hello world',...
                    'Fontsize',         self.fontSize,...
                    'Units',            'normalized',...
                    'Position',         [lastPos(1)+lastPos(3)+self.horMargin,...
                                         1-3*self.boxHeight-3*self.verMargin,...
                                         3.5*self.boxWidth,...
                                         2.5*self.boxHeight]);
            self.writeButton = uicontrol(...
                    'Parent',           self.fig,...
                    'Style',            'pushbutton',...
                    'String',           'write',...
                    'Fontsize',         self.fontSize,...
                    'Units',            'normalized',...
                    'Position',         [lastPos(1)+lastPos(3)+self.horMargin,...
                                         1-4*self.boxHeight-3*self.verMargin,...
                                         3.5*self.boxWidth,...
                                         self.boxHeight],...
                    'Callback',         @self.writeText);
                
                
            %% PS3 input enable button
            
            self.ps3Button = uicontrol(...
                    'Parent',           self.fig,...
                    'Style',            'pushbutton',...
                    'String',           'Start PS3',...
                    'Fontsize',         self.fontSize,...
                    'Units',            'normalized',...
                    'Position',         [lastPos(1)+lastPos(3)+self.horMargin,...
                                         1-7*self.boxHeight-3*self.verMargin,...
                                         3.5*self.boxWidth,...
                                         self.boxHeight],...
                    'Callback',         @self.ps3Start);
            
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
                    'Value',            P(ii),...
                    'Callback',         {@self.sliderCallback, 'pos'});
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
                self.posListener = [self.posListener, ... 
                    addlistener(self.posSlider(ii) ,'Value', 'PostSet', ...
                    @(hObj, evtData) self.updateSliderValue(hObj, evtData, self.posSliderBox(ii), 'pos'))];
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
                    'Value',            q(ii)*180/pi,...
                    'Callback',         {@self.sliderCallback, 'joint'});
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
                self.jointListener = [self.jointListener,...
                    addlistener(self.jointSlider(ii) ,'Value', 'PostSet', ...
                    @(hObj, evtData) self.updateSliderValue(hObj, evtData, self.jointSliderBox(ii), 'joint'))];
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
        
        
        %% Callback from routine-button
        function routineMove(self, ~, ~)
            points = self.PointSequence;
            rotations = self.RotationSequence;
            % Perform inverse kinematics
            qMotion = zeros(10,6);
            for ii= 1:1:size(points,1)
                P = points(ii,:)';
                R = rotMat(rotations(ii,1), 'z') * rotMat(rotations(ii,2), 'y') * rotMat(rotations(ii,3), 'x');
                qMotion(ii+1,:) = self.robotKin.inverseKinematics(P, R, self.configSelect.Value);
            end
            % Pass states to planning
            [Qplan,vq,aq,tarray] = path_planning(30,self.t_step,[self.robotKin.q;
                                           qMotion], ...
                                          [self.robotKin.qdotMax;
                                           0.5*self.robotKin.qdotMax]);
            if isempty(Qplan)
                errordlg('Desired path not possible - blend times too large', 'Error', 'modal')
                return
            end
            self.n = 1;
            self.toggleInputs('off');
            self.outFigure.updatePlots(Qplan,vq,aq,tarray)
            self.qMatrix = Qplan;
            [self.nMax, ~] = size(self.qMatrix);
            % start timer/animation
            start(self.timerObj);
        end
        
        
        %% Move through given points and orientations
        function moveThroughLocs(self, Plist, Clist)
            if size(Clist,1) ==1
                Clist(1:size(Plist,1),:) = Clist(1,:);
            end
            qMotion = zeros(size(Plist,1)+1,6);
            qMotion(1,:) = self.robotKin.q;
            %Check if motion is possible 
            for ii = 1:1:size(Plist,1)
                qNext = self.robotKin.inverseKinematics(Plist(ii,:), Clist(ii,:), self.configSelect.Value);
                if isempty(qNext) % if qNext is empty, move is not possible
                    errordlg('Desired path not possible', 'Error', 'modal')
                    return
                end
                qMotion(ii,:) = qNext;
            end
            % Plan the motion
            [Qplan,vq,aq,tarray] = path_planning(5,self.t_step,qMotion, ...
                                          [self.robotKin.qdotMax;
                                           0.3*self.robotKin.qdotMax]);
            if isempty(Qplan)
                    errordlg('Desired path not possible - blend times too large', 'Error', 'modal')
                return
            end
            self.n = 1;
            self.toggleInputs('off')
            % Animate
            self.qMatrix = Qplan;
            [self.nMax, ~] = size(self.qMatrix);
            
            start(self.timerObj);
            % Plot
            self.outFigure.updatePlots(Qplan,vq,aq,tarray)
        end
        
        
        %% Move to new location
        function moveToLoc(self, P, C)
            qNext = self.robotKin.inverseKinematics(P, C, self.configSelect.Value);
            if isempty(qNext) % if qNext is empty, move is not possible
                errordlg('Desired location not possible', 'Error', 'modal')
                return
            end
            self.n = 1;
            self.toggleInputs('off');
            [Qplan,vq,aq,tarray] = path_planning(5,self.t_step,[self.robotKin.q;
                                           qNext], ...
                                          [self.robotKin.qdotMax;
                                           0.3*self.robotKin.qdotMax]);
            % TODO replace angular acceleration maxima with realistic
            % values
            self.outFigure.updatePlots(Qplan,vq,aq,tarray)
            self.qMatrix = Qplan;
            [self.nMax, ~] = size(self.qMatrix);
            % start timer/animation
            start(self.timerObj);
        end
        
        
        %% Move to neutral state function
        function moveToNeutralState(self)
            stop(self.timerObj);
            qNext = [0,0,0,0,0,0];           
            self.n = 1;
            self.toggleInputs('off');
            [Qplan,~,~,~] = path_planning(2,self.t_step,[self.robotKin.q;
                                           qNext], ...
                                          [self.robotKin.qdotMax;
                                           0.3*self.robotKin.qdotMax]);
            % TODO replace angular acceleration maxima with realistic
            % values
            self.qMatrix = Qplan;
            [self.nMax, ~] = size(self.qMatrix);
            % start timer/animation
            start(self.timerObj);
        end
        
        
        %% write user specified text in 3D space, using Museum of Arts and Design (MAD) font
        function writeText(self, ~, ~)
            inputString = self.textBox.String;
            % sanitize input, only allow alphabetic characters and spaces
            stringMatch = regexp(inputString, '[a-z ]*', 'ignorecase', 'match');
            inputString = strcat(stringMatch{:});
            % show sanitized input 
            self.textBox.String = inputString;
            R=self.robotKin.getRot(6);
            P = self.robotKin.getPos(6);
            scale =100;
            textMoves = self.fontRender.sequentializeText(upper(inputString),scale,[R(:,1),R(:,2)],P);
            self.alphabetInterpol(textMoves);
        end
        
        
        %% Alphabet Interpolation
         function alphabetInterpol(self, Poslist)
             R = self.robotKin.getRot(6); %axis normal to plane to write in
             ax  = R(:,3);
             qmotion =[];
             for ii = 2:size(Poslist,3)
                if Poslist(3,1,ii) == 0
                    CartInterpol = cartLineEqual(Poslist(1,:,ii)',Poslist(2,:,ii)',...
                                                self.t_step,self.vAlphabet);
                    for jj = 1:size(CartInterpol, 2)
                        q = self.robotKin.inverseKinematics(CartInterpol(:,jj), R, self.configSelect.Value);
                        if isempty(q) % if qNext is empty, move is not possible
                            CartInterpol(:,jj)
                            errordlg('Desired location not possible - Line error', 'Error', 'modal')
                            return
                        end
                        qmotion = [qmotion;q];
                    end
                   
                else
                    CartInterpol = cartCircEqual(Poslist(1,:,ii)',Poslist(2,:,ii)',...
                                                Poslist(4,:,ii)',ax,Poslist(3,1,ii)',...
                                                self.t_step,self.vAlphabet);
                    qTemp = zeros(1,2);
                    for jj = 1:size(CartInterpol, 2)
                        q = self.robotKin.inverseKinematics(CartInterpol(jj,:), R, self.configSelect.Value);
                        if isempty(q) % if qNext is empty, move is not possible
                            disp(CartInterpol(jj,:))
                            errordlg('Desired location not possible - Circ error', 'Error', 'modal')
                            return
                        end
                        qmotion = [qmotion;q];
                    end
                end
                disp(ii/size(Poslist,3))
            end
            x = [];
            for ii = 1:length(qmotion)
            A = self.robotKin.forwardKinematics(qmotion(ii,:), 6);
            x = [x, A(1:3, 4)];
            end
            figure(4)
            plot3(x(1,:), x(2,:), x(3,:))
            self.n = 1;
            self.toggleInputs('off');
            self.qMatrix = qmotion;
            [self.nMax, ~] = size(self.qMatrix);
            % start timer/animation
            start(self.timerObj);
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
                        errordlg('Desired location not possible', 'Error', 'modal')
                        return
                    end
                case 'joint'
                    qNext = cell2mat({self.jointSlider.Value})*pi/180;
            end
            self.updateQ(qNext);
        end
        
        
        %% slider callback, called when releasing slider,
        % updates the other inputs to reflect the current position
        function sliderCallback(self, ~, ~, type)
            switch type
                case 'pos' % update joint orientation sliders
                    [self.jointListener.Enabled] = deal(false);
                    values = num2cell(self.robotKin.q*180/pi);
                    [self.jointSlider.Value] = values{:};
                    values = cellfun(@(x) sprintf('%.2f', x), values, 'UniformOutput',false);
                    [self.jointSliderBox.String] = values{:};
                    values = cellfun(@(x) sprintf('%.2f', x), num2cell(self.robotKin.getPos(6)), 'UniformOutput',false);
                    [self.posInput.String] = values{:};
                    [self.jointListener.Enabled] = deal(true);
                case 'joint' % update end effector position sliders & boxes
                    [self.posListener.Enabled] = deal(false);
                    values = num2cell(self.robotKin.getPos(6));
                    [self.posSlider.Value] = values{:};
                    values = cellfun(@(x) sprintf('%.2f', x), values, 'UniformOutput',false);
                    [self.posSliderBox.String] = values{:};
                    [self.posInput.String] = values{:};
                    [self.posListener.Enabled] = deal(true);
            end
        end
        
        
        %% Update joint parameters & send to outputs
        function updateQ(self, qNext)
            self.udps(qNext);
            self.robotKin.updateParams(qNext);
            self.outFigure.updatePos(self.robotKin.getPos(6));
        end
        
        
        %% Timer callback - animate movement
        function timerCallback(self, ~, ~)
            if self.n > self.nMax
                stop(self.timerObj);
                return
            end
            % get next joint orientations
            qNext = self.qMatrix(self.n, :);
            self.n = self.n + 1;
            % send to spanviewer and output figure
            self.udps(qNext);
            ANext = self.robotKin.forwardKinematics(qNext, 6);
            self.outFigure.updatePos(ANext(1:3,4));
        end
        
        
        %% Timer stop function, for ending move
        function timerStopFunction(self, ~, ~)
            % update robot class
            qNext = self.qMatrix(self.n-1, :);
            self.robotKin.updateParams(qNext);
            self.toggleInputs('on');
            % update sliders to new position
            [self.jointListener.Enabled, self.posListener.Enabled] = deal(false);
            qValues = num2cell(self.robotKin.q*180/pi);
            [self.jointSlider.Value] = qValues{:};
            qValues = cellfun(@(x) sprintf('%.2f', x), qValues, 'UniformOutput',false);
            [self.jointSliderBox.String] = qValues{:};
            posValues = num2cell(self.robotKin.getPos(6));
            [self.posSlider.Value] = posValues{:};
            posValues = cellfun(@(x) sprintf('%.2f', x), posValues, 'UniformOutput',false);
            [self.posInput.String] = posValues{:};
            [self.posSliderBox.String] = posValues{:};
            [self.jointListener.Enabled, self.posListener.Enabled] = deal(true);
        end
        
        
        %% PS3 controller input callbacks
        function ps3Callback(self,~, ~)
            set_param('ps3Sim', 'SimulationCommand', 'continue', 'SimulationCommand', 'pause')
            angles = [self.rtoA.OutputPort(1).data(4:5)', 0];
            C = rotMat(angles(1), 'z')*rotMat(angles(2), 'y')*rotMat(angles(3), 'x');
            qNext = self.robotKin.inverseKinematics(self.rtoA.OutputPort(1).data(1:3), C, self.configSelect.Value);
            if isempty(qNext)
                self.ps3Pause(0,0);
                errordlg('Desired location not possible', 'Error', 'modal')
                return
            end
            self.qMatrix = qNext;
            self.udps(self.qMatrix);
            ANext = self.robotKin.forwardKinematics(self.qMatrix, 6);
            self.outFigure.updatePos(ANext(1:3,4));
        end
        
        function ps3Start(self, ~, ~)
            self.toggleInputs('off')
            self.ps3Button.String = 'Pause PS3 input';
            self.ps3Button.Callback = @self.ps3Pause;
            start(self.timerObjPs3);
        end
        
        function ps3Pause(self, ~, ~)
            self.n = 2;
            stop(self.timerObjPs3);
            self.ps3Button.String = 'Start PS3 input';
            self.ps3Button.Callback = @self.ps3Start;
        end
        
        %% toggle input button enable (during moves), state must be 'on'/'off'
        function toggleInputs(self, state)
            self.goButton.Enable = state;
            self.routineButton.Enable = state;
            self.writeButton.Enable = state;
            [self.jointSlider.Enable] = deal(state);
            [self.posSlider.Enable] = deal(state);
        end
    end
end

