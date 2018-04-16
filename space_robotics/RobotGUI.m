function RobotGUI()
    %ROBOTGUI opens GUI for control & output of ARC Mate 120iC robot simulation
    
    % add paths to required .m files
    addpath('helper_functions')
    addpath('font')
    
    % set up DH parameters
    a = zeros(1,6);
    d = zeros(1,6);
    alpha = [pi/2 0 pi/2 pi/2 pi/2 0];
    dTheta = [0 pi/2 0 pi pi 0];    
    a(1) = 150;
    d(1) = 525;
    a(2) = 790;
    a(3) = 250;
    d(4) = 835;
    d(6) = 100;
    qMin = [-185, -175, -229, -200, -180, -450]*pi/180;
    qMax = [ 185,   85,  229,  200,  180,  450]*pi/180;
    qdotMax = [195,178,180,360,360,550]*pi/180; % Maximum joint velocities from data sheet
    % Set up objects;
    robotKin = dhRobot(a, d, alpha, zeros(1,6), 'RRRRRR', dTheta, qMin, qMax,qdotMax); % robot inv & forward kinematics
    udps = dsp.UDPSender; % udp packet sender
    outFigure = outputFig(robotKin.ATotal{6}(1:3,4));
    inFigure = inputFig(robotKin, udps, outFigure);
    
    % creat figure windows
    outFigure.createFigure;
    inFigure.createFigure;
        
    % Open up spanviewer
%      winopen('spanviewer_files\ARCMate120iC.span');
end