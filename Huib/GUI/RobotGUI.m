function RobotGUI()
    %ROBOTGUI opens GUI for control & output of ARC Mate 120iC robot simulation
    
    % DH parameters
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
    
    % Set up objects;
    robotKin = dhRobot(a, d, alpha, zeros(1,6), 'RRRRRR', dTheta); % robot inv & forward kinematics
    udps = dsp.UDPSender; % udp packet sender
    
    % input class
        % figure A
        % position input 
    % output class
        % figure B
    
end

