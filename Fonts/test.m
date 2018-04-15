%M = containers.Map{{'a', 'b'}, {{1 2 {2 3}},{1,4,{4,5}}}}
%M{'a'} = {1 2 {2 3}}
%x = M('a')
%x{3}{1}

%Font('A')

x = Font('A')

x = [1]

%plot3([0,1],[1,0],[2,5])
%axis('equal')
drawChar('A')

Font = containers.Map();

Font('A') = {{{0,0},{1,0},{0,0,0}},{{1,0},{1,0.55},{0,0,0}},{{1,0.55},{0,0.55},{0.5,0.55,-1}},...
            {{0,0.55},{0,0},{0,0,0}}};
function drawChar(c)%, origin = [0,0], tess = 1)
    sequence = Font(c);
    
    
    %"draw a single character from the font; tess is pts per unit angle * 10"
    if ~exist('tess')
        tess = 1
    end
    
    if ~exist('origin')
        origin = [0,0]
    end
    
    
    dx =  origin(0)
    dy =  origin(1)

    for s = sequence
        if s{2}{2} == 0
            %straight line
            print 'straight segment'
            plt.plot([s{0}{0} +dx, s{1}{0}+dx], [s{0}{1}+dy,s{1}{1}+dy])
        end
    end
end
        %else if s{2}{2} in [-1, 1]
            %circular segment
            %renderCircle(s[0], s[1], s[2][:2], s[2][2], tess, origin = origin)

