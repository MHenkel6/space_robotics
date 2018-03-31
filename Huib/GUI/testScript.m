function testScript()


x = (-1:0.01:1).^2;
y = (-1:0.01:1).^3;
z = (-1:0.01:1).^4;
n = 1;

outFigure = outputFig([x(n), y(n), z(n)]);
outFigure.createFigure;
t_step = 0.016;
timerObj = timer(...
                'ExecutionMode',        'fixedRate',...
                'Period',               t_step,...
                'TimerFcn',             @plotNext...
                );
nMax = length(x);
start(timerObj);
            
            
function plotNext(~, ~)
    n = n+1;
    if n > nMax
        stop(timerObj)
        return
    end
    outFigure.updatePos([x(n), y(n), z(n)])
end

end