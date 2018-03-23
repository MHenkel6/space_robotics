function plotBase( transMat , scale)
    p = transMat(1:3, 4);
    x = [p, transMat(1:3,1)*scale+p];
    y = [p, transMat(1:3,2)*scale+p];
    z = [p, transMat(1:3,3)*scale+p];
    hold on
    plot3(x(1,:), x(2,:), x(3,:), 'r', 'LineWidth', 2)
    plot3(y(1,:), y(2,:), y(3,:), 'g', 'LineWidth', 2)
    plot3(z(1,:), z(2,:), z(3,:), 'b', 'LineWidth', 2)
end

