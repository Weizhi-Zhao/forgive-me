% L1 = 20;
% L2 = 17.503;
% L3 = 17;
% x = 0:0.1:L2/2;
% coordinate = [x; zeros(1, length(x)); ones(1, length(x)) * -4 / 3 * L1 + 0.1 * (x - 4) .^ 2];
% angles = InverseKinematics(coordinate);
% c2 = forwardKinematics(angles);
% plot(c2(1, :), c2(3, :), LineWidth=5, Color=[1, 0, 0]);
coordinate = generateTrajectory(0);
hold on
color = 0;
for i = 1:47
    coordinate = [coordinate, generateTrajectory(i)];
    plot(coordinate(1, end-1:end), coordinate(3, end-1:end), LineWidth=3, Color=[color / 3, 1-color/3, 0.7]);
    pause(0.3);
    if mod(i, 16) == 0
        color = color + 1;
    end
end

plot(coordinate(1, :), coordinate(3, :), LineWidth=3, Color=[1, 0, 0]);