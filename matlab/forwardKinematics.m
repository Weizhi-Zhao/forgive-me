function coordinate = forwardKinematics(angles)

    L1 = 20;
    L2 = 17.503;
    L3 = 17;

    theta = angles(2, :);
    phi = angles(3, :);

    coordinate(1, :) = L1 * sin(theta) + L2 * cos(pi / 2 - theta - phi);
    coordinate(3, :) = -( L1 * cos(theta) + L2 * sin(pi / 2 - theta - phi) );
    coordinate(2, :) = zeros(1, length(theta));
end