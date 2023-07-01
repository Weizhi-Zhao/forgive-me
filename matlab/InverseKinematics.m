function angles = InverseKinematics(coordinate)
    
    L1 = 20;
    L2 = 17.503;
    L3 = 17;

    x = coordinate(1, :);
    y = coordinate(2, :);
    z = coordinate(3, :);

    angles = [];

    % 向上为正
    angles(3, :) = pi - acos( ( x.^2 + z.^2 - L1^2 - L2^2 ) / (-2 * L1 * L2) );
                    
    % 向前为正
    angles(2, :) = pi / 2 - atan(-z ./ x) - acos( ...
                                                (x.^2 + z.^2 + L1^2 - L2^2) ...
                                                ./ (2 * sqrt( x.^2 + z.^2 ) * L1) ...
                                            );

    angles(1, :) = zeros(1, length(x));
end