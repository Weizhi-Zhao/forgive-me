function coordinate = generateTrajectory(phase)

    L1 = 20;
    L2 = 17.503;
    L3 = 17;

    phase = mod(phase, 16);
    % 如果把总周期的一半当成子周期，subPhase就是子周期的相位
    subPhase = 0;
    if phase >= (16 / 2)
        subPhase = phase - 16 / 2;
    else
        subPhase = phase;
    end

    % 把子相位缩放到2Pi里
    t = subPhase / (16 / 2 - 1);
    coordinate = zeros(3, 1);
    if phase < (16 / 2)
        % 抬腿
        coordinate(1) = 25 * ( t  - 0.5 / pi * sin(2*pi*t) );
        coordinate(2) = 0;
        coordinate(3) = 10 * ( 0.5 - 0.5 * cos(2*pi*t) ) - L3;
    else
        % 落腿
        coordinate(1) = 25 * ( 1 - t  + 0.5 / pi * sin(2*pi*t) );
        coordinate(2) = 0;
        coordinate(3) = -2 * ( 0.5 - 0.5 * cos(2*pi*t) ) - L3;
    end
end