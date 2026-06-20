function H = jacobianH(x, u, p)


    n  = numel(x);              
    y  = vehicleMeasurementModel(x, u, p);
    m  = numel(y);              
    H  = zeros(m, n);
    eps = 1e-4;

    for j = 1:n
        dx = zeros(n,1);
        dx(j) = eps;
        y2 = vehicleMeasurementModel(x + dx, u, p);
        H(:,j) = (y2 - y) / eps;
    end
end
