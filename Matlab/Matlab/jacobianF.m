function F = jacobianF(x, u, dt, p)

    n  = numel(x);              
    F  = zeros(n);
    fx = vehicleProcessModel(x, u, dt, p);
    eps = 1e-4;

    for j = 1:n
        dx = zeros(n,1);
        dx(j) = eps;
        fx2 = vehicleProcessModel(x + dx, u, dt, p);
        F(:,j) = (fx2 - fx) / eps;
    end
end
