function [A, B, C, err] = approximation_fmin(weight, resistance)

x = weight;
y = resistance;

NRCF = @(b) norm((y - hyprb(b, x)).*x);
% NRCF = @(b) sum(norm(y - hyprb(b,x)),1);
options = optimoptions('fmincon');
options = optimoptions(options, 'Display', 'off');

err = Inf;
Best = [0; 0; 0];

for i = 1:100
    B0 = rand(1, 3)' * 1000;
    B = fmincon(NRCF, B0, [0, 0, 1], [0], [], [], [], [], [], options);
    if NRCF(B) < err
        err = NRCF(B);
        Best = B;
    end
end

A = Best(1);
B = Best(2);
if evalin( 'base', 'exist(''x0_eq_0'',''var'') == 1' ) && evalin( 'base', 'x0_eq_0 == true' )
    C = 0;
else
    C = Best(3);
end
