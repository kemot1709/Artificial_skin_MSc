function a = approximation_weight(data, method)
% Exstract important data
weights = [0, extractfield(data, 'weight')];
for i = 1:size(data, 1)
    avg(i) = data(i).work_avg(method);
end
avg = [data(1).idle_read(method), avg];
max_weights = extractfield(data, 'work_max');
max_weights = reshape(max_weights, [], size(data, 1));
min_weights = extractfield(data, 'work_min');
min_weights = reshape(min_weights, [], size(data, 1));

% Plot data before approximation
x1 = avg;
y1 = weights;
x1_dn = [0, min_weights(method, :)];
x1_up = [0, max_weights(method, :)];
plot_method(x1, y1, x1_dn, x1_up, 'Aproksymacja wybranego sposobu kompensacji');
hold on;

% Aproksymacja danych
a = fmin(x1, y1);
xFit = linspace(0, max(x1));
yFit = a * xFit;
plot(xFit, yFit, 'r-');
plot(xFit, xFit, 'g-');

grid on;
legend('Wybrana metoda kompensacji', 'Aproksymacja', 'Rzeczywisty pomiar wagi', 'Location', 'Best');
hold off;
end


function a = fmin(x, y)

NRCF = @(a) norm(y-a*x);
options = optimoptions('fmincon');
options = optimoptions(options, 'Display', 'off');

err = Inf;
Best = 0;

for i = 1:100
    a0 = rand(1, 1)' * 1000;
    B = fmincon(NRCF, a0, 0, 0, [], [], [], [], [], options);
    if NRCF(B) < err
        err = NRCF(B);
        Best = B;
    end
end

a = Best;
end