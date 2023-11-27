function ret = plot_data(data, names, filter, methods)
% methods is currently unused, need to be properly implemented

% Pull out important data
for i = 1:size(data, 1)
    avg(i, :) = data(i).work_avg;
end

% More important data
weights = [0, extractfield(data, 'weight')];
idles = extractfield(data, 'idle_read');
idles = reshape(idles, [], size(avg, 1))';
max_weights = extractfield(data, 'work_max');
max_weights = reshape(max_weights, [], size(avg, 1))';
min_weights = extractfield(data, 'work_min');
min_weights = reshape(min_weights, [], size(avg, 1))';
avg = [data(1).idle_read; avg];


% Filter only selected samples
if strcmp(filter, 'all')
    columns = 1:size(avg, 1);
else
    columns = 1;
    for i = 1:size(data, 1)
        if contains(data(i).obj_params, filter)
            columns = [columns, i + 1];
        end
    end

end

% Plot comparison between methods
figure('Name', 'Comparison of selected compensation methods depending on applied weight');
plot(weights(columns), [avg(columns, :)'; weights(columns)]);
grid
legend('Location', 'Best');
legend('Method 1', 'Method 2', 'Method 3', 'Method 4', 'Method 5', ...
    'Method 6', 'Method 7', 'Method 8', 'Method 9', 'Real weight')
ylabel('Weight read by tactile sensor');
xlabel('Real weight of item');
title('Comparison of selected compensation methods depending on applied weight');

for i = 1:size(avg, 2)
    x1 = weights;
    y1 = avg(:, i)';
    y1_dn = [0, min_weights(:, i)'];
    y1_up = [0, max_weights(:, i)'];
    plot_method(x1(columns), y1(columns), y1_dn(columns), y1_up(columns), char(names(i)));
end

ret = 0;
end