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
figure('Name', 'Porównanie wybranych sposobów kompensacji w zale¿noœci od przy³o¿onego nacisku');
plot([avg(columns, :)'; weights(columns)], weights(columns));
grid
legend('Location', 'Best');
legend('Kompensacja 1', 'Kompensacja 2', 'Kompensacja 3', 'Kompensacja 4', 'Kompensacja 5', ...
    'Kompensacja 6', 'Kompensacja 7', 'Kompensacja 8', 'Kompensacja 9', 'Pomiar wagi')
xlabel('Waga odczytana przez sztuczn¹ skórê');
ylabel('Rzeczywista waga przedmiotu');
title('Porównanie wybranych sposobów kompensacji w zale¿noœci od przy³o¿onego nacisku');


for i = 1:size(avg, 2)
    x1 = avg(:, i)';
    y1 = weights;
    x1_dn = [0, min_weights(:, i)'];
    x1_up = [0, max_weights(:, i)'];
    plot_method(x1(columns), y1(columns), x1_dn(columns), x1_up(columns), char(names(i)));
end

ret = 0;
end