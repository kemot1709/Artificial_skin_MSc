clear;
close all;
params;


% Import all data from csv files
filenames = ls(PATH + "*weight_test*");
for i = 1:size(filenames, 1)
    data(i) = data_import(PATH + filenames(i, :));
end

% sort data by weight
T = struct2table(data);
sortedT = sortrows(T, 'weight');
data = table2struct(sortedT);

% Plot data
plot_data(data, M_NAMES, 'all', 0);
plot_data(data, M_NAMES, 'd28', 0);
plot_data(data, M_NAMES, 'd50', 0);
plot_data(data, M_NAMES, 'd150', 0);
