clear;
close all;

params;

filenames = ls(PATH + "*weight_test*");

for i = 1:size(filenames, 1)
    data(i) = data_import(PATH + filenames(i, :));
end

% sort data by weight
T = struct2table(data);
sortedT = sortrows(T, 'weight');
data = table2struct(sortedT);


weights = extractfield(data, 'weight');

for i = 1:size(data, 1)
    avg(i, :) = data(i).work_avg;
end

figure;
plot([avg'; weights], weights);
