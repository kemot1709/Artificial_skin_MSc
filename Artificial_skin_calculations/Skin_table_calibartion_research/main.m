clear;
close all;

params;

filenames = ls(PATH + "*weight_test*");

for i = 1:size(filenames, 1)
    data(i) = data_import(PATH + filenames(i, :));
end
