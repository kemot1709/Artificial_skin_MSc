clear;
close all;


load("workspace/28.mat");
load("workspace/50.mat");
load("workspace/150.mat");


figure;
errorbar(a28{2}, a28{1}, a28{1}-a28{3}, a28{4}-a28{1}, [], [], '*-');
hold on;
errorbar(a50{2}, a50{1}, a50{1}-a50{3}, a50{4}-a50{1}, [], [], '*-');
errorbar(a150{2}, a150{1}, a150{1}-a150{3}, a150{4}-a150{1}, [], [], '*-');
figure_options(a28{5})
hold off;

figure;
errorbar(b28{2}, b28{1}, b28{1}-b28{3}, b28{4}-b28{1}, [], [], '*-');
hold on;
errorbar(b50{2}, b50{1}, b50{1}-b50{3}, b50{4}-b50{1}, [], [], '*-');
errorbar(b150{2}, b150{1}, b150{1}-b150{3}, b150{4}-b150{1}, [], [], '*-');
figure_options(b28{5})
hold off;


function figure_options(method)
grid
legend('Location', 'Best');
legend('28mm diameter object', '50mm diameter object', '150mm diameter object')
xlabel('Real weight of item');
ylabel('Weight read by tactile sensor');
title(strcat('Comaparison for method: ', method));
end
