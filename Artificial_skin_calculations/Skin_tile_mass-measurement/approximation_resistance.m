clear;
close all;

take4of6 = true; % avg 4 of 6

d1 = data_importer('data/data_1x1.csv');
d2 = data_importer('data/data_1x2.csv');
d3 = data_importer('data/data_2x2.csv');

x1 = d1(:,1);
x2 = d2(:,1);
x3 = d3(:,1);
x_0err = zeros(size(x1,1),1);

if ~take4of6
y1 = d1(:,8);
y1_dn = abs(y1-d1(:,2));
y1_up = abs(y1-d1(:,7));

y2 = d2(:,8);
y2_dn = abs(y2-d2(:,2));
y2_up = abs(y2-d2(:,7));

y3 = d3(:,8);
y3_dn = abs(y3-d3(:,2));
y3_up = abs(y3-d3(:,7));
else
y1 = d1(:,9); 
y1_dn = abs(y1-d1(:,3));
y1_up = abs(y1-d1(:,6));

y2 = d2(:,9); 
y2_dn = abs(y2-d2(:,3));
y2_up = abs(y2-d2(:,6));

y3 = d3(:,9); 
y3_dn = abs(y3-d3(:,3));
y3_up = abs(y3-d3(:,6));
end

[A,B,C,~] = approximation_fmin(x1,y1);
hyprb_best1 = [A,B,C];
[A,B,C,~] = approximation_fmin(x2,y2);
hyprb_best2 = [A,B,C];
[A,B,C,~] = approximation_fmin(x3,y3);
hyprb_best3 = [A,B,C];


figure(1)
hold on;

errorbar(x1, y1, y1_dn,y1_up,x_0err,x_0err, 'ob', 'DisplayName', 'Pole 1x1 cm')
plot(x1, hyprb(hyprb_best1,x1), '--b', 'DisplayName', 'Pole 1x1 cm - aproksymacja')
errorbar(x2, y2, y2_dn,y2_up,x_0err,x_0err, 'xr', 'DisplayName', 'Pole 1x2 cm')
plot(x2, hyprb(hyprb_best2,x2), '--r', 'DisplayName', 'Pole 1x2 cm - aproksymacja')
errorbar(x3, y3, y3_dn,y3_up,x_0err,x_0err, 'pg', 'DisplayName', 'Pole 2x2 cm')
plot(x3, hyprb(hyprb_best3,x3), '--g', 'DisplayName', 'Pole 2x2 cm - aproksymacja')

hold off
grid
legend('Location','Best');
xlabel('Si³a nacisku [g]')
ylabel('Rezystancja [$\Omega$]','Interpreter','latex')
set(gca,'XScale','log')
set(gca,'YScale','log')
