function fig = plot_method(x, y, y_dn, y_up, name)

fig = figure('Name', name);
errorbar(x, y, y-y_dn, y_up-y, [], [], '*-b');
ylabel('Weight read by tactile sensor');
xlabel('Real weight of item');
title(name);

end