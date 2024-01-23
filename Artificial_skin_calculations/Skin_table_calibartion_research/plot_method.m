function fig = plot_method(x, y, y_dn, y_up, name)

fig = figure('Name', name);
errorbar(x, y, y-y_dn, y_up-y, [], [], '*-b');
ylabel('Weight read by tactile sensor');
xlabel('Real weight of item');
title(name);

% set(gcf, 'Position', [0,0,700,350])
% ax = gca;
% outerpos = ax.OuterPosition;
% ti = ax.TightInset; 
% left = outerpos(1) + ti(1);
% bottom = outerpos(2) + ti(2);
% ax_width = outerpos(3) - ti(1) - ti(3);
% ax_height = outerpos(4) - ti(2) - ti(4);
% ax.Position = [left bottom ax_width ax_height];

end