function fig = plot_method(x, y, x_dn, x_up, name)

fig = figure('Name', name);
errorbar(x, y, [], [], x-x_dn, x_up-x, '*-b');
xlabel('Waga odczytana przez sztuczn¹ skórê');
ylabel('Rzeczywista waga przedmiotu');
title(name);

end