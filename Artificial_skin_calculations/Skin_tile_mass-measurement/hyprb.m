function value = hyprb(b, x)
if evalin( 'base', 'exist(''x0_eq_0'',''var'') == 1' ) && evalin( 'base', 'x0_eq_0 == true' )
    b(3) = 0;
end
value = b(1) + b(2) ./ (x + b(3));
