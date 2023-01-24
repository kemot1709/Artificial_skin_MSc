function output = kolor(number)

switch (number)
    case 1
        r = 0;
        g = 0;
        b = 1;
    case 2
        r = 1;
        g = 0;
        b = 0;
    case 3
        r = 0;
        g = 1;
        b = 0;
    case 4
        r = 0;
        g = 0;
        b = 0.1724;
    case 5
        r = 1;
        g = 0.1034;
        b = 0.7241;
    case 6
        r = 1;
        g = 0.8276;
        b = 0;
    case 7
        r = 0;
        g = 0.3448;
        b = 0;
    case 8
        r = 0.5172;
        g = 0.5172;
        b = 1;
    case 9
        r = 0.6207;
        g = 0.3103;
        b = 0.2759;
    case 10
        r = 0;
        g = 1;
        b = 0.7586;
    case 11
        r = 0;
        g = 0.5172;
        b = 0.5862;
    case 12
        r = 0;
        g = 0;
        b = 0.4828;
    case 13
        r = 0.5862;
        g = 0.8276;
        b = 0.3103;
    case 14
        r = 0.9655;
        g = 0.6207;
        b = 0.8621;
    case 15
        r = 0.8276;
        g = 0.0690;
        b = 1;
    case 16
        r = 0.4828;
        g = 0.1034;
        b = 0.4138;
    case 17
        r = 0.9655;
        g = 0.0690;
        b = 1;
    case 18
        r = 1;
        g = 0.7586;
        b = 0.5172;
    case 19
        r = 0.1379;
        g = 0.1379;
        b = 0.0345;
    case 20
        r = 0.5517;
        g = 0.6552;
        b = 0.4828;
    otherwise
        r = 0;
        g = 0;
        b = 1;
end
output = [r, g, b];
end