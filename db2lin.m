function [Y] = db2lin(x)
    Y = 10 .^ (x / 10);
end