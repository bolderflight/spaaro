close all;
clear;
clc;

test_input = [0.4,0.3];
test_output = test_input;

[test_output(1), test_output(2)] = circular_limit(test_input(1), test_input(2));

figure()
circle2(0, 0,1);
hold on
plot (test_input(1), test_input(2), 'r*');
plot (test_output(1), test_output(2), 'b*');
circle2(0, 0,1);
xlim([-2,2])
ylim([-2,2])
grid on
grid minor



function [x,y] = circular_limit(x_in, y_in)
    % Normalize square [-1,1] to unit circular limit
    r = sqrt(x_in.^2 + y_in.^2);
    if r <= 1
        x = x_in;
        y = y_in;
    else
        x = x_in ./ r;
        y = y_in ./ r;
    end
end

function h = circle2(x,y,r)
d = r*2;
px = x-r;
py = y-r;
h = rectangle('Position',[px py d d],'Curvature',[1,1],'LineStyle','--');
daspect([1,1,1])
end