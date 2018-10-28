clear; clc; close all;

t=-10:0.1:10;
u= @(t) heaviside(t);

xa=u(t)-u(t+2);
ya=u(t)-2.*u(t-1)+u(t-2);
xb=sin(t).*(u(t)-u(t-2*pi));
yb=u(t)-u(t-2*pi);
xc=sin(t).*(u(t)-u(t-pi));
yc=u(t)-u(t-pi);

for i=1:6
    subplot(3,2,i)
    switch i
        case 1
            plot(t,xa); title('a) x(t)')
        case 2
            plot(t,ya,'r'); title('a) y(t)')
        case 3
            plot(t,xb); title('b) x(t)')
        case 4
            plot(t,yb,'r'); title('b) y(t)')
        case 5
            plot(t,xc); title('c) x(t)')
        case 6
            plot(t,yc,'r'); title('c) y(t)')
    end
    grid on;
    axis([-1 8, -1.5 1.5])
end