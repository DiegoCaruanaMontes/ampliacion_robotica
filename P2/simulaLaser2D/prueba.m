clear all
close all
clc

% Entorno cerrado definido por una lista de puntos
x=[-10 -10 10 10 20 -10]';
y=[-10 10 20  0 -10 -10]';


% Posición y orientación del vehiculo
x0= 10;
y0= -5;
phi0= pi; % Entre -pi y pi

rangos= laser2D(x, y, x0, y0, phi0);

dibujaBarrido(x, y, x0, y0, phi0, rangos);

figure

dibujaBarrido(x, y, 0, 0, 0, rangos);

% rangos(1) apunta al frente. Al aumentar "i", se gira en sentido
% antihorario
% 90º -> 18 (izquierda)
% 270º -> 54 (derecha)

%while not(x>a || x<0 || y>b || y<0)
% Asumir que la ganancia de giro es lo suficientemente baja para que el
% robot no de media vuelta

