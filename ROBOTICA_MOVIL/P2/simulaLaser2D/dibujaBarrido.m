function d=dibujaBarrido(x, y, x0, y0, phi0, rangos)
% Dibuja barrido 2D
% x, y: lista de puntos del entorno cerrado
% x0, y0, phi0: posici�n y orientaci�n del veh�culo
% rangos: lista ordenada de distancias medidas
% d: devuelve distancia a los laterales del robot

% C�lculo de coordenadas locales
paso= 5*pi/180;
alfa= 0:paso:2*pi-paso;
xl= rangos.*cos(alfa);
yl= rangos.*sin(alfa);

% C�lculo de coordenadas globales
xg= x0+cos(phi0)*xl-sin(phi0)*yl;
yg= y0+sin(phi0)*xl+cos(phi0)*yl;

plot(x, y, xg, yg, '*')
axis('equal')
hold on

% dado que el vehículo tiene su eje x local en dirección de avance 
% las lecturas correspondientes a los laterales corresponden a un
% giro de pi/2 del barrido original del laser
d =  rangos(18) + rangos(54);

end