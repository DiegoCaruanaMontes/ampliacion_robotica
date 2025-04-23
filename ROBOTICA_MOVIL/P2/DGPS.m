function pos = DGPS(X, Y, P)
%DGPS Simulacion de un receptor GPS diferencial 
% X, Y: posicion real en el plano del receptor
% pos: coordenadas x, y, phi medidas

pos= [X; Y; P]+sqrtm([0.4 -0.014 0; -0.014 0.5 0;0 0 0.1])*randn(3, 1);

pos = real(pos);

end

