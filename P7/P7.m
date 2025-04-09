function P7()  
a_estrella = input("¿Desea utilizar A* o DIJKSTRA? (1 o 2)");
n1 = input("¿Donde quieres empezar?  ");
n2 = input("¿Donde quieres terminar?  ");
mapa2

n_start = nodos(n1,:);
n_end = nodos(n2,:);

map_img=imread('mapa2.pgm');
map_neg=imcomplement(map_img);
map_bin=imbinarize(map_neg);
mapa=binaryOccupancyMap(map_bin);
show(mapa);

% Marcar los puntos de inicio y destino
hold on;
plot(n_start(2), n_start(3), 'go','MarkerFaceColor','green');  % Dibujamos el origen
plot(n_end(2), n_end(3), 'ro','MarkerFaceColor','red');  % Dibujamos el destino



%Planificación de trayectoria
if a_estrella == 1 
    x = length(costes(:,1));
    y = length(costes(1,:));
    for j = 1:x
        for k=1:y
            if costes(j,k) ~= 0
                c1= nodos(j,:);
                c2 = nodos(k,:);
                dist = sqrt((c1(3)-c2(3))^2 + (c1(2)-c2(2))^2);
                nuevos_costes(j,k) = dist;
            end
        end
    end
   H = nuevos_costes*0.9; 
   [coste,ruta]=A_ESTRELLA(nuevos_costes,H,n_start(1),n_end(1))
else 
    [coste,ruta]=DIJKSTRA(costes,n_start(1),n_end(1))
end

n = length(ruta);

for i = 1:n-1
    m = ruta(i);
    j = ruta(i+1);
    alcanzado = CAMPOS_POTENCIALES(nodos(m,2),nodos(m,3),nodos(j,2),nodos(j,3),mapa);
    if ~alcanzado
        break
    end
end
if alcanzado
   disp("¡Has llegado a tu destino!")
end
end
