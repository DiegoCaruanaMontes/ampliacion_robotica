function alcanzado = CAMPOS_POTENCIALES(x0,y0,x1,y1,mapa, res)

%% Inicialización

robot=[x0,y0,0];     % El robot empieza en la posición de origen (orientacion cero)
destino = [x1,y1];
path = [];                 % Se almacena el camino recorrido
path = [path; robot]; % Se añade al camino la posicion actual del robot
iteracion=0;              % Se controla el nº de iteraciones por si se entra en un minimo local
% Configuracion del sensor (laser de barrido)
max_rango=10;
angulos=-pi/2:(pi/180):pi/2; % resolucion angular barrido laser

% Caracteristicas del vehiculo y parametros del metodo
v=0.4;            % Velocidad del robot
D=1.5;%1.5           % Rango del efecto del campo de repulsión de los obstáculos
alfa=1;%1           % Coeficiente de la componente de atracción
beta=100;%100      % Coeficiente de la componente de repulsión

%% Calculo de la trayectoria

while norm(destino-robot(1:2)) > v && iteracion<1000    % Hasta menos de una iteración de la meta (10 cm)
   % TU CODIGO AQUI %%%%%%%%%%%%%

    F_total = 0;
    obs=rayIntersection(mapa,robot,angulos, max_rango);

    F_atr = alfa*(destino-robot(1:2));
    
    S = size(obs,1);
    F_rep = zeros(S,2);
    for j = 1:1:S
        if ~isnan(obs(j))
            rho_obs = norm(obs(j,:)-robot(1:2));
            if rho_obs <= D
                F_rep(j,:) = beta*(1/rho_obs-1/D)*(robot(1:2)-obs(j,:))/rho_obs^3;
            elseif rho_obs > D
                F_rep(j,:) = [0 0];
            end
            F_total = F_total + F_rep(j,:);
        end
    end
    F_total = F_total + F_atr;
    
    % Mover al robot en la dirección de la fuerza
    F_total = F_total/norm(F_total);% Normalizo el vector
    delta_pos = F_total*v;
    delta_theta = atan2(F_total(2),F_total(1));
    robot = robot + [delta_pos delta_theta];

    path = [path;robot];	% Se añade la nueva posición al camino seguido
    plot(path(:,1),path(:,2),'r');
    drawnow

    iteracion=iteracion+1;
end

if iteracion==1000  % Se ha caído en un mínimo local
    fprintf('No se ha podido llegar al destino.\n')
    alcanzado = false;
else 
    alcanzado = true;
end
    
end