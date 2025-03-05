% Modelo P1
R = 0;
K = 0;

v = 1.2; % constante
dmin = 20; % Separación mínima entre puntos
T = 0.3; % Tiempo de muestreo GPS
epsilon = 1; % Distancia de alcance al waypoint


trayectoria = [0 0;20 0;20 20;-10 30;-20 -10;0 -30;0 0];

% Probar varios valores de ganancia (Controlador P)
G = [1 2 3];

% Inicializar variables
x = 0;
y = 0;
theta = 0;
d = 1000;

for j=2:size(trayectoria,1) % Each point in the trayectory
    p = trayectoria(j,:);
    % Go to point until d<epsilon
    while d>epsilon
        % Calcular w con control proporcional
        vector = p-[x y];
        d = norm(vector);
        theta_r = atan2(vector(2),vector(1));
        theta_g = theta_r-theta;
        w = G*theta_g;

        % MCI
        [wi,wd] = MCI(v,w,K,R);

        % Simulate a step
        % [dx,dy,dtheta] = step(wi,wd,T)

        % Recalculate position
        x = x+dx;
        y = y+dy;
        theta = theta+dtheta;

        % Add noise
        [x,y,theta] = DGPS(x,y,theta);
    end
end

% Mostrar gráficamente el camino (Coordenadas en metros)
%figure
%plot()
% Gráfica velocidad
%figure
%plot()