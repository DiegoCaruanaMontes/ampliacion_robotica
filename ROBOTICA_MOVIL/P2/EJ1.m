clc;
close all;

% Parámetros del robot y simulación
R = 0.1;
K = 0.4;
v_deseada = 1.2; % constante
T = 0.3; % Tiempo de muestreo GPS
epsilon = 1; % Distancia de alcance al waypoint
tau = 0.12; %Constante de tiempo del actuador
v_max = 15; %velocidad máxima
Ts = 0.1; %salto en la simulación
Kp = 1; % Ganancia del controlador proporcional

trayectoria = [0 0;20 0;20 20;-10 30;-20 -10;0 -30;0 0];
t_total = 150;
t = 0:Ts:t_total;

% Inicializar variables de estado
x = zeros(size(t));
y = zeros(size(t));
theta = zeros(size(t));

% Velocidades angulares de las ruedas y sus referencias
wi = zeros(size(t));
wd = zeros(size(t));
wri = zeros(size(t));
wrd = zeros(size(t));

% Velocidades lineal y angular del robot
v = zeros(size(t));
w = zeros(size(t));

% Posición GPS (con ruido simulado)
pos_gps = NaN(3, length(t));

% Inicialización de variables adicionales
wp_index = 2; % Comenzar en el segundo punto (el primero es el inicial)
x(1) = trayectoria(1,1);
y(1) = trayectoria(1,2);
theta(1) = 0;

% Bucle principal de simulación
for k = 1:length(t)-1
    % 1. Posición y ángulo actual
    pos_actual = [x(k), y(k)];
    theta_k = theta(k);

    % 2. Distancia al objetivo y cambio de punto
    objetivo = trayectoria(wp_index, :);
    dist_obj = norm(objetivo - pos_actual);

    if dist_obj < epsilon && wp_index < size(trayectoria,1)
        wp_index = wp_index + 1;
        objetivo = trayectoria(wp_index, :);
    end

    % 3. Ángulo deseado y error angular
    theta_d = atan2(objetivo(2) - y(k), objetivo(1) - x(k));
    error_theta = wrapToPi(theta_d - theta_k);

    % 4. Control proporcional
    w(k) = Kp * error_theta;
    v(k) = v_deseada;

    % 5. Cálculo de referencias para ruedas
    wri(k) = (2*v(k) - w(k)*2*K)/(2*R);
    wrd(k) = (2*v(k) + w(k)*2*K)/(2*R);

    % 6. Dinámica de actuadores (primer orden)
    dwi = (-wi(k) + wri(k)) / tau;
    dwd = (-wd(k) + wrd(k)) / tau;

    wi(k+1) = wi(k) + Ts * dwi;
    wd(k+1) = wd(k) + Ts * dwd;

    % 7. Velocidad real del robot
    v_real = (wi(k) + wd(k)) * R / 2;
    w_real = (wd(k) - wi(k)) * R / (2*K);

    % Limitar velocidad angular
    if abs(w_real) > 15
        w_real = sign(w_real) * 15;
    end

    % 8. Actualización de odometría
    theta(k+1) = theta(k) + w_real * Ts;
    x(k+1) = x(k) + v_real * cos(theta(k)) * Ts;
    y(k+1) = y(k) + v_real * sin(theta(k)) * Ts;

    % 9. Simulación de GPS cada T segundos
    if mod(k, round(T/Ts)) == 0
        pos_gps(:,k) = DGPS(x(k), y(k), theta(k)); % Función externa
        x(k) = pos_gps(1,k);
        y(k) = pos_gps(2,k);
        theta(k) = pos_gps(3,k);
    end
end

% ---- Graficar la trayectoria ----
figure;
plot(trayectoria(:,1), trayectoria(:,2), 'ro-', 'LineWidth', 2); % Trayectoria deseada
hold on;
plot(x, y, 'b-', 'LineWidth', 1.5); % Trayectoria seguida
legend('Trayectoria deseada', 'Trayectoria seguida');
xlabel('Posición en X (m)');
ylabel('Posición en Y (m)');
title('Trayectoria del Robot');
grid on;
axis equal;

% ---- Graficar velocidad angular en función del tiempo ----
figure;
plot(w, 'r-', 'LineWidth', 1.5);
xlabel('Tiempo (iteraciones)');
ylabel('Velocidad angular w (rad/s)');
title('Evolución de la Velocidad Angular');
grid on;

% ---- Graficar velocidad lineal en función del tiempo ----
figure;
plot(v, 'b-', 'LineWidth', 1.5);
xlabel('Tiempo (iteraciones)');
ylabel('Velocidad lineal v (m/s)');
title('Evolución de la Velocidad Lineal');
grid on;