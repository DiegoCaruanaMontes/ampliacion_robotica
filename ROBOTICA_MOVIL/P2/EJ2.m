clc;
close all;

% Parámetros físicos
R = 0.1;
K = 0.4;
Tm = 0.12;           % Constante de tiempo del actuador
v_deseada = 0.3;     % Velocidad deseada
v_max = 15;
Ts = 0.025;          % Tiempo de muestreo
T_total = 200;
t = 0:Ts:T_total;

% Pasillo
a = 30;
b = 10;
contorno_x = [0 a a 0 0];
contorno_y = [0 0 b b 0];

% Inicialización de pose
x_real = zeros(size(t));
y_real = zeros(size(t));
theta_real = zeros(size(t));

x_real(1) = 5;
y_real(1) = 5;
theta_real(1) = 0;

% Inicialización de velocidades de rueda
wi = zeros(size(t));
wd = zeros(size(t));
wri = zeros(size(t));
wrd = zeros(size(t));

% Velocidades lineal y angular
v = zeros(size(t));
w = zeros(size(t));

% Trayectoria
trayectoria_x = [];
trayectoria_y = [];

% Puntos objetivo (para debug/visualización)
x_target = NaN(size(t));
y_target = NaN(size(t));

% Lookahead y control
d_lookahead = 1;

for k = 1:length(t)-1
    trayectoria_x = [trayectoria_x, x_real(k)];
    trayectoria_y = [trayectoria_y, y_real(k)];

    % Cada 0.5s: obtener objetivo con láser
    if mod(k, round(0.5/Ts)) == 0
        rangos = laser2D(contorno_x, contorno_y, x_real(k), y_real(k), theta_real(k));

        d_m180 = rangos(19);
        d_m175 = rangos(18);
        d_180 = rangos(19+36);
        d_175 = rangos(19+37);

        % Cálculos angulares y de distancia
        ang = deg2rad(5);
        d_lateral = sqrt(d_180^2 + d_175^2 - 2*d_180*d_175*cos(ang));
        alpha = acos((d_180^2 + d_lateral^2 - d_175^2) / (2*d_180*d_lateral));
        delta_phi = alpha - pi/2;

        dx1 = d_180 * cos(delta_phi);
        dx2 = d_m180 * cos(delta_phi);
        dL = (dx1 + dx2)/2 - dx1;

        % Coordenadas locales del punto objetivo
        delta_x = dL * sin(delta_phi) + d_lookahead * cos(delta_phi);
        delta_y = dL * cos(delta_phi) - d_lookahead * sin(delta_phi);

        % Transformar a global
        R_theta = [cos(theta_real(k)) -sin(theta_real(k)); sin(theta_real(k)) cos(theta_real(k))];
        p_obj_local = [delta_x; delta_y];
        p_obj_global = [x_real(k); y_real(k)] + R_theta * p_obj_local;

        x_target(k) = p_obj_global(1);
        y_target(k) = p_obj_global(2);

        % Calcular curvatura y w
        gamma_d = 2 * delta_y / (delta_x^2 + delta_y^2);
        w(k) = gamma_d * v_deseada;
    else
        % Mantener objetivo anterior
        if k > 1
            x_target(k) = x_target(k-1);
            y_target(k) = y_target(k-1);
            w(k) = w(k-1);
        end
    end

    v(k) = v_deseada;

    % MCI
    wri(k) = (2*v(k) - w(k)*2*K)/(2*R);
    wrd(k) = (2*v(k) + w(k)*2*K)/(2*R);

    % Dinámica de actuadores
    dwi = (-wi(k) + wri(k)) / Tm;
    dwd = (-wd(k) + wrd(k)) / Tm;

    % Actualización de velocidad de ruedas
    wi(k+1) = wi(k) + Ts * dwi;
    wd(k+1) = wd(k) + Ts * dwd;

    % Velocidades reales
    v_real = (wi(k) + wd(k)) * R / 2;
    w_real = (wd(k) - wi(k)) * R / (2*K);

    % Saturación de velocidad angular
    if abs(w_real) > 15
        w_real = sign(w_real) * 15;
    end

    % Odometría
    theta_real(k+1) = theta_real(k) + w_real * Ts;
    x_real(k+1) = x_real(k) + v_real * cos(theta_real(k)) * Ts;
    y_real(k+1) = y_real(k) + v_real * sin(theta_real(k)) * Ts;

    % Comprobación de límites del pasillo
    if x_real(k+1) < 0 || x_real(k+1) > a || y_real(k+1) < 0 || y_real(k+1) > b
        break;
    end
end

% Graficar resultados
figure;
hold on;

% Dibujar pasillo
plot(contorno_x, contorno_y, 'k-', 'LineWidth', 2);

% Dibujar trayectoria del robot
plot(trayectoria_x, trayectoria_y, 'b-', 'LineWidth', 2);

% Puntos objetivo en rojo
plot(x_target, y_target, 'r.', 'MarkerSize', 8);

% Inicio y fin de la trayectoria
scatter(trayectoria_x(1), trayectoria_y(1), 60, 'g', 'filled'); % Inicio
scatter(trayectoria_x(end), trayectoria_y(end), 60, 'm', 'filled'); % Fin

% Etiquetas y estilo
xlabel('X (m)');
ylabel('Y (m)');
title('Trayectoria del Robot');
grid on;
axis equal;
legend('Pasillo', 'Trayectoria', 'Puntos objetivo', 'Inicio', 'Fin', ...
       'Location', 'northeastoutside');