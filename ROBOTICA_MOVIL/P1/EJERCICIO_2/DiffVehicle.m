classdef DiffVehicle
    
    properties
        d       double % Distancia entre ruedas (m)
        R       double % Radio de las ruedas (m)           
        motor_i Motor  % Motor izquierdo
        motor_d Motor  % Motor derecho
    end
    
    methods
        function obj = DiffVehicle(d, R, motor_i, motor_d)
            % Constructor
            obj.d = d;
            obj.R = R;
            obj.motor_i = motor_i;
            obj.motor_d = motor_d;
        end

        function [x, y, theta] = simulate(obj, wid, wdd, t, T)
            % Simula el movimiento del vehículo durante un tiempo T para
            % actuación constante en las ruedas.
            % 
            % PARÁMETROS
            % wid: velocidad angular deseada para la rueda izquierda
            % wdd: velocidad angular deseada para la rueda derecha
            % t: tiempo de muestreo
            % T: tiempo total de simulación
            %
            % SALIDA
            % x: vector de posiciones en el eje x global
            % y: vector de posiciones en el eje y global
            % theta: vector de ángulos en el sistema de referencia global
            
            n = round(T/t); % Número de iteraciones

            % Inicializar salida
            x = zeros(1,n);
            y = zeros(1,n);
            theta = zeros(1,n);

            wi = zeros(1,n);
            wd = zeros(1,n);

            v = zeros(1,n);
            w = zeros(1,n);

            for i = 2:n
                [x(i), y(i), theta(i), wi(i), wd(i), v(i), w(i)] = obj.step(x(i-1), y(i-1), theta(i-1), wi(i-1), wd(i-1), wid, wdd, t);
            end
        end

        function [x, y, theta, wi, wd, v, w] = step(obj, x0, y0, theta0, wi0, wd0, wid, wdd, t)
            % Calcula el incremento de la pose del vehículo
            % 
            % PARÁMETROS
            % x0: valor de la pose en el eje x global
            % y0: valor de la pose en el eje y global
            % theta0: valor de theta en el sistema de coordenadas global
            % wi0: velocidad angular anterior de la rueda izquierda
            % wd0: velocidad angular anterior de la rueda derecha
            % wid: velocidad angular deseada para la rueda izquierda
            % wdd: velocidad angular deseada para la rueda derecha
            % t: incremento de tiempo / tiempo de muestreo
            %
            % SALIDA
            % x: nuevo valor de la pose en el eje x global
            % y: nuevo valor de la pose en el eje y global
            % theta: nuevo valor theta en el sistema de coordenadas
            % wi: velocidad angular de la rueda izquierda
            % wd: velocidad angular de la rueda derecha
            % v: velocidad lineal
            % w: velocidad angular
            
            % Calcular velocidades angulares
            wi = obj.motor_i.get_w(wi0, wid, t);
            wd = obj.motor_i.get_w(wd0, wdd, t);

            % Modelo cinemático directo
            [v, w] = obj.mcd(wi, wd);

            % Odometría
            [x, y, theta] = obj.odom(v, w, x0, y0, theta0, t);

            %disp([x, y, theta, wi, wd, v, w])
            
        end

        function [v, w] = mcd(obj, wi, wd)
            % Modelo cinemático directo
            %
            % PARÁMETROS
            % wi: velocidad angular de la rueda izquierda
            % wd: velocidad angular de la rueda izquierda
            % x0: valor de la pose en el eje x global
            % y0: valor de la pose en el eje y global
            % t: incremento de tiempo / tiempo de muestreo
            %
            % SALIDA
            % v: velocidad lineal
            % w: velocidad angular
            v = (wi + wd) * obj.R / 2;
            w = (wd - wi) * obj.R / obj.d;
        end

        function [x, y, theta] = odom(obj, v, w, x0, y0, theta0, t)
            % Calcula la siguiente pose odométrica
            %
            % PARÁMETROS
            % v: velocidad lineal
            % w: velocidad angular
            %
            % SALIDA
            % x: vector de posiciones en el eje x global
            % y: vector de posiciones en el eje y global
            % theta: vector de ángulos en el sistema de referencia global
            
            ds = v*t;
            dx = cos(theta0)*ds;
            dy = sin(theta0)*ds;
            dtheta = w*t;
            
            x = x0 + dx;
            y = y0 + dy;
            theta = theta0 + dtheta;
        end


        function plot(obj, x, y, theta)
            % Dibuja los resultados de la función obj.simulate. Dibuja un
            % plano XY con la trayectoria del robot, y un gráfico temporal
            % con la orientación del mismo.
            %
            % PARÁMETROS
            % x: vector de posiciones en el eje x global
            % y: vector de posiciones en el eje y global
            % theta: vector de ángulos en el sistema de referencia global

            % XY plot
            figure
            plot(x, y);
            xlabel('Posición en X');
            ylabel('Posición en Y');
            title('Trayectoria del Robot');
            grid on;
            axis equal;

            % THETA plot
        end
    end
end

