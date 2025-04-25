classdef Motor
    % Representación de un motor como sistema de primer orden discreto
    
    properties
        a double % Parámetro para las exponenciales del modelo
        w_max   double % Velocidad máxima permitida (rad/s)
    end
    
    methods
        function obj = Motor(Tw, v_max)
            %Constructor
            % Tw: constante de tiempo del sistema de primer orden
            obj.a = 1/Tw;
        end
       
        function w = get_w(obj, w0, wd, t)
            % Calcula la velocidad angular del motor a partir de la
            % velocidad angular anterior, la velocidad angular deseada y el
            % tiempo de muestreo.
            %
            % PARÁMETROS
            % w_ant: velocidad angular anterior
            % wd: velocidad angular deseada
            % t: tiempo de muestreo
            %
            % SALIDA
            % w: velocidad angular en el siguiente instante de tiempo
            
            % Solución de la ecuación en diferencias de primer orden
            w = exp(-obj.a*t)*w0 + (1-exp(-obj.a*t))*wd;
            % Saturación a máxima velocidad
            w = min(abs(w), obj.w_max)*sign(w);
        end
    end
end

