function [coste,ruta]=DIJKSTRA(G,node_start,node_end)
% Inicializar
ruta = [];
coste = Inf;

n = size(G,1);
pila = zeros(n,3); % Nodo | Coste | Nodo anterior
pila(:,1)=1:1:n;
pila(:,2)=Inf;

% node_start al principio
pila(node_start,:) = [];
pila = [[node_start,0,0];pila];

resultados = [];

for i = 1:1:n
    columns = find(G(pila(1,1),:)~=0);
    for c = columns
        %disp(c)
        k = G(pila(1,1),c);
        idx1 = find(pila(:,1)==pila(1,1));
        idx2 = find(pila(:,1)==c);
        
        if pila(idx1,2)+k<pila(idx2,2)
            pila(idx2,2) = pila(idx1,2)+k;% incremento
            % añadir nodo anterior
            pila(idx2,3) = pila(idx1,1);
        end
        
    end

    % quitar nodo
    resultados = [resultados;pila(1,:)];
    pila(1,:) = [];
    % sort
    pila = sortrows(pila,2);
    
    %disp(pila)
end

%disp(resultados)
% Calcular ruta en base a resultados

node = node_end;
idx = find(resultados(:,1)==node);
coste = resultados(idx,2);
ruta = [node ruta];
while node ~= node_start
    idx = find(resultados(:,1)==node);
    node = resultados(idx,3);
    ruta = [node ruta];
end

end
