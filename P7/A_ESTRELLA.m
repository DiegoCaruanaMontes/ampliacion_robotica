function [coste,ruta]=A_ESTRELLA(G,H,node_start,node_end)
% Inicializar
ruta = [];
coste = Inf;

n = size(G,1);
pila = zeros(n,4); % Nodo | Coste real de transición | Coste estimado desde S | Nodo anterior
pila(:,1)=1:1:n;
pila(:,2)=Inf;
pila(:,3)=Inf;


% node_start al principio
pila(node_start,:) = [];
pila = [[node_start,0,H(node_end,node_start),0];pila];

resultados = [];

while pila(1,1)~=node_end
    columns = find(G(pila(1,1),:)~=0);
    for c = columns
        %disp(c)
        k = G(pila(1,1),c);
        h = H(node_end,c);
        idx1 = find(pila(:,1)==pila(1,1));
        idx2 = find(pila(:,1)==c);
        
        pila(idx2,2) = pila(idx1,2)+k;% incremento
        pila(idx2,3) = k+h;
        % añadir nodo anterior
        pila(idx2,4) = pila(idx1,1); 
    end
    pila
    resultados = [resultados;pila(1,:)]
    pila(1,:) = [];
    
    pila = sortrows(pila,3);
end
resultados = [resultados;pila(1,:)]

node = node_end;
idx = find(resultados(:,1)==node);
coste = resultados(idx,2);
ruta = [node ruta];
while node ~= node_start
    idx = find(resultados(:,1)==node);
    node = resultados(idx,4);
    ruta = [node ruta];
end

end
