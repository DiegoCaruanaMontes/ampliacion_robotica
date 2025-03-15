function feat_rem=remove_landmarks(dlocal)
global XX PX

XXtemp=[];
PXtemp=[];
len= length(XX);
feat_rem=[];

for i = 4:2:len   % recorrer todas las marcas
if ((XX(i)<(XX(1)-dlocal)) || (XX(i)>(XX(1)+dlocal)) || (XX(i+1)<(XX(2)-dlocal)) || (XX(i+1)>(XX(2)+dlocal)))
     % marca fuera del mapa local
disp('se elimina marca ')
     disp(i)
     
       XXtemp=[XX(1:i-1);XX(i+2:len)];
       PXtemp=[PX(:,1:i-1) PX(:,i+2:length(PX))];
       PXtemp=[PXtemp(1:i-1,:); PXtemp(i+2:length(PX),:)];
       feat_rem=[feat_rem; XX(i) XX(i+1)];
    end
end
if (length(feat_rem)>0)
     XX=XXtemp;
     PX=PXtemp;
     disp('Marca/s eliminada/s ');
     disp(feat_rem)
end
