function [wi,wd] = MCI2(v,w,K,R)
    % No se tienen en cuenta singularidades (v=0)
    gamma = w/v; 
    wi = v*(1-K*gamma)/R;
    wd = v*(1+K*gamma)/R;
end