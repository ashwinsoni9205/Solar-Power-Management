function Vref = fcn(V,I,t)

Vrefmax=200;
Vrefmin=0;
Vrefinit= 69;
deltaVref=0.1;
persistent Vold Pold Vrefold;

% if(t<2)
% Vold = 0;
% Pold = 0;
% Vrefold = 0;
% Vref = 69;

% else

if isempty(Vold)
    Vold=V;
    Pold=V*I;
    Vrefold=Vrefinit;
end
P=V*I;
dV=V-Vold;
dP=P-Pold;

if dP ~= 0
    if dP>0
        if dV>0
            Vref = Vrefold + deltaVref;
        else
            Vref = Vrefold - deltaVref;
        end
    else
            if dV>0
                Vref=Vrefold - deltaVref;
            else
                Vref=Vrefold + deltaVref;
            end
     end
    
else 
    Vref = Vrefold;
end
    if Vref>=Vrefmax
        Vref=Vrefmax;
    elseif Vref <= Vrefmin
        Vref = Vrefmin;
    end
    Vrefold=Vref;
    Vold=V;
    Pold=P;
end
         
 
