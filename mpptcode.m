function Vref = fcn(V,I)

Vrefmax=200;
Vrefmin=0;
Vrefinit=200;
deltaVref=1;
persistent Vold Pold Vrefold;

if isempty(Vold)
    Vold=0;
    Pold=0;
    Vrefold=Vrefinit;
end
P=V*I;
dV=V-Vold;
dP=P-Pold;

if dP ~= 0
    if dP<0
        if dV<0
            Vref = Vrefold + deltaVref;
        else
            Vref = Vrefold - deltaVref;
        end
    else
            if dV<0
                Vref=Vrefold - deltaVref;
            else
                Vref=Vrefold + deltaVref;
            end
     end
    
else 
    Vref = Vrefold;
end
    if Vref>=Vrefmax || Vref<= Vrefmin
        Vref=Vrefold ;
    end
    Vrefold=Vref;
    Vold=V;
    Pold=P;
end
         
  
 
