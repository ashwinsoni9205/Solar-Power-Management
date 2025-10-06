function [duty,error_iL,error_iL_int2]= picontrol(Vpv,Ipv,IL,VPVref,t,clk)

persistent error_iL_int windup IL_nmin1 IL_nmin2 duty2 error_iL2;

if(t<0)
duty = 0.0;
error_iL = 0;
error_iL_int2 = 0;
else

% gains and constants:
kp = 0.5;
ki = 20;
Ts = 1e-4; %Sampling time;
I_INT_MAX = 0.7;
I_INT_MIN = -0.7;
ILref = (Vpv*Ipv)/VPVref;

% Persistent variables for integrators

if isempty(error_iL_int)
    error_iL_int = 0;
    windup = 0;
    IL_nmin1 = 0;
    IL_nmin2 = 0;
    duty2 = 0;
    error_iL2 = 0;
end

if(clk == 1)
% control loop:
error_iL = ILref + windup - ((0.6*IL)+(0.25*IL_nmin1)+(0.15*IL_nmin2)); 
error_iL2 = error_iL;
IL_nmin2 = IL_nmin1;
IL_nmin1 = IL;
error_iL_int = error_iL_int + (error_iL*Ts); 
% if (error_iL_int > I_INT_MAX)
%             error_iL_int = I_INT_MAX;
% elseif (error_iL_int < I_INT_MIN)
%             error_iL_int = I_INT_MIN;
% end
error_iL_int2 = error_iL_int;
temp = (kp*error_iL) + (ki*error_iL_int);

if temp > 0.95
    duty = 0.95;
elseif temp < 0
    duty = 0;
else 
    duty = temp;
    duty2 = duty;
end

windup = duty - temp;


% gains and constants:

% Vpv = Vpv*100000;
% Ipv = Ipv*100000;
% IL = IL*100000;
% 
% kp = 50000;
% ki = 200000;
% Ts = 1e-4; %Sampling time;
% 
% ILref = (Vpv*Ipv)/VPVref;
% 
% % Persistent variables for integrators
% persistent error_iL_int;
% if isempty(error_iL_int)
%     error_iL_int = 0;
% end
% 
% % control loop:
% error_iL = ILref - IL; 
% error_iL_int = error_iL_int + (error_iL*Ts); 
% duty = ((kp*error_iL) + (ki*error_iL_int))/(100000*100000);
% 
% if duty > 0.95
%     duty = 0.95;
% elseif duty < 0
%     duty = 0;
% end
else
    duty = duty2;
    error_iL = error_iL2;
    error_iL_int2 = error_iL_int;
end
end
end