clear allvars;

<<<<<<< Updated upstream
J = 0.005;  %inertia
D = 0.026;  %Damping
Kt = 1.2;   %Torque transform ratio
wp = 20;    %Pole 
=======
J = 0.0052;  %inertia
D = 0.034;  %Damping
Kt = 1.8;   %Torque transform ratio
wp = 50;    %Pole should be larger than that of plant
>>>>>>> Stashed changes
Ku = 10;    %limit Kp gain
Pu = 0.2;   %Time period at Ku
SigNum = 3; %Significant number


%% design PD
[Cpd,CoPD] = designPD(Kt,J,D,wp);

tau = 1/CoPD.a0;
Kp = CoPD.b0*tau;
Kd = (CoPD.b1-Kp)*tau;

disp(strcat('PD Gain: [tau, Kp, Kd]=[', num2str(round(tau,SigNum,'significant')),',', num2str(round(Kp,SigNum,'significant')),',', num2str(round(Kd,SigNum,'significant')),']'));


%% design PID

[Cpid,CoPID] = designPID(Kt,J,D,wp);

tau = 1/CoPID.a1;
Ki = CoPID.b0*tau;
Kp = (CoPID.b1-Ki)*tau;
Kd = (CoPID.b2-Kp)*tau;

disp(strcat('PID Gain: [tau, Kp, Kd, Ki]=[', num2str(round(tau,SigNum,'significant')),', ', num2str(round(Kp,SigNum,'significant')),',', num2str(round(Kd,SigNum,'significant')),',', num2str(round(Ki,SigNum,'significant')),']'));

%% design PID by Ziegler-Nichols

disp(strcat('PID Gain by ZN: [Kp, Kd, Ki]=[', num2str(round(0.5*Ku,SigNum,'significant')),',', num2str(round(2/Pu,SigNum,'significant')),',', num2str(round(Pu/8,SigNum,'significant')),']'));


<<<<<<< Updated upstream
=======
%% evaluation of controller

s=tf('s');
P=1*Kt/(0.61*J*s^2+1.1*D*s);

% figure(1);
% step(Cpid*P/(1+Cpid*P),Cpd*P/(1+Cpd*P));
% legend({'PID','PD'});
% hold on;
%
figure(2);
nyquist(Cpd*P);
%xlim([-3 1]);
%ylim([-5 5]);
legend({'PID','PD'});
hold on;
%}
>>>>>>> Stashed changes

function [Cpd,Coeff] = designPD(Kt,J,D,wp)

d = D/J;
K = Kt/J;

Spd =  [1 0 0 0;
        d 1 0 0;
        0 d K 0;
        0 0 0 K;];

bpd = [1; 3*wp; 3*wp^2; wp^3];

PDpara = Spd\bpd;
Coeff.a0 = PDpara(2);
Coeff.b1 = PDpara(3);
Coeff.b0 = PDpara(4);


s = tf('s');
Cpd = (Coeff.b1*s + Coeff.b0)/(s + Coeff.a0);

end

function [Cpid,Coeff] = designPID(Kt,J,D,wp)

Kt = Kt/J;
d = D/J;
k = 0;

Spid = [1 0 0 0 0;
        d 1 0 0 0;
        k d Kt 0 0;
        0 k 0 Kt 0;
        0 0 0 0 Kt;];

bpid = [1; 4*wp; 6*wp^2; 4*wp^3; wp^4];

PIDpara = Spid\bpid;
Coeff.a1 = PIDpara(2);
Coeff.b2 = PIDpara(3);
Coeff.b1 = PIDpara(4);
Coeff.b0 = PIDpara(5);

s = tf('s');
Cpid = (Coeff.b2*s^2 + Coeff.b1*s + Coeff.b0)/(s^2 + Coeff.a1*s);

end