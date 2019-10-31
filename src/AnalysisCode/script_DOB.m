%close all; clc;

s = tf('s');
P = 1/(0.0053*s^2 + 0.026*s);
Pn = 1.8/(0.005*s^2 + 0.032*s);
w = 300*2*pi; % rad/s
Q = w^2/(s^2 + 2*w*s + w^2);
%Cpd = 1.55 + 0.0882*s/(0.0087*s+1);

H = 1000e-6;

sim('block_dob');


