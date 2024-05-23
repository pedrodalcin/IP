clc
clear
close all
syms 'alfa' 'gama' 'fi' 'x0' 'y0' 'z0' real
syms 'dalfa' 'dgama' 'dfi' 'dx0' 'dy0' 'dz0' real
syms 'theta1' 'theta2' 'theta3' 'theta4' 'theta5' 'theta6' 't' real
syms 't1p' 't2p' 't3p' 't4p' 't5p' 't6p' real
syms 'd' 'D' 'a' 'b' 'l1' 'l2' real
syms 'f(t)' 'g(t)' 'h(t)' 'i(t)' 'j(t)' 'k(t)'
q0 = [x0;y0;z0];
AX=[1,0,0;0,cos(alfa),-sin(alfa);0,sin(alfa),cos(alfa)];
AY=[cos(gama),0,sin(gama);0,1,0;-sin(gama),0,cos(gama)];
AZ=[cos(fi),-sin(fi),0;sin(fi),cos(fi),0;0,0,1];
 H = theta1 - f(t);
 I = theta2 - g(t);
 J = theta3 - h(t);
 K = theta4 - i(t);
 L = theta5 - j(t);
 M = theta6 - k(t);
 T = AX*AY*AZ;
betha=0;
phi1 = q0 + T*[-d/2;0;-D/2] -  [(l1*cos(theta1) + l2*cos(theta1+theta2));(l1*sin(theta1) + l2*sin(theta1+theta2));0]
phi2 = q0 + T*[-d/2;0;D/2] -  [(l1*cos(theta5) + l2*cos(theta5+theta6))*cos(betha);(l1*sin(theta5) + l2*sin(theta5+theta6));b-(l1*cos(theta5) + l2*cos(theta5+theta6))*sin(betha)];
phi3 = q0 + T*[d/2;0;0] -  [a+(l1*cos(theta3) + l2*cos(theta3+theta4));(l1*sin(theta3) + l2*sin(theta3+theta4));b/2];

q = [x0;y0;z0;alfa;gama;fi;theta1;theta2;theta5;theta6];
PHI = vertcat(phi1,phi2,H,I,J,K);
nu=-diff(PHI,t);
phitt=-diff(nu,t);
phiq=jacobian(PHI,q);
DET=det(phiq);
qp =[dx0;dy0;dz0;dalfa;dgama;dfi;t1p;t2p;t3p;t4p];
gamma = -(jacobian(phiq*qp,q))*qp-2*diff(phiq,t)*qp-phitt;