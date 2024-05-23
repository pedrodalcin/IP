clc
clear
close all
syms 'alfa' 'gama' 'fi' 'x0' 'y0' 'z0' real
syms 'dalfa' 'dgama' 'dfi' 'dx0' 'dy0' 'dz0' real
syms 'theta1' 'theta2' 'theta3' 'theta4' 'theta5' 'theta6' 'theta7' 'theta8' 't' real
syms 't1p' 't2p' 't3p' 't4p' 't5p' 't6p' 't7p' 't8p' real
syms 'd' 'D' 'a' 'b' 'l1' 'l2' 'betha' real
syms 'f(t)' 'g(t)' 'h(t)' 'i(t)' 'j(t)' 'k(t)' 'l(t)' 'm(t)' 'n(t)' 'o(t)' 'p(t)'
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
 N = theta7 - l(t);
 O = theta8 - m(t);
 P = x0 - n(t);
 Q = y0 - o(t);
 R = z0 - p(t);
 T = AX*AY*AZ;
%parte da frente
 phi1 = q0 + T*[-d/2;0;-D/2] -  [(l1*cos(theta1) + l2*cos(theta1+theta2))*cos(betha);(l1*sin(theta1) + l2*sin(theta1+theta2));(l1*cos(theta1) + l2*cos(theta1+theta2))*sin(betha)];
 phi3 = q0 + T*[-d/2;0;D/2] -  [(l1*cos(theta5) + l2*cos(theta5+theta6))*cos(betha);(l1*sin(theta5) + l2*sin(theta5+theta6));b + (l1*cos(theta5) + l2*cos(theta5+theta6))*sin(betha)];
%parte de trás
 phi2 = q0 + T*[d/2;0;-D/2] -[a+(l1*cos(theta3) + l2*cos(theta3+theta4))*cos(betha);(l1*sin(theta3) + l2*sin(theta3+theta4));(l1*cos(theta3) + l2*cos(theta3+theta4))*sin(betha)];
 phi4 = q0 + T*[d/2;0;D/2] - [a+(l1*cos(theta7) + l2*cos(theta7+theta8))*cos(betha);(l1*sin(theta7) + l2*sin(theta7+theta8));b+(l1*cos(theta7) + l2*cos(theta7+theta8))*sin(betha)];
 q = [x0;y0;z0;alfa;gama;fi;theta1;theta2;theta3;theta4;theta5;theta6;theta7;theta8];
 PHI = vertcat(phi1+phi3,phi2+phi4,H, I, J, K, L, M, N, O);
 PHI1 = phi1+phi3;
 PHI2 = phi2+phi4;
 nu=-diff(PHI,t);
 phitt=-diff(nu,t);
 phiq=jacobian(PHI,q);
%  DET=det(phiq);
qp =[dx0;dy0;dz0;dalfa;dgama;dfi;t1p;t2p;t3p;t4p;t5p;t6p;t7p;t8p];
gamma = -(jacobian(phiq*qp,q))*qp-2*diff(phiq,t)*qp-phitt;