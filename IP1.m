clear
clc
close all
%definindo os parametros iniciais
%tamanho da pata
l1=8;
l2=5;
%medida da plataforma
d=16.8;
D=24;
%função motora 
wa=0.4;
wb=0;
wc=0;
wd=0;
we=0.4;
wf=0;
%posições iniciais estimadas
a=(l2*cos(13*pi/30)-l1*cos(3*pi/5))*2+d;
b=D;
h=l1*sin(3*pi/5)+l2*sin(13*pi/30);
betha=0;
%intervalo de tempo
dt = 0.05;
t = 0:dt:20;
%equações de restrição
phi = @(q,t) ([ q(1) - l2*cos(q(7) + q(8)) - (D*sin(q(5)))/2 - l1*cos(q(7)) - (d*cos(q(6))*cos(q(5)))/2;
                q(2) - l2*sin(q(7) + q(8)) - l1*sin(q(7)) - (d*(cos(q(4))*sin(q(6)) + cos(q(6))*sin(q(4))*sin(q(5))))/2 + (D*cos(q(5))*sin(q(4)))/2;
                q(3) - (d*(sin(q(4))*sin(q(6)) - cos(q(4))*cos(q(6))*sin(q(5))))/2 - (D*cos(q(4))*cos(q(5)))/2;
                q(1) - l2*cos(q(9) + q(10)) + (D*sin(q(5)))/2 - l1*cos(q(9)) - (d*cos(q(6))*cos(q(5)))/2;
                q(2) - l2*sin(q(9) + q(10)) - l1*sin(q(9)) - (d*(cos(q(4))*sin(q(6)) + cos(q(6))*sin(q(4))*sin(q(5))))/2 - (D*cos(q(5))*sin(q(4)))/2;
                q(3) - b - (d*(sin(q(4))*sin(q(6)) - cos(q(4))*cos(q(6))*sin(q(5))))/2 + (D*cos(q(4))*cos(q(5)))/2;
                q(7)-0.1*sin(wa*t)-3*pi/5; ...
                q(8)-0.1*sin(wb*t)+pi/4; ...
                q(9)-0.1*sin(we*t)-3*pi/5; ...
                q(10)-0.1*sin(wf*t)+pi/4]);

fiq = @(q,t) [1, 0, 0,                                                                                     0,                       (d*cos(q(6))*sin(q(5)))/2 - (D*cos(q(5)))/2,                                  (d*cos(q(5))*sin(q(6)))/2,   l2*sin(q(7) + q(8)) + l1*sin(q(7)),  l2*sin(q(7) + q(8)),                                          0,                        0;
              0, 1, 0,   (d*(sin(q(4))*sin(q(6)) - cos(q(4))*cos(q(6))*sin(q(5))))/2 + (D*cos(q(4))*cos(q(5)))/2, - (D*sin(q(4))*sin(q(5)))/2 - (d*cos(q(6))*cos(q(5))*sin(q(4)))/2, -(d*(cos(q(4))*cos(q(6)) - sin(q(4))*sin(q(6))*sin(q(5))))/2, - l2*cos(q(7) + q(8)) - l1*cos(q(7)), -l2*cos(q(7) + q(8)),                                          0,                        0;
              0, 0, 1,   (D*cos(q(5))*sin(q(4)))/2 - (d*(cos(q(4))*sin(q(6)) + cos(q(6))*sin(q(4))*sin(q(5))))/2,   (D*cos(q(4))*sin(q(5)))/2 + (d*cos(q(4))*cos(q(6))*cos(q(5)))/2, -(d*(cos(q(6))*sin(q(4)) + cos(q(4))*sin(q(6))*sin(q(5))))/2,                                          0,                        0,                                          0,                        0;
              1, 0, 0,                                                                                     0,                       (D*cos(q(5)))/2 + (d*cos(q(6))*sin(q(5)))/2,                                  (d*cos(q(5))*sin(q(6)))/2,                                          0,                        0,   l2*sin(q(9) + q(10)) + l1*sin(q(9)),  l2*sin(q(9) + q(10));
              0, 1, 0,   (d*(sin(q(4))*sin(q(6)) - cos(q(4))*cos(q(6))*sin(q(5))))/2 - (D*cos(q(4))*cos(q(5)))/2,   (D*sin(q(4))*sin(q(5)))/2 - (d*cos(q(6))*cos(q(5))*sin(q(4)))/2, -(d*(cos(q(4))*cos(q(6)) - sin(q(4))*sin(q(6))*sin(q(5))))/2,                                          0,                        0, - l2*cos(q(9) + q(10)) - l1*cos(q(9)), -l2*cos(q(9) + q(10));
              0, 0, 1,  -(d*(cos(q(4))*sin(q(6)) + cos(q(6))*sin(q(4))*sin(q(5))))/2 - (D*cos(q(5))*sin(q(4)))/2,   (d*cos(q(4))*cos(q(6))*cos(q(5)))/2 - (D*cos(q(4))*sin(q(5)))/2, -(d*(cos(q(6))*sin(q(4)) + cos(q(4))*sin(q(6))*sin(q(5))))/2,                                          0,                        0,                                          0,                        0;
              0, 0, 0,                                                                                     0,                                                               0,                                                        0,                                          1,                        0,                                          0,                        0;
              0, 0, 0,                                                                                     0,                                                               0,                                                        0,                                          0,                        1,                                          0,                        0;
              0, 0, 0,                                                                                     0,                                                               0,                                                        0,                                          0,                        0,                                          0,                        0;
              0, 0, 0,                                                                                     0,                                                               0,                                                        0,                                          0,                        0,                                          0,                        0];

v = @(q,t)[0;0;0;0;0;0;wa;wb;we;wf];
q = zeros(10,length(t));
q(:,1) = [11.5;10;12;0;0;0;3*pi/5;-pi/4;3*pi/5;-pi/4];
qp = zeros(10,length(t));
q2p = zeros(10,length(t));
deltaq = [0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1];
%calculando os vetores q, qp e q2p para cada instante de tempo
for i = 1:length(t)-1
    deltaq = [0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1];
    tolerance = 1e-10;
    while any(deltaq > tolerance) && any(abs(phi(q(:,i),t(i))) > tolerance)
        deltaq = -pinv(fiq(q(:,i),t(i)))*phi(q(:,i),t(i));
        q(:,i) = q(:,i) + deltaq;
    end
    t(i+1) = t(i) + dt;
    q(:,i+1) = q(:,i);
end
figure(1)
plot(t,q(1,:),'b-')
figure(2)
plot(t,q(2,:),'r')
figure(3)
plot(t,q(3,:),'g',t,q(1,:),'b-',t,q(2,:),'r')
xlabel('t');
ylabel('q');
title('Posição do centroide em função do tempo');
legend('z0', 'x0', 'y0', 'Location', 'Best');
grid on;