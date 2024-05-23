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
wa=0.2;
wb=0.4;
wc=0.2;
wd=0.4;
we=0.2;
wf=0.4;
wg=0.2;
wh=0.4;
%posições iniciais estimadas
a=(-l1*cos(3*pi/5)+l2*cos(7*pi/20))*2+d;
b=D;
h=l2*sin(7*pi/20)+l1*sin(3*pi/5);
betha=pi/10;
%intervalo de tempo
dt = 0.05;
t = 0:dt:60;
%equações de restrição
phi = @(q,t) ([ 2*q(1) - cos(betha)*(l2*cos(q(7) + q(8)) + l1*cos(q(7))) - cos(betha)*(l2*cos(q(11) + q(12)) + l1*cos(q(11))) - d*cos(q(6))*cos(q(5));
                2*q(2) - l2*sin(q(7) + q(8)) - l2*sin(q(11) + q(12)) - l1*sin(q(7)) - l1*sin(q(11)) - d*(cos(q(4))*sin(q(6)) + cos(q(6))*sin(q(4))*sin(q(5)));
                2*q(3) - b - sin(betha)*(l2*cos(q(7) + q(8)) + l1*cos(q(7))) - sin(betha)*(l2*cos(q(11) + q(12)) + l1*cos(q(11))) - d*(sin(q(4))*sin(q(6)) - cos(q(4))*cos(q(6))*sin(q(5)));
                2*q(1) - 2*a - cos(betha)*(l2*cos(q(9) + q(10)) + l1*cos(q(9))) - cos(betha)*(l2*cos(q(13) + q(14)) + l1*cos(q(13))) + d*cos(q(6))*cos(q(5));
                2*q(2) - l2*sin(q(9) + q(10)) - l2*sin(q(13) + q(14)) - l1*sin(q(9)) - l1*sin(q(13)) + d*(cos(q(4))*sin(q(6)) + cos(q(6))*sin(q(4))*sin(q(5)));
                2*q(3) - b - sin(betha)*(l2*cos(q(9) + q(10)) + l1*cos(q(9))) - sin(betha)*(l2*cos(q(13) + q(14)) + l1*cos(q(13))) + d*(sin(q(4))*sin(q(6)) - cos(q(4))*cos(q(6))*sin(q(5)));
                q(7)-0.1*sin(wa*t)-3*pi/5; ...
                q(8)-0.1*sin(wb*t)+pi/4; ...
                q(9)+0.1*sin(wc*t)-2*pi/5; ...
                q(10)+0.1*sin(wd*t)-pi/4; ...
                q(11)-0.1*sin(we*t)-3*pi/5; ...
                q(12)-0.1*sin(wf*t)+pi/4; ...
                q(13)+0.1*sin(wg*t)-2*pi/5; ...
                q(14)+0.1*sin(wh*t)-pi/4]);

fiq = @(q,t) [2, 0, 0,                                                    0,            d*cos(q(6))*sin(q(5)),                                  d*cos(q(5))*sin(q(6)), cos(betha)*(l2*sin(q(7) + q(8)) + l1*sin(q(7))), l2*sin(q(7) + q(8))*cos(betha),                                                     0,                                  0, cos(betha)*(l2*sin(q(11) + q(12)) + l1*sin(q(11))), l2*sin(q(11) + q(12))*cos(betha),                                                     0,                                  0;
              0, 2, 0,  d*(sin(q(4))*sin(q(6)) - cos(q(4))*cos(q(6))*sin(q(5))), -d*cos(q(6))*cos(q(5))*sin(q(4)), -d*(cos(q(4))*cos(q(6)) - sin(q(4))*sin(q(6))*sin(q(5))),            - l2*cos(q(7) + q(8)) - l1*cos(q(7)),           -l2*cos(q(7) + q(8)),                                                     0,                                  0,            - l2*cos(q(11) + q(12)) - l1*cos(q(11)),           -l2*cos(q(11) + q(12)),                                                     0,                                  0;
              0, 0, 2, -d*(cos(q(4))*sin(q(6)) + cos(q(6))*sin(q(4))*sin(q(5))),  d*cos(q(4))*cos(q(6))*cos(q(5)), -d*(cos(q(6))*sin(q(4)) + cos(q(4))*sin(q(6))*sin(q(5))), sin(betha)*(l2*sin(q(7) + q(8)) + l1*sin(q(7))), l2*sin(q(7) + q(8))*sin(betha),                                                     0,                                  0, sin(betha)*(l2*sin(q(11) + q(12)) + l1*sin(q(11))), l2*sin(q(11) + q(12))*sin(betha),                                                     0,                                  0;
              2, 0, 0,                                                    0,           -d*cos(q(6))*sin(q(5)),                                 -d*cos(q(5))*sin(q(6)),                             0,                                  0,                    (l2*sin(q(9) + q(10))+l1*sin(q(9)))*cos(betha), l2*sin(q(9) + q(10))*cos(betha),                                                     0,                                  0, cos(betha)*(l2*sin(q(13) + q(14)) + l1*sin(q(13))), l2*sin(q(13) + q(14))*cos(betha);
              0, 2, 0, -d*(sin(q(4))*sin(q(6)) - cos(q(4))*cos(q(6))*sin(q(5))),  d*cos(q(6))*cos(q(5))*sin(q(4)),  d*(cos(q(4))*cos(q(6)) - sin(q(4))*sin(q(6))*sin(q(5))),                                                     0,                                  0,            - l2*cos(q(9) + q(10)) - l1*cos(q(9)),           -l2*cos(q(9) + q(10)),                                                     0,                                  0,            - l2*cos(q(13) + q(14)) - l1*cos(q(13)),           -l2*cos(q(13) + q(14));
              0, 0, 2,  d*(cos(q(4))*sin(q(6)) + cos(q(6))*sin(q(4))*sin(q(5))), -d*cos(q(4))*cos(q(6))*cos(q(5)),  d*(cos(q(6))*sin(q(4)) + cos(q(4))*sin(q(6))*sin(q(5))),                                                     0,                                  0, sin(betha)*(l2*sin(q(9) + q(10)) + l1*sin(q(9))), l2*sin(q(9) + q(10))*sin(betha),                                                     0,                                  0, sin(betha)*(l2*sin(q(13) + q(14)) + l1*sin(q(13))), l2*sin(q(13) + q(14))*sin(betha);
              0, 0, 0,                                                    0,                              0,                                                    0,                                                     1,                                  0,                                                     0,                                  0,                                                     0,                                  0,                                                     0,                                  0;
              0, 0, 0,                                                    0,                              0,                                                    0,                                                     0,                                  1,                                                     0,                                  0,                                                     0,                                  0,                                                     0,                                  0;
              0, 0, 0,                                                    0,                              0,                                                    0,                                                     0,                                  0,                                                     1,                                  0,                                                     0,                                  0,                                                     0,                                  0;
              0, 0, 0,                                                    0,                              0,                                                    0,                                                     0,                                  0,                                                     0,                                  1,                                                     0,                                  0,                                                     0,                                  0;
              0, 0, 0,                                                    0,                              0,                                                    0,                                                     0,                                  0,                                                     0,                                  0,                                                     1,                                  0,                                                     0,                                  0;
              0, 0, 0,                                                    0,                              0,                                                    0,                                                     0,                                  0,                                                     0,                                  0,                                                     0,                                  1,                                                     0,                                  0;
              0, 0, 0,                                                    0,                              0,                                                    0,                                                     0,                                  0,                                                     0,                                  0,                                                     0,                                  0,                                                     1,                                  0;
              0, 0, 0,                                                    0,                              0,                                                    0,                                                     0,                                  0,                                                     0,                                  0,                                                     0,                                  0,                                                     0,                                  1];
 


% v = @(q,t)[0;0;0;0;0;0;wa;wb;we;wf];
q = zeros(14,length(t));
q(:,1) = [13.1;12;12;0;0;0;3*pi/5;-pi/4;3*pi/5;-pi/4;3*pi/5;-pi/4;3*pi/5;-pi/4];
qp = zeros(14,length(t));
q2p = zeros(14,length(t));
deltaq = [0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1];
%calculando os vetores q, qp e q2p para cada instante de tempo
for i = 1:length(t)-1
    deltaq = [0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1;0.1];
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