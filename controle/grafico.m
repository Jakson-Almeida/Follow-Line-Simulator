close all
clc

load dist_percorrida.txt;
load vel_estrategia.txt;
load vel_linear.txt;
load tempo.txt;

x = dist_percorrida(:,1); % Deslocamento do robo em metros
y = vel_estrategia(:,1);  % Velocidade (em m/s) calculada pela estrat�gia em fun��o da dist�ncia
z = vel_linear(:,1);      % Velocidade (em m/s) do rob�
t = tempo(:,1);           % Tempo total decorrido em segundos

%plot(t, x);
%plot(x, y);

plot(t, y, 'r', t, z, 'b'); % Velocidades em fun��o do tempo
title('Velocidades em fun��o do tempo')
xlabel('Tempo em segundos')
ylabel('Velocidade linear em m/s')

% plot(x, y, 'r', x, z, 'b'); % Velocidades em fun��o da dist�ncia
% title('Velocidades em fun��o da dist�ncia')
% xlabel('Dist�ncia em metros')
% ylabel('Velocidade linear em m/s')

%plot(x, z, 'r');