close all
clc

load dist_percorrida.txt;
load vel_estrategia.txt;
load vel_linear.txt;
load tempo.txt;

x = dist_percorrida(:,1); % Deslocamento do robo em metros
y = vel_estrategia(:,1);  % Velocidade (em m/s) calculada pela estratégia em função da distância
z = vel_linear(:,1);      % Velocidade (em m/s) do robô
t = tempo(:,1);           % Tempo total decorrido em segundos

%plot(t, x);
%plot(x, y);

plot(t, y, 'r', t, z, 'b'); % Velocidades em função do tempo
title('Velocidades em função do tempo')
xlabel('Tempo em segundos')
ylabel('Velocidade linear em m/s')

% plot(x, y, 'r', x, z, 'b'); % Velocidades em função da distância
% title('Velocidades em função da distância')
% xlabel('Distância em metros')
% ylabel('Velocidade linear em m/s')

%plot(x, z, 'r');