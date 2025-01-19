clc

load dist_percorrida.txt; % Deslocamento do robo em metros
load vel_esq.txt;         % Velocidade (em m/s) da roda esquerda
load vel_dir.txt;         % Velocidade (em m/s) da roda direita
load tempo.txt;           % Tempo total decorrido em segundos

%plot(tempo, dist_percorrida);
%plot(dist_percorrida, vel_esq);

plot(tempo, vel_esq, 'r', tempo, vel_dir, 'b'); % Velocidades em função do tempo
%plot(tempo, vel_esq);
%plot(t, v_esq, t, v_dir, tempo, vel_esq, 'g', tempo, -vel_dir, 'black');
title ('Velocidades em função do tempo');
xlabel('Tempo em segundos');
ylabel('Velocidade de rotação em m/s');

%plot(dist_percorrida, vel_esq, 'r', dist_percorrida, vel_dir, 'b'); % Velocidades em função da distância
%plot(dist_percorrida, vel_dir, 'r');