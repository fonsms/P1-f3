% UART_READ.m:
%
% Script para realizar una lectura de datos del ST link USB.
%
% Fecha: 09/03/20     Version: 1.0
% Copyright: Alfonso Moreno Sanz

% Borrar variables y cerrar ventanas
%clear all;
%close all;
%clc;

i = i+1;
cuenta = int2str(i+1);
delete(instrfindall);
%Leemos uart
numero_com = "COM8";
s = serialport(numero_com,19200, 'Parity','none','StopBits',1,'Timeout',240);
configureTerminator(s,35);%pongo que #
data = readline(s);
delete(s)
clear s
%Creación de file.tsv 
asteriscos  = "****************************************************************************************\n";
nombre_columnas = "k \t N \n ";
%nombre_columnas = "k \t N \t Para un valor de kd= 15 y ref =Pi\n ";
%cambiar ruta para no sobrescribir
data = strcat(asteriscos,nombre_columnas,data,asteriscos);
%ruta = strcat('./Medidas/Nuevas/Medida12V_',cuenta,'.tsv');
ruta='./Medidas/Controlador_proporcional/Nuevas/IDkp83kd17ki19ref03pi.tsv';
fichero_medida = fopen(ruta, 'w');
fprintf(fichero_medida,data);
fclose(fichero_medida);
