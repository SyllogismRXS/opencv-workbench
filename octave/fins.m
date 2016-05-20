close all;
clear all;
clc all;

D = csvread('../../../fins.csv');
% Throw away first empty row (column titles)
D = D(2:end,:);

% Frame : 1
% ID    : 2
% X     : 3
% Y     : 4
% Age   : 5

FRAME = D(:,1);
X = D(:,3);
Y = D(:,4);

%plot(X,Y,"*");
plot3(X,Y,FRAME,"*");
