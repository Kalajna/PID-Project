M = csvread('data.csv');
time_vector = linspace(0, numel(M) - 1, numel(M));

% Wyświetlenie wykresu


plot(time_vector, M);
title('Temperatura w funkcji czasu');
xlabel('Czas');
ylabel('Temperatura (°C)');

M1 = csvread('data2.csv');
time_vector = linspace(0, numel(M1) - 1, numel(M1));

% Wyświetlenie wykresu


plot(time_vector, M1);
title('Temperatura w funkcji czasu');
xlabel('Czas');
ylabel('Temperatura (°C)');
