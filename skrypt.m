M = csvread('pomiary.csv');
t = 1:2939;
m_znormalizowane = M(:) - M(1);

figure;

% Wykres przebiegu znormalizowanego i odpowiedzi skokowej obiektu
plot(t, m_znormalizowane, 'DisplayName', 'm\_znormalizowane');
hold on;

% Parametry obiektu
K = 36.5873;
T1 = 625.9053;
theta = 75.0521;

% Tworzenie transmitancji obiektu
numerator = K;
denominator = [T1, 1];
system = tf(numerator, denominator, 'InputDelay', theta);

% Symulacja odpowiedzi skokowej obiektu
t_obiektu = 0:1:3000;  % Długość symulacji
u_skok = ones(size(t_obiektu));  % Skok jednostkowy
y_obiektu = lsim(system, u_skok, t_obiektu);

% Dodanie przebiegu odpowiedzi skokowej obiektu do wykresu
plot(t_obiektu, y_obiektu, 'DisplayName', 'Odpowiedź Skokowa Obiektu');

% Legenda i etykiety osi
legend();
title('Porównanie Odpowiedzi Skokowej i m\_znormalizowane');
xlabel('Czas');
ylabel('Wartość');

Kp=T/(k*(TauC+To));
Ti=min(T, 4*(TauC+To));