%% =========================================================================
% Zadanie eliminacyjne konkursu RCDC - Skrypt Inicjalizacyjny
% Zespół (nazwa)
% =========================================================================
clear all;
%% 1. Parametry początkowe symulacji
v_ego = 40;        % [m/s] Prędkość początkowa naszego pojazdu
v_lead = 0;       % [m/s] Prędkość pojazdu poprzedzającego 
d_init = 150;      % [m] Dystans początkowy.

%% 2. Parametry czujników i filtru 
radar_max_range = 150;  % [m] Zasięg przedniego radaru 
fs = 100;               % [Hz] Częstotliwość próbkowania
tau_radar = 0.02;       % [s] Stała czasowa filtru radaru. 

%% 3. Inne parametry
ttc_threshold = 10;    % [s] Krytyczny czas do kolizji 
sim_time = 40;

%% 4. Otwarcie i uruchomienie modelu
%open_system("ADAS_System");
%%
load("fuzzy_in_workspace.mat");
%%
out = sim("ADAS_System");


%%
sim1 = out;
t = sim1.tout;
dist = sim1.dist.Data;
ttc = sim1.ttc.Data;
v_ego_data = sim1.v_ego.Data;
breakingforce = sim1.breakingforce.Data;
%%
figure;
subplot(2,2,1)
plot(t,dist);
title("Relative distance")
xlabel("Time [s]")
ylabel("distance [m]")
grid on;

subplot(2,2,2)
plot(t,ttc)
title("TTC")
xlabel("Time [s]")
ylabel("Time-to-Colision [s]")
grid on;

subplot(2,2,3)
plot(t,v_ego_data)
title("Our car velocity")
xlabel("Time [s]")
ylabel("velocity [m/s]")
grid on;

subplot(2,2,4)
plot(t,breakingforce)
title("breaking force percent")
xlabel("Time [s]")
ylabel("force percent [m/s^2]")
grid on;
