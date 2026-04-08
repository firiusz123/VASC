%% =========================================================================
% Zadanie eliminacyjne konkursu RCDC - Skrypt Inicjalizacyjny
% Zespół (nazwa)
% =========================================================================

%% 1. Parametry początkowe symulacji
v_ego = 40;        % [m/s] Prędkość początkowa naszego pojazdu
v_lead = 20;       % [m/s] Prędkość pojazdu poprzedzającego 
d_init = 25;      % [m] Dystans początkowy.

%% 2. Parametry czujników i filtru 
radar_max_range = 150;  % [m] Zasięg przedniego radaru 
fs = 100;               % [Hz] Częstotliwość próbkowania
tau_radar = 0.02;       % [s] Stała czasowa filtru radaru. 

%% 3. Inne parametry
minimal_breaking_time_buffer = 3;    % [s] Krytyczny czas do kolizji 
max_deceleration = 8;
sim_time = 40;
%% AED PARAMETERS
AED_overtake = 0;
AED_overtake = boolean(AED_overtake);
decc_max = -8;
decc_min = -8;
decc_lead_max = -7;

%% 4. Otwarcie i uruchomienie modelu
%open_system("ADAS_System");
%%
%%load("fuzzy_1.mat");
%%
out = sim("ADAS_System");


%%
sim1 = out;
t = sim1.tout;
dist = sim1.dist.Data;
ttc = sim1.ttc.Data;
v_ego_data = sim1.v_ego.Data;
breakingforce = sim1.breakingforce.Data;
distF = out.distF.Data;

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
hold on;
v_lead_1 = ones(1,length(t));
v_lead_1 = v_lead_1 * v_lead;
plot(t,v_lead_1)
grid on;
hold off;

subplot(2,2,4)
breakingforce = squeeze(breakingforce);
plot(t,breakingforce)
title("breaking force")
xlabel("Time [s]")
ylabel("normalized breaking force  [0..1]")
grid on;
%%
figure;
plot(t,dist);
title("Relative distance vs Real Breaking distance")
xlabel("Time [s]")
ylabel("distance [m]")
grid on;
hold on;
plot(t,distF);
legend("Relative distance","Real Breaking distance")
