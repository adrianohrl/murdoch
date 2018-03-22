clc;
clear all;
close all;

% Gaussiana do Ciclo (1.5, 1.0)
desired_mean = 1.5;
desired_std = 1.0;
filename = 'cycles.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
tasks = length(data);
disp(['Size: ' num2str(length(data))]);
disp(['Mean: ' num2str(mean(data))]);
disp(['Standard deviation: ' num2str(std(data))]);
h = figure;
plot(data);
grid on;
title('Dispersão das durações entre a geração de tarefas', 'fontsize', 24);
ylabel('Duração [s]', 'fontsize', 24);
ylabel('Amostra', 'fontsize', 24);
set(gca, 'linewidth', 2, 'fontsize', 18);
saveas(h, 'disp_cycles.tex');

[counters centers] = hist(data);
counters = 100 * counters / sum(counters);
h = figure;
bar(centers, counters);
hold on;
x = linspace(centers(1), centers(end), 100);
fc = max(counters) / normpdf(desired_mean, desired_mean, desired_std);
plot(x, fc * normpdf(x, desired_mean, desired_std), '-r', 'linewidth', 3);
fc = max(counters) / normpdf(mean(data), mean(data), std(data));
plot(x, fc * normpdf(x, mean(data), std(data)), '--k', 'linewidth', 3);
grid on;
title('Histograma da duração entre a geração de tarefas', 'fontsize', 24);
ylabel('Frequência [\%]', 'fontsize', 24);
xlabel('Duração [s]', 'fontsize', 24);
set(gca, 'linewidth', 2, 'fontsize', 18);
saveas(h, 'hist_cycles.tex');

% Gaussiana do Número de Waypoints (4, 2)
desired_mean = 4;
desired_std = 2;
filename = 'waypoints.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['Size: ' num2str(length(data))]);
disp(['Mean: ' num2str(mean(data))]);
disp(['Standard deviation: ' num2str(std(data))]);
h = figure;
plot(data);
grid on;
title('Dispersão do número de pontos de passagem gerados', 'fontsize', 24);
ylabel('Quantidade [un]', 'fontsize', 24);
xlabel('Amostra', 'fontsize', 24);
set(gca, 'linewidth', 2, 'fontsize', 18);
saveas(h, 'disp_waypoints.tex');

[counters centers] = hist(data);
counters = 100 * counters / sum(counters);
h = figure;
bar(centers, counters);
hold on;
x = linspace(centers(1), centers(end), 100);
fc = max(counters) / normpdf(desired_mean, desired_mean, desired_std);
plot(x, fc * normpdf(x, desired_mean, desired_std), '-r', 'linewidth', 3);
fc = max(counters) / normpdf(mean(data), mean(data), std(data));
plot(x, fc * normpdf(x, mean(data), std(data)), '--k', 'linewidth', 3);
grid on;
title('Histograma do número de pontos de passagem gerados', 'fontsize', 24);
ylabel('Frequência [\%]', 'fontsize', 24);
xlabel('Quantidade [un]', 'fontsize', 24);
set(gca, 'linewidth', 2, 'fontsize', 18);
saveas(h, 'hist_waypoints.tex');

% Gaussiana da Coordenada das abcissas (0.0, 10.0)
desired_mean = 0.0;
desired_std = 10.0;
filename = 'waypoints-x.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['Size: ' num2str(length(data))]);
disp(['Mean: ' num2str(mean(data))]);
disp(['Standard deviation: ' num2str(std(data))]);
h = figure;
plot(data);
grid on;
title('Dispersão da coordenada das abcissas', 'fontsize', 24);
ylabel('Coordenada [m]', 'fontsize', 24);
xlabel('Amostra', 'fontsize', 24);
set(gca, 'linewidth', 2, 'fontsize', 18);
saveas(h, 'disp_waypoints_x.tex');

[counters centers] = hist(data);
counters = 100 * counters / sum(counters);
h = figure;
bar(centers, counters);
hold on;
x = linspace(centers(1), centers(end), 100);
fc = max(counters) / normpdf(desired_mean, desired_mean, desired_std);
plot(x, fc * normpdf(x, desired_mean, desired_std), '-r', 'linewidth', 3);
fc = max(counters) / normpdf(mean(data), mean(data), std(data));
plot(x, fc * normpdf(x, mean(data), std(data)), '--k', 'linewidth', 3);
grid on;
title('Histograma da coordenada das abcissas', 'fontsize', 24);
ylabel('Frequência [\%]', 'fontsize', 24);
xlabel('Coordenada [m]', 'fontsize', 24);
set(gca, 'linewidth', 2, 'fontsize', 18);
saveas(h, 'hist_waypoints_x.tex');

% Gaussiana da Coordenada das ordenadas (0.0, 15.0)
desired_mean = 0.0;
desired_std = 15.0;
filename = 'waypoints-y.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['Size: ' num2str(length(data))]);
disp(['Mean: ' num2str(mean(data))]);
disp(['Standard deviation: ' num2str(std(data))]);
h = figure;
plot(data);
grid on;
title('Dispersão da coordenada das ordenadas', 'fontsize', 24);
ylabel('Coordenada [m]', 'fontsize', 24);
xlabel('Amostra', 'fontsize', 24);
set(gca, 'linewidth', 2, 'fontsize', 18);
saveas(h, 'disp_waypoints_y.tex');

[counters centers] = hist(data);
counters = 100 * counters / sum(counters);
h = figure;
bar(centers, counters);
hold on;
x = linspace(centers(1), centers(end), 100);
fc = max(counters) / normpdf(desired_mean, desired_mean, desired_std);
plot(x, fc * normpdf(x, desired_mean, desired_std), '-r', 'linewidth', 3);
fc = max(counters) / normpdf(mean(data), mean(data), std(data));
plot(x, fc * normpdf(x, mean(data), std(data)), '--k', 'linewidth', 3);
grid on;
title('Histograma da coordenada das ordenadas', 'fontsize', 24);
ylabel('Frequência [\%]', 'fontsize', 24);
xlabel('Coordenada [m]', 'fontsize', 24);
set(gca, 'linewidth', 2, 'fontsize', 18);
saveas(h, 'hist_waypoints_y.tex');

% Probabilidade do skill 'camera' (0.75, unário)
filename = 'skills-camera.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['Size: ' num2str(length(data))]);
disp(['O recurso camera foi requisitado ' num2str(100 * length(data) / tasks) ' [\%] das vezes que uma tarefa foi gerada.']);

% Probabilidade do skill 'battery' (0.7, contínuo)
filename = 'skills-battery.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['Size: ' num2str(length(data))]);
disp(['O recurso battery foi requisitado ' num2str(100 * length(data) / tasks) ' [\%] das vezes que uma tarefa foi gerada.']);

% Gaussiana da quantidade necessária de bateria (0.6, 0.2)
desired_mean = 0.6;
desired_std = 0.2;
filename = 'skills-level-battery.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['Size: ' num2str(length(data))]);
disp(['Mean: ' num2str(mean(data))]);
disp(['Standard deviation: ' num2str(std(data))]);
h = figure;
plot(data);
grid on;
title('Dispersão da quantidade de bateria requerida', 'fontsize', 24);
ylabel('Quantidade [\%]', 'fontsize', 24);
xlabel('Amostra', 'fontsize', 24);
set(gca, 'linewidth', 2, 'fontsize', 18);
saveas(h, 'disp_skills_battery.tex');

[counters centers] = hist(data);
counters = 100 * counters / sum(counters);
h = figure;
bar(centers, counters);
hold on;
x = linspace(centers(1), centers(end), 100);
fc = max(counters) / normpdf(desired_mean, desired_mean, desired_std);
plot(x, fc * normpdf(x, desired_mean, desired_std), '-r', 'linewidth', 3);
fc = max(counters) / normpdf(mean(data), mean(data), std(data));
plot(x, fc * normpdf(x, mean(data), std(data)), '--k', 'linewidth', 3);
grid on;
title('Histograma da quantidade de bateria requerida', 'fontsize', 24);
ylabel('Frequência [\%]', 'fontsize', 24);
xlabel('Quantidade [\%]', 'fontsize', 24);
set(gca, 'linewidth', 2, 'fontsize', 18);
saveas(h, 'hist_skills_battery.tex');

% Probabilidade do skill 'processor' (0.6, discreto)
filename = 'skills-processor.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['Size: ' num2str(length(data))]);
disp(['O recurso processor foi requisitado ' num2str(100 * length(data) / tasks) ' [\%] das vezes que uma tarefa foi gerada.']);

% Gaussiana da quantidade necessária de processador (3, 1)
desired_mean = 3;
desired_std = 1;
filename = 'skills-level-processor.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['Size: ' num2str(length(data))]);
disp(['Mean: ' num2str(mean(data))]);
disp(['Standard deviation: ' num2str(std(data))]);
h = figure;
plot(data);
grid on;
title('Dispersão da quantidade de processadores requerida', 'fontsize', 24);
ylabel('Quantidade [un]', 'fontsize', 24);
xlabel('Amostra', 'fontsize', 24);
set(gca, 'linewidth', 2, 'fontsize', 18);
saveas(h, 'disp_skills_processor.tex');

centers = min(data) : max(data);
[counters centers] = hist(data, centers, length(centers));
counters = 100 * counters / sum(counters);
h = figure;
bar(centers, counters, 'hist');
hold on;
x = linspace(centers(1), centers(end), 100);
fc = max(counters) / normpdf(desired_mean, desired_mean, desired_std);
plot(x, fc * normpdf(x, desired_mean, desired_std), '-r', 'linewidth', 3);
fc = max(counters) / normpdf(mean(data), mean(data), std(data));
plot(x, fc * normpdf(x, mean(data), std(data)), '--k', 'linewidth', 3);
grid on;
title('Histograma da quantidade de processadores requerida', 'fontsize', 24);
ylabel('Frequência [\%]', 'fontsize', 24);
xlabel('Quantidade [un]', 'fontsize', 24);
set(gca, 'linewidth', 2, 'fontsize', 18);
saveas(h, 'hist_skills_processor.tex');

% Probabilidade do skill 'strength' (0.5, contínua)
filename = 'skills-strength.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['Size: ' num2str(length(data))]);
disp(['O recurso strength foi requisitado ' num2str(100 * length(data) / tasks) ' [\%] das vezes que uma tarefa foi gerada.']);

% Gaussiana da quantidade necessária de força (5.6, 3.4)
desired_mean = 5.6;
desired_std = 3.4;
filename = 'skills-level-strength.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['Size: ' num2str(length(data))]);
disp(['Mean: ' num2str(mean(data))]);
disp(['Standard deviation: ' num2str(std(data))]);
h = figure;
plot(data);
grid on;
title('Dispersão da intensidade de força requerida', 'fontsize', 24);
ylabel('Intensidade [N]', 'fontsize', 24);
xlabel('Amostra', 'fontsize', 24);
set(gca, 'linewidth', 2, 'fontsize', 18);
saveas(h, 'disp_skills_strength.tex');

[counters centers] = hist(data);
counters = 100 * counters / sum(counters);
h = figure;
bar(centers, counters);
hold on;
x = linspace(centers(1), centers(end), 100);
fc = max(counters) / normpdf(desired_mean, desired_mean, desired_std);
plot(x, fc * normpdf(x, desired_mean, desired_std), '-r', 'linewidth', 3);
fc = max(counters) / normpdf(mean(data), mean(data), std(data));
plot(x, fc * normpdf(x, mean(data), std(data)), '--k', 'linewidth', 3);
grid on;
title('Histograma da intensidade de força requerida', 'fontsize', 24);
ylabel('Frequência [\%]', 'fontsize', 24);
xlabel('Intensidade [N]', 'fontsize', 24);
set(gca, 'linewidth', 2, 'fontsize', 18);
saveas(h, 'hist_skills_strength.tex');

% Probabilidade do skill 'laserscan' (0.35, unário)
filename = 'skills-laserscan.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['Size: ' num2str(length(data))]);
disp(['O recurso laserscan foi requisitado ' num2str(100 * length(data) / tasks) ' [\%] das vezes que uma tarefa foi gerada.']);