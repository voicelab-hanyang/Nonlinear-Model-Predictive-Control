%% RL Training Rewards Analysis - Simple Plot
% Plot training rewards with moving average and 95% confidence bounds only

%% Load reward data
rewards = res.rewards;
fprintf('Loaded %d episode rewards\n', length(rewards));

%% Analysis Parameters
window_size = 50; % Moving average window
confidence_level = 0.95; % Confidence interval (95%)

%% Calculate statistics
episodes = 1:length(rewards);

% Moving average
moving_avg = movmean(rewards, window_size);

% Moving standard deviation for confidence bounds
moving_std = movstd(rewards, window_size);
alpha = 1 - confidence_level;
z_score = norminv(1 - alpha/2); % For 95% CI, z = 1.96

% Confidence bounds
upper_bound = moving_avg + z_score * moving_std / sqrt(window_size);
lower_bound = moving_avg - z_score * moving_std / sqrt(window_size);

%% Create plot
figure('Position', [100, 100, 1200, 600]);
hold on;

% Plot confidence bounds
fill([episodes, fliplr(episodes)], [upper_bound', fliplr(lower_bound')], ...
     [0.8, 0.9, 1], 'EdgeColor', 'none', 'FaceAlpha', 0.3, 'DisplayName', '95% Confidence Interval');

% Plot raw rewards (light gray)
plot(episodes, rewards, 'Color', [0.7, 0.7, 0.7], 'LineWidth', 0.5, ...
     'DisplayName', 'Episode Rewards');

% Plot moving average
plot(episodes, moving_avg, 'b-', 'LineWidth', 2, 'DisplayName', ...
     sprintf('Moving Average (N=%d)', window_size));

xlabel('Episode', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Episode Reward', 'FontSize', 12, 'FontWeight', 'bold');
title('Deep Q-Learning Training Progress', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontSize', 11);
grid on;
xlim([1, length(rewards)]);

% Print basic statistics
fprintf('\n=== Training Summary ===\n');
fprintf('Total Episodes: %d\n', length(rewards));
fprintf('Final Moving Average: %.2f\n', moving_avg(end));
fprintf('Overall Mean: %.2f\n', mean(rewards));

fprintf('\nPlot created!\n');