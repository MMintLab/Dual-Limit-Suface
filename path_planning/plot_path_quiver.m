function [] = plot_path_quiver(waypoints, plotanimation, interplation_steps)
%PLOT_PATH_QUIVER plot planned sliding path as quiver
%   [] = PLOT_PATH_QUIVER(WP, PLOTANIMATION, INTERPOLATION_STEPS) plot quiver plot given waypoints,
%   if ANIMATION is true, will draw the animation frame by frame, otherwise
%   plot a single figure. INTERPOLATION_STEPS is how many interpolation
%   points between two adjacent waypoints

% set figure
figure
hold on 

xlabel('x/m')
ylabel('y/m')

set(findall(gcf,'-property','FontName'),'FontName','times new roman')
set(findall(gcf,'-property','FontSize'),'FontSize',12)
title('Planned sliding path')
axis equal

waypoints_shape = size(waypoints);
waypoints_len = waypoints_shape(1);
x = waypoints(:,1);
y = waypoints(:,2);
theta = waypoints(:,3);

% interpolation
interpolated_len = (waypoints_len-1) * (1+interplation_steps)+1;

x_i = interp(x, 1+interplation_steps);
x_i = x_i(1:interpolated_len);
y_i = interp(y, 1+interplation_steps);
y_i = y_i(1:interpolated_len);
theta_i = interp(theta, 1+interplation_steps);
theta_i = theta_i(1:interpolated_len);


arrow_length = 0.1 * max((max(x_i)-min(x_i)),(max(y_i)-min(y_i)));
line_width = 1*arrow_length;
arrow_size = 1*arrow_length;

x_range = [-(max(x_i)-min(x_i)*0.5+min(x_i)), (max(x_i)-min(x_i)*0.5+max(x_i))];
y_range = [-(max(y_i)-min(y_i)*0.5+min(y_i)), (max(y_i)-min(y_i)*0.5+max(y_i))];

if ~plotanimation
% plot single figure
    hold on
    for i =2:interpolated_len-1
        quiver(x_i(i), y_i(i),cos(theta_i(i))*arrow_length, sin(theta_i(i))*arrow_length, 'blue',...
        'LineWidth', line_width, 'AutoScale','on', ...
        'MaxHeadSize', arrow_size)
    end

    % plot the start pose
    quiver(x_i(1), y_i(1),cos(theta_i(1))*arrow_length, sin(theta_i(1))*arrow_length, 'Color',[0.4 0.75 0.4],...
        'LineWidth', 2.5 * line_width, 'AutoScale','on', ...
        'MaxHeadSize', arrow_size)
    
    % plot the goal pose
    quiver(x_i(end), y_i(end),cos(theta_i(end))*arrow_length, sin(theta_i(end))*arrow_length, 'red',...
        'LineWidth', 2.5 * line_width, 'AutoScale','on', ...
        'MaxHeadSize',arrow_size)
    
    xlim(x_range);
    ylim(y_range);
    last_i = length(x_i);
    plot(x_i(1:last_i),y_i(1:last_i),'k', 'LineWidth',1);
    
    box on

else
% animate the planned path
    hold on
    box on
    xlim(x_range);
    ylim(y_range);
    % plot the start pose
    quiver(x_i(1), y_i(1),cos(theta_i(1))*arrow_length, sin(theta_i(1))*arrow_length, 'Color',[0.4 0.75 0.4],...
        'LineWidth', 2.5 * line_width, 'AutoScale','on', ...
        'MaxHeadSize', arrow_size)
    
    % plot the goal pose
    quiver(x_i(end), y_i(end),cos(theta_i(end))*arrow_length, sin(theta_i(end))*arrow_length, 'red',...
        'LineWidth', 2.5 * line_width, 'AutoScale','on', ...
        'MaxHeadSize',arrow_size)
    drawnow;
    pause(0.1)
    

    for i = 2:interpolated_len-1
        quiver(x_i(i), y_i(i),cos(theta_i(i))*arrow_length, sin(theta_i(i))*arrow_length, 'blue',...
        'LineWidth', line_width, 'AutoScale','on', ...
        'MaxHeadSize', arrow_size)
        plot(x_i(i-1:i),y_i(i-1:i),'k', 'LineWidth',1);
        quiver(x_i(1), y_i(1),cos(theta_i(1))*arrow_length, sin(theta_i(1))*arrow_length, 'Color',[0.4 0.75 0.4],...
        'LineWidth', 2.5 * line_width, 'AutoScale','on', ...
        'MaxHeadSize', arrow_size)
    
        % plot the goal pose
        quiver(x_i(end), y_i(end),cos(theta_i(end))*arrow_length, sin(theta_i(end))*arrow_length, 'red',...
        'LineWidth', 2.5 * line_width, 'AutoScale','on', ...
        'MaxHeadSize',arrow_size)
        drawnow;
        pause(0.1)

    end
    

    last_i = length(x_i);
    plot(x_i(last_i-1:last_i),y_i(last_i-1:last_i),'k', 'LineWidth',1);
    drawnow;
    pause(0.1)


end

end