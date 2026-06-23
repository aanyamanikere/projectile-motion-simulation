% Defining User Input Parameters
A = [0, 0, 0];  % Initial Launch Coordinates [x0, y0, z0]
D = 21;         % Horizontal travel distance to obstacle wall (m)
H = 12;         % Height of the blocking building (m)

% Run the Optimization Function (Defined at the bottom)
[best_V, best_alpha, t_span, trajectory] = calculateMinimumLaunch(A, D, H);

% Displaying Results to the Command Window
disp('OPTIMIZED SIMULATION RESULTS')
if isfinite(best_V)
    disp(['Optimized Launch Velocity (V) : ', num2str(best_V), ' m/s']);
    disp(['Optimized Launch Angle (alpha): ', num2str(best_alpha), ' degrees']);
else
    disp('No physically viable trajectory found within the boundaries');
end

% Generating the Visual Simulation Plots
if isfinite(best_V)
    % Create a single wide figure window to hold both plots side-by-side
    figure; 
    
    % 3D View
    subplot(1, 2, 1); % 1 row, 2 columns, position 1
    
    % Plotting the 3D Trajectory Curve
    plot3(trajectory.x, trajectory.z, trajectory.y, 'b-', 'LineWidth', 2.5); 
    grid on;
    hold on;
    
    % Extracting x0 from array A for building positioning
    x0 = A(1);
    % Creating z_vec and y_vec which represent width (Z) and height (Y)
    z_vec = -5 : 1 : 5; % Points from -5 to 5 meters
    y_vec =  0 : 1 : H; % Points from ground to roof height

    % Generating the 2D grid matrix planes using meshgrid
    [Z, Y] = meshgrid(z_vec, y_vec);

    % Creating the X matrix with the same size as the Y and Z
    X = zeros(size(Z)) + (x0 + D); 
    
    % Plotting the building as a 3D surface mesh
    surf(X, Z, Y, 'FaceColor', 'r'); % surf(Horizontal_X, Horizontal_Y, Vertical_Height)
    
    % Target B's Position (10m past building, 5m high)
    xB = x0 + D + 10;
    plot3(xB, 0, 5, 'g*', 'MarkerSize', 12, 'LineWidth', 2.5);
    
    % Labeling axes and Customizing the plot
    xlabel('X-Axis: Flight Distance (m)', 'FontSize', 10);
    ylabel('Z-Axis: Lateral Displacement (m)', 'FontSize', 10);
    zlabel('Y-Axis: Altitude (m)', 'FontSize', 10);
    title('3D Trajectory View', 'FontSize', 11);
    legend('Projectile Path', 'Obstacle Building', 'Target B', 'Location', 'best');
    hold off;
    
    % 2D View
    subplot(1, 2, 2); % 1 row, 2 columns, position 2
    
    % Plotting the 2D Trajectory (Flight Distance vs Altitude)
    plot(trajectory.x, trajectory.y, 'b-', 'LineWidth', 2.5);
    grid on;
    hold on;
    
    % Plotting the building wall as a simple 2D vertical line
    % From x = (x0+D) at ground level (y=0) up to roof level (y=H)
    plot([x0 + D, x0 + D], [0, H], 'r-', 'LineWidth', 4); % plot([x1, x2], [y1, y2]) ; (distances along the ground & heights above the ground)
    
    % Plotting Target B in 2D
    plot(xB, 5, 'g*', 'MarkerSize', 12, 'LineWidth', 2.5);
    
    % Customizing the 2D Panel
    xlabel('Horizontal Flight Distance (m)', 'FontSize', 10);
    ylabel('Vertical Altitude/Height (m)', 'FontSize', 10);
    title('2D Clearance View', 'FontSize', 11);
    legend('Projectile Path', 'Obstacle Wall Edge', 'Target B', 'Location', 'best');
    hold off;
end

function [best_V, best_alpha, t_span, trajectory] = calculateMinimumLaunch(A, D, H)
    g = 9.81;              % Gravitational acceleration constant (m/s^2)
    delta = 2.0;           % Clearance buffer (meters)
    
    % Unpacking user-defined coordinate inputs
    x0 = A(1);
    y0 = A(2);
    z0 = A(3);
    
    % Predefined constants for Target B
    xB = x0 + D + 10;      % Always 10 meters past the building edge
    yB = 5;                % Always 5 meters above the ground
    zB = 0;                % Always zero meters on the z-axis
    
    % Equations for horizontal distance
    x_total = sqrt((xB - x0)^2 + (zB - z0)^2); 
    beta = atan((zB - z0)/(xB - x0));            
    
    % Setting the range for optimization
    alpha_range = linspace(1, 89, 2000);
    best_V = Inf;
    best_alpha = NaN;
    
    for i = 1:length(alpha_range)
        alpha_rad = deg2rad(alpha_range(i)); % MATLAB accepts angles in radians, not degrees
        
        % Calculating V_req from formula for trajectory of a projectile
        num = g * (x_total^2);
        den = 2 * (cos(alpha_rad)^2) * (y0 + x_total*tan(alpha_rad) - yB); 
        
        if den > 0
            V_req = sqrt(num / den);
            
            % Computing the trajectory height exactly above the building 
            t_build = D / (V_req * cos(alpha_rad));
            y_build = y0 + V_req * sin(alpha_rad) * t_build - 0.5 * g * (t_build^2);
            
            % Checking building collision constraint boundaries
            if y_build >= (H + delta) % Checks the clearance constraint
                if V_req < best_V % Checks if V_req uses lower velocity than the current value saved in best_V
                    best_V = V_req; % If V_req < best_V, it is updated with the new lowest velocity found
                    best_alpha = alpha_range(i); % Saves the matching launch angle that achieved this path
                end
            end
        end
    end
    % Constructing the trajectory if a real solution was found
    if isfinite(best_V)
        alpha_final = deg2rad(best_alpha);
        T_flight = x_total / (best_V * cos(alpha_final));
        t_span = linspace(0, T_flight, 200);
        
        trajectory.x = x0 + best_V * cos(alpha_final) * cos(beta) * t_span;
        trajectory.y = y0 + best_V * sin(alpha_final) * t_span - 0.5 * g * (t_span.^2);
        trajectory.z = z0 + best_V * cos(alpha_final) * sin(beta) * t_span;
    else
        best_V = NaN;
        best_alpha = NaN;
        t_span = [];
        trajectory = [];
    end
end