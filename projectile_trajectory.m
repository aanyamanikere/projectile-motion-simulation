% Defining User Input Parameters
    x0 = input("Enter initial launch x coordinate: ");
    y0 = input("Enter initial launch y coordinate: ");
    z0 = input("Enter initial launch z coordinate: ");

    A = [x0,y0,z0];

% Loop goes on until the user provides valid inputs
while true
    D = input('Enter the horizontal distance in m: '); 
    H = input('Enter the height of the blocking building in m: '); 
    if D <= 0 || H <= 0
        disp('INPUT ERROR: Dimensions must be positive. Please try again.');
        continue;
    else
        break;        
    end
end

% Run the Optimization Function
[best_V, best_alpha, scenario_status, trajectory] = calculateMinimumLaunch(A, D, H);

% Generating the Visual Simulation Plots 
% Plots for CLEARING, GRAZING, and COLLIDING (Skips only if UNREACHABLE)
if ~strcmp(scenario_status, 'UNREACHABLE TARGET') && ~isempty(trajectory)
    figure; 
    
    % 3D View
    subplot(1, 2, 1); % 1 row, 2 columns, position 1

    % Plotting the 3D Trajectory Curve
    plot3(trajectory.x, trajectory.z, trajectory.y, 'b-', 'LineWidth', 2.5); 
    grid on;
    hold on;
    
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
    xlabel('X-Axis: Flight Distance (m)');
    ylabel('Z-Axis: Lateral Displacement (m)');
    zlabel('Y-Axis: Altitude (m)');
    title('3D Trajectory View');
    legend('Projectile Path', 'Obstacle Building', 'Target B', 'Location', 'best');
    hold off;
    
    % 2D View
    subplot(1, 2, 2); % 1 row, 2 columns, position 2
    plot(trajectory.x, trajectory.y, 'b-', 'LineWidth', 2.5);
    grid on;
    hold on;

    % Plotting the building wall as a simple 2D vertical line
    % From x = (x0+D) at ground level (y=0) up to roof level (y=H)
    plot([x0 + D, x0 + D], [0, H], 'r-', 'LineWidth', 4); % plot([x1, x2], [y1, y2]) ; (distances along the ground & heights above the ground)
    
    % Plotting Target B in 2D
    plot(xB, 5, 'g*', 'MarkerSize', 12, 'LineWidth', 2.5);
    
    % Labeling axes and Customizing the plot
    xlabel('Horizontal Flight Distance (m)');
    ylabel('Vertical Altitude/Height (m)');
    title('2D Clearance View');
    legend('Projectile Path', 'Obstacle Wall Edge', 'Target B', 'Location', 'best');
    axis equal; % Used for equal scaling of the X and Y axes
    hold off;
else
    disp('Launch Optimization Status: FAILED');
    disp('Reason: No flight trajectory can be generated for an unreachable target.');
end

function [best_V, best_alpha, scenario_status, trajectory] = calculateMinimumLaunch(A, D, H)
    g = 9.81;            % Gravitational acceleration constant (m/s^2)
    delta = 2;           % Clearance buffer (meters)
    
    % Unpacking user-defined coordinate inputs
    x0 = A(1);
    y0 = A(2);
    z0 = A(3);

    % Predefined constants for Target B
    xB = x0 + D + 10;   % Always 10 meters away from the building
    yB = 5;             % Always 5 meters above the ground 
    zB = 0;             %  Always zero meters on the z-axis  
    
    % Equations for horizontal distance
    x_total = sqrt((xB - x0)^2 + (zB - z0)^2); 
    beta = atan((zB - z0)/(xB - x0));            
   
    % Setting the range for optimization
    alpha_range = linspace(1, 89, 2000);

    % Initializing vectors to store data for tabular display
    V_required_vec = NaN(size(alpha_range));
    y_at_building_vec = NaN(size(alpha_range));
    
    % Initialized prevent workspace crashes
    best_V = Inf;
    best_alpha = NaN;
    best_y_build = NaN;
    
    % Trajectory Optimization Scanning Loop
    for i = 1:length(alpha_range)
        alpha_rad = deg2rad(alpha_range(i)); % MATLAB accepts angles in radians, not degrees
        
        % Calculating V_req from formula for trajectory of a projectile
        num = g * (x_total^2);
        den = 2 * (cos(alpha_rad)^2) * (y0 + x_total*tan(alpha_rad) - yB); 
        
        if den > 0
            V_req = sqrt(num / den);
            V_required_vec(i) = V_req; % Save to vector for table
            
            % Computing the trajectory height exactly above the building 
            t_build = D / (V_req * cos(alpha_rad));
            y_build = y0 + V_req * sin(alpha_rad) * t_build - 0.5 * g * (t_build^2);
            y_at_building_vec(i) = y_build;  % Save to vector for table
            
            % Captures absolute lowest velocity target match
            if V_req < best_V % Checks if V_req uses lower velocity than the current value saved in best_V
                best_V = V_req; % If V_req < best_V, it is updated with the new lowest velocity found
                best_alpha = alpha_range(i); % Saves the matching launch angle that achieved this path
                best_y_build = y_build; % best_y_build holds onto the roof clearance height of the best trajectory option
            end
        end
    end
    
    % Display Tested Linspace Parameter Matrix
    disp('-------------------------------------------------------------');
    disp('   Launch Angle     Required Velocity     Height at Building');
    disp('   ------------     -----------------     ------------------');
    for i = 1:length(alpha_range) 
        if ~isnan(V_required_vec(i)) % prevents from displaying garbage values  (NaN)
            fprintf('       %.1f\t\t\t%.1f\t\t\t%.1f\n',alpha_range(i), V_required_vec(i), y_at_building_vec(i));
        end
    end
    disp('---------------------------------------------------------------');
    
    % Scenario Classification Block
    disp('OPTIMIZED SIMULATION RESULTS:');
    disp(' ')
    disp('SCENARIO ANALYSIS CRITERIA');
    
    if isnan(best_V) || best_V == Inf
        scenario_status = 'UNREACHABLE TARGET';
        disp('Scenario Status: UNREACHABLE TARGET');
        disp('ERROR: Geometry configurations make hitting Target B mathematically impossible.');
        trajectory = [];
        return;
        
    elseif best_y_build >= (H + delta) && best_y_build < (H + delta + 5)
        scenario_status = 'GRAZING';
        disp('Scenario Status: GRAZING');
        disp('WARNING: The projectile clears the building, but with an extremely tight safety margin!');
        disp(['Optimized Launch Velocity (V) : ', num2str(best_V)]);
        disp(['Optimized Launch Angle (alpha): ', num2str(best_alpha)]);
        
    elseif best_y_build >= (H + delta + 5)
        scenario_status = 'CLEARING';
        disp('Scenario Status: CLEARING [Safe clearance margin met]');
        disp(['Optimized Launch Velocity (V) : ', num2str(best_V)]);
        disp(['Optimized Launch Angle (alpha): ', num2str(best_alpha)]);
        
    else
        scenario_status = 'COLLIDING';
        disp('Scenario Status: COLLIDING');
        disp('ERROR: The minimum required velocity path results in a building collision.');
        disp(['Optimized Launch Velocity (V) : ', num2str(best_V)]);
        disp(['Optimized Launch Angle (alpha): ', num2str(best_alpha)]);
    end
    
    % Generating the trajectory arrays so they can be visualized on screen
    alpha_final = deg2rad(best_alpha);
    T_flight = x_total / (best_V * cos(alpha_final));
    t_step = linspace(0, T_flight, 200);
    
    trajectory.x = x0 + best_V * cos(alpha_final) * cos(beta) * t_step;
    trajectory.y = y0 + best_V * sin(alpha_final) * t_step - 0.5 * g * (t_step.^2);
    trajectory.z = z0 + best_V * cos(alpha_final) * sin(beta) * t_step;
end
