function [scale_matrix, rot_matrix] = getScaleData(CPA_data, R_safe)
    % Compute scaling parameters, rotation matrix, and collision metrics from CPA data
    % using a probabilistic PCA-based approach.
    %
    % The function uses the most recent CPA data point as the center, and computes
    % the covariance matrix of the differences relative to this point. Eigenvalue
    % decomposition is applied to determine the principal axes. Then, a cost-based
    % scaling is determined based on the spread (standard deviation) along each axis,
    % adjusted by a gain factor and capped by a threshold.
    
    % Set parameters for scaling.
    threshold = 2; 
    scale_gain = 3;
    
    % Use the most recent CPA (last column) as the reference point.
    mean_CPA = CPA_data(:, end);
    
    % Center the CPA data by subtracting the reference point.
    centered_Data = CPA_data - mean_CPA;
    
    % Compute the covariance matrix of the centered data.
    % Transpose is taken because each row represents a variable.
    cov_matrix = cov(centered_Data');
    
    % Perform eigenvalue decomposition to obtain principal axes.
    [eigenvectors, eigenvalues] = eig(cov_matrix);
    
    % Sort the eigenvalues in descending order and reorder the eigenvectors.
    [~, order] = sort(diag(eigenvalues), 'descend');
    eigenvectors = eigenvectors(:, order);
    
    % Define the rotation matrix aligning the data to its principal components.
    rot_matrix = eigenvectors';
    
    % Initialize outputs for scale factors and collision metrics.
    scale_matrix = ones([3, 1]);
    collision_threat = zeros([3, 1]);
    collision_std = zeros([3, 1]);
    
    % Proceed only if sufficient CPA data points are available.
    if size(CPA_data, 2) > 2
        % Transform the centered CPA data into the principal component coordinates.
        transformed_data = rot_matrix * centered_Data;
        
        % Compute the mean and standard deviation along each principal axis.
        % mean_transformed = mean(transformed_data, 2);
        std_transformed = std(transformed_data, 0, 2);
        
        % Loop over each dimension to compute the collision threat and scale factor.
        for i = 1:3
            % Define a normal probability density function centered at the latest
            % transformed CPA value for dimension i.
            pdf_normal = @(x) (1/(std_transformed(i)*sqrt(2*pi))) * exp(-0.5 * ((x - transformed_data(i, end))/std_transformed(i)).^2);
            
            % If variance is zero, avoid division by zero.
            if std_transformed(i) == 0
                Pt = 1;
            else
                % Integrate the PDF over the interval [-R_safe, R_safe] as a measure.
                Pt = integral(pdf_normal, -R_safe, R_safe);
            end
            
            % Assign the computed probability as the collision threat measure.
            collision_threat(i) = Pt;
            % Record the standard deviation as part of the collision metric.
            collision_std(i) = std_transformed(i);
            
            % Compute the scale factor: increase the effective radius by a factor
            % proportional to the spread and the probability, then normalize by R_safe.
            scale_matrix(i) = (R_safe + scale_gain * std_transformed(i) * Pt) / R_safe;
            % Cap the scale factor at a specified threshold.
            if scale_matrix(i) > threshold
                scale_matrix(i) = threshold;
            end
        end
    else
        % Insufficient data: return default values (no scaling, identity rotation)
        scale_matrix = [1; 1; 1];
        rot_matrix = eye(3, 3);
    end
end
