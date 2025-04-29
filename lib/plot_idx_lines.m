function plot_idx_lines(idx, time, input_linewidth)
    % plot_idx_lines: Plot vertical dashed lines at specified time indices.
    %
    %   plot_idx_lines(idx, time, input_linewidth)
    %
    %   Inputs:
    %       idx             - Vector of indices; each index corresponds to an element in the 'time' vector.
    %       time            - A vector of time values.
    %       input_linewidth - Scalar that specifies the line width for the vertical lines.
    %
    % The function loops through each provided index in 'idx' and plots a vertical dashed
    % black line ('--k') at the corresponding time value using the xline function.
    
    for i = idx
        % Plot a vertical dashed line at the time corresponding to index 'i'
        xline(time(i), '--k', 'LineWidth', input_linewidth);
    end
end
