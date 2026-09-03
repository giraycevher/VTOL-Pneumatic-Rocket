function pattern = pwm_quantize(u_normalized, pwm_patterns, pwm_levels)

    % 0-1 araligina zorla
    u_clamp = max(0, min(1, u_normalized));

    % En yakin PWM seviyesini bul
    [~, idx] = min(abs(pwm_levels - u_clamp));

    % Ilgili pattern'i sec
    pattern = pwm_patterns(idx, :);

end