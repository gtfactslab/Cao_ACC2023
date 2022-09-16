function disturbance = disturbance_z(z)
    noise_observations = [10, 3;
                            5, 1;
                            2, 4;
                            0, 5;
                            -2, 5;
                            -7, 1;
                            -10, 6];
    [disturbance, ~] = fit_params(noise_observations(:, 1), noise_observations(:, 2), z);
end
