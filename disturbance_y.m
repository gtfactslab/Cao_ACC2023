function disturbance = disturbance_y(z)
    noise_observations = [-3, 3;
                            0, 1;
                            2, 4;
                            5, 4;
                            8, 5;
                            10, 3];
    [disturbance, ~] = fit_params(noise_observations(:, 1), noise_observations(:, 2), z);

end