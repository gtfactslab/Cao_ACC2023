function cost = f(x, u, e, data, block_int)
     
    state_centers = (x(1:block_int:end, 1:6) + x(1:block_int:end, 7:end))/2;
    
    cost = sum(state_centers(:, 3));

end