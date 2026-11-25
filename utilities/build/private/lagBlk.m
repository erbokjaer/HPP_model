function lagBlk(sys, name, T, y0)
    % first-order lag y(k) = p*y(k-1) + (1-p)*u(k), p = 1 - Ts/T, initial output y0
    nb(sys, "lag", name, "PoleZ", "1 - Ts/" + T, "ICPrevOutput", y0);
end
