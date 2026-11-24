function c = tagColor(cm, tag)
    if any(cm.ref == tag)
        c = palette("blue");            % references, orders, commands
    elseif any(cm.meas == tag)
        c = palette("green");           % measurements
    else
        c = palette("orange");          % internal signals, flags
    end
end
