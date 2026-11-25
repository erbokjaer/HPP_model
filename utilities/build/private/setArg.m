function setArg(blk, argName, value)
    ip = get_param(blk, "InstanceParameters");
    k = strcmp({ip.Name}, argName);
    assert(any(k), "Model block %s has no argument %s", blk, argName);
    ip(k).Value = char(value);
    set_param(blk, "InstanceParameters", ip);
end
