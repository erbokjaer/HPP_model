function [hIn, xMid] = ctrlInputs(sys, names, dims, cm, ts)
    % Inport -> Goto for every input, in a column on the left (sample
    % time ts, default "Ts")
    if nargin < 5, ts = "Ts"; end
    y = 40;  all = strings(0);  right = 0;
    for k = 1:numel(names)
        add_block("simulink/Sources/In1", sys + "/" + names(k), "Port", num2str(k), ...
            "OutDataTypeStr", "double", "PortDimensions", dims(k), "SampleTime", ts, "Position", [0 0 30 14]);
        [b, rr, bl] = rowChain(sys, 40, y, [stg(names(k), 0), stg(">" + names(k))], cm);
        y = b + 34;  all = [all, bl]; %#ok<AGROW>
        right = max(right, rr);
    end
    hIn = areaAround(sys, "Inputs", all, areaColor("interface"));
    xMid = right + 90;
end
