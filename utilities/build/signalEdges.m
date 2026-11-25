function E = signalEdges(sys)
%SIGNALEDGES Signal connections of one system level, Goto / From resolved.
%   E = signalEdges(sys) lists "src:port -> dst:type:port" for every
%   connection (a From counts as its Goto's source; Goto inputs left out).
%   Two layouts of the same model have the same list.
sys = string(getfullname(sys));
L = find_system(sys, "SearchDepth", 1, "FindAll", "on", "Type", "line");
raw = strings(0, 4);  gsrc = dictionary(string.empty, string.empty);
for l = L(:)'
    sp = get_param(l, "SrcPortHandle");  if sp < 0, continue, end
    sb = string(get_param(get_param(sp, "Parent"), "Name"));
    s = sb + ":" + get_param(sp, "PortNumber");
    for dp = get_param(l, "DstPortHandle")'
        if dp < 0, continue, end
        db = get_param(dp, "Parent");
        d = string(get_param(db, "Name")) + ":" + get_param(dp, "PortType") + ":" + get_param(dp, "PortNumber");
        if get_param(db, "BlockType") == "Goto"
            gsrc(string(get_param(db, "GotoTag"))) = s;
        else
            raw(end+1, :) = [s, d, sb, string(get_param(sys + "/" + sb, "BlockType"))]; %#ok<AGROW>
        end
    end
end
E = strings(0, 1);
for k = 1:size(raw, 1)
    s = raw(k, 1);
    if raw(k, 4) == "From"
        t = string(get_param(sys + "/" + raw(k, 3), "GotoTag"));
        if isKey(gsrc, t), s = gsrc(t); end
    end
    E(end+1, 1) = s + " -> " + raw(k, 2); %#ok<AGROW>
end
E = unique(E);
end
