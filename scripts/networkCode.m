function code = networkCode(name)
%NETWORKCODE  Variant code of the network model in HPP.slx.
%   networkCode("nodal")    -> 1   nodal solver Z = inv(Y): fast
%   networkCode("simscape") -> 2   Simscape Electrical in frequency-and-time
%                                  mode: component-wise, about 5 times slower
%   networkCode("dq")       -> 3   linear state-space network in the dq frame
%                                  (networkDqData): electromagnetic dynamics
%   The Network block in HPP.slx is a Variant Subsystem; the variant control
%   variable HPP_network takes this code (runCase sets it per run).
    arguments
        name (1,1) string {mustBeMember(name, ["nodal", "simscape", "dq"])}
    end
    code = find(name == ["nodal", "simscape", "dq"]);
end
