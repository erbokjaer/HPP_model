%% ===================================================================
%  Small helpers for building systems
%  ===================================================================
function newSubsystem(sys)
    % empty subsystem block
    add_block("simulink/Ports & Subsystems/Subsystem", sys);
    delete_line(sys, "In1/1", "Out1/1");
    delete_block(sys + "/In1");
    delete_block(sys + "/Out1");
end
