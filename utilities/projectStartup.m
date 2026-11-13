%PROJECTSTARTUP  Runs automatically when HPP_model.prj is opened.
%   Loads the default parameter struct into the base workspace so the
%   models can be opened and simulated interactively straight away.
%   Batch/HPC runs do NOT rely on this - they pass p via SimulationInput.

p = HPP_parameters();
fprintf("HPP_model: loaded default parameters into 'p' (variant: %s)\n", p.meta.variant);

% Close any open Editor tabs, then open the entry script and README
% (interactive sessions only, so batch/HPC runs are unaffected).
% close() prompts to save modified files rather than discarding changes.
if usejava("desktop")
    close(matlab.desktop.editor.getAll);
    root = currentProject().RootFolder;
    edit(fullfile(root, "README.md"));
    edit(fullfile(root, "main.m"));
end
