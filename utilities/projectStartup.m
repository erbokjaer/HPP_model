%PROJECTSTARTUP  Runs automatically when HPP_model.prj is opened.
%   Loads the default parameter struct into the base workspace so the
%   models can be opened and simulated interactively straight away.
%   Batch/HPC runs do NOT rely on this - they pass p via SimulationInput.

p = HPP_parameters();
fprintf("HPP_model: loaded default parameters into 'p' (variant: %s)\n", p.meta.variant);
