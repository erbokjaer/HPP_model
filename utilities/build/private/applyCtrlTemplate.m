function applyCtrlTemplate(mdl, map, varargin)
    % layout of an asset controller from the template models/layout/
    % asset_ctrl.json (captured from the BESS_ctrl hand layout, see
    % captureLayout / applyLayout); the signal connections stay as built.
    % varargin: Place=..., Tags=... (applyLayout)
    f = fullfile(currentProject().RootFolder, "models", "layout", "asset_ctrl.json");
    if ~isfile(f), return, end
    r = applyLayout(mdl, f, "Map", map, varargin{:});
    if ~isempty(r.missing),  fprintf("  %s template: not here: %s\n", mdl, strjoin(r.missing, ", ")); end
    if ~isempty(r.unplaced), fprintf("  %s template: placed by its neighbours: %s\n", mdl, strjoin(r.unplaced, ", ")); end
end
