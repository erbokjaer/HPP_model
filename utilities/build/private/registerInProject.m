function registerInProject(file)
    % add a newly generated model to the MATLAB project (no-op if already in it)
    pr = currentProject;
    if isempty(findFile(pr, file))
        addFile(pr, file);
    end
end
