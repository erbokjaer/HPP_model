function folder = resultsFolder(setName, create)
%RESULTSFOLDER  results/<set> in the project; created with create = true.
    folder = fullfile(currentProject().RootFolder, "results", string(setName));
    if nargin > 1 && create && ~isfolder(folder)
        mkdir(folder);
    end
end
