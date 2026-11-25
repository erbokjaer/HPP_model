function closeIfLoaded(mdl)
    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
end
