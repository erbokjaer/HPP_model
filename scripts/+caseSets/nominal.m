function cases = nominal()
%NOMINAL  The nominal case: row "nominal" of caseSets.handpicked.
%   This is the default case set for runCase, runSweep and defineCases.

    cases = caseSets.handpicked();
    cases = cases("nominal", :);
end
