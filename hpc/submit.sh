#!/bin/bash
# Submit one LSF job per case of a case set (scripts/+caseSets).
#
#   hpc/submit.sh                      nominal set
#   hpc/submit.sh grid_Pref_Pbess      all cases of that set
#   hpc/submit.sh grid_Pref_Pbess 9    skip the MATLAB call that counts the cases
#
# Run from the project root. Results: results/<caseSet>/case_NNN.mat

set -euo pipefail

CASESET="${1:-nominal}"
N="${2:-}"

cd "$(dirname "$0")/.."
mkdir -p hpc/logs

if [ -z "$N" ]; then
    module load matlab/R2026a
    N=$(matlab -batch "openProject('HPP_model.prj'); fprintf('NCASES=%d\n', height(defineCases('${CASESET}')))" \
        | grep '^NCASES=' | cut -d= -f2)
fi

if ! [[ "$N" =~ ^[0-9]+$ ]] || [ "$N" -lt 1 ]; then
    echo "Could not get the number of cases for '${CASESET}'." >&2
    exit 1
fi

echo "Submitting ${N} job(s) for case set '${CASESET}'"
export CASESET
bsub -J "hpp_${CASESET}[1-${N}]" < hpc/job.lsf
