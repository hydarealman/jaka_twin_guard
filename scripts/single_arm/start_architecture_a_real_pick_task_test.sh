#!/usr/bin/env bash
set -eo pipefail

# Compatibility entry point for real pick/sort validation.  Manual validation
# is deliberately performed with MoveIt's native RViz Plan and Execute buttons;
# the production behavior tree remains available through the real-run script.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[fruit-arm] manual pick validation uses native RViz Plan and Execute"
echo "[fruit-arm] no web dashboard and no automatic pick task will be started"
exec bash "${SCRIPT_DIR}/start_architecture_a_real_fruit_plan_execute.sh" "$@"
