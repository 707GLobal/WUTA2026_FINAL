#!/usr/bin/env bash
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BATCH="$ROOT/wuta_logs/sessions/batch_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BATCH"
echo "$BATCH" > "$ROOT/wuta_logs/sessions/latest_batch.txt"
for scenario in straight_pid noisy trackdrive velocity_dropout pose_dropout empty_path nan_path emergency; do
    mission=acceleration
    [[ "$scenario" != trackdrive ]] || mission=trackdrive
    run_dir="$ROOT/wuta_logs/$mission/$(date +%Y%m%d_%H%M%S)_${scenario}"
    ln -s "$run_dir" "$BATCH/$scenario"
    args=(--duration 22 --mission acceleration --speed 2 --output "$run_dir")
    case "$scenario" in
        noisy) args+=(--noise .15 --start-y .6 --start-yaw .1) ;;
        trackdrive) args=(--duration 40 --mission trackdrive --speed 4 --start-y 0 --start-yaw 0 --output "$run_dir") ;;
        *_dropout|empty_path|nan_path|emergency) args+=(--fault "$scenario") ;;
    esac
    echo "开始测试：$scenario"
    "$ROOT/start_wuta_sim_validation.sh" "${args[@]}"
    echo "$scenario exit=$?" | tee -a "$BATCH/results.txt"
done
/usr/bin/python3 "$ROOT/tools/summarize_wuta_sim.py" "$BATCH"
