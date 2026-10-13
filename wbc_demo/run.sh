#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
if [[ "${CONDA_DEFAULT_ENV:-}" != "xense-wbc" ]]; then
    echo "Please run: mamba activate xense-wbc" >&2
    exit 1
fi
source devel/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
mode="${1:-all}"
if [[ $# -gt 0 ]]; then shift; fi

case "$mode" in
    sim)
        exec ros2 launch genie_sim_bringup app.launch.py \
            scene:="$repo_root/wbc_demo/scene_fixed.yaml" \
            launcher_config:=launcher_ovrtx_isaac_physx headless:=true rviz:=false "$@"
        ;;
    moveit)
        exec ros2 launch genie_sim_moveit wbc.launch.py \
            use_ros2_control:=false use_rviz:=true rviz_config:="$repo_root/wbc_demo/fixed_base.rviz" "$@"
        ;;
    demo)
        exec python wbc_demo/dual_arm_ik.py "$@"
        ;;
    all)
        if [[ ! "${OMNI_KIT_ACCEPT_EULA:-}" =~ ^([Yy][Ee][Ss]|[Yy]|1)$ ]]; then
            echo "Isaac Sim requires your acceptance of the NVIDIA Omniverse EULA." >&2
            echo "If you agree, run: export OMNI_KIT_ACCEPT_EULA=YES" >&2
            exit 1
        fi
        mkdir -p wbc_demo/logs
        children=()
        cleanup() {
            for child in "${children[@]}"; do
                kill -INT -- "-$child" 2>/dev/null || true
            done
            for attempt in {1..15}; do
                remaining=0
                for child in "${children[@]}"; do
                    if kill -0 -- "-$child" 2>/dev/null; then
                        remaining=1
                        if [[ "$attempt" -eq 10 ]]; then
                            kill -TERM -- "-$child" 2>/dev/null || true
                        fi
                    fi
                done
                if [[ "$remaining" -eq 0 ]]; then break; fi
                sleep 1
            done
            for child in "${children[@]}"; do
                kill -KILL -- "-$child" 2>/dev/null || true
            done
            wait || true
        }
        trap cleanup EXIT
        trap 'exit 130' INT TERM
        setsid bash wbc_demo/run.sh sim > wbc_demo/logs/sim.log 2>&1 &
        children+=("$!")
        setsid bash wbc_demo/run.sh moveit > wbc_demo/logs/moveit.log 2>&1 &
        children+=("$!")
        python wbc_demo/dual_arm_ik.py "$@" 2>&1 | tee wbc_demo/logs/demo.log
        echo "Demo complete. RViz and simulation stay open; press Ctrl+C to stop."
        wait
        ;;
    *)
        echo "Usage: bash wbc_demo/run.sh {all|sim|moveit|demo} [arguments]" >&2
        exit 2
        ;;
esac
