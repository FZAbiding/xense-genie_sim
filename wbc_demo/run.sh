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
    umi-web)
        if [[ $# -ne 0 ]]; then
            echo "umi-web starts episode 000 with defaults; use umi-live for custom arguments." >&2
            exit 2
        fi
        if python -c 'import socket,sys; connection=socket.socket(); connection.settimeout(0.5); sys.exit(connection.connect_ex(("127.0.0.1",8765)))'; then
            echo "Port 8765 is already in use. Open http://127.0.0.1:8765 or stop its owner first." >&2
            exit 1
        fi
        session_tag="$(date +%Y%m%d-%H%M%S)-$$"
        export WBC_SCENE="logs/scene-umi-web-${session_tag}.yaml"
        python wbc_demo/umi_demo.py view-scene \
            --input wbc_demo/data/converted/episode-000.json \
            --output "wbc_demo/$WBC_SCENE"
        exec bash wbc_demo/run.sh umi-live \
            --input wbc_demo/data/converted/episode-000.json \
            --scene "wbc_demo/$WBC_SCENE" \
            --output "wbc_demo/logs/umi-web-${session_tag}"
        ;;
    sim)
        exec ros2 launch genie_sim_bringup app.launch.py \
            scene:="$repo_root/wbc_demo/${WBC_SCENE:-scene_stress.yaml}" \
            launcher_config:=launcher_ovrtx_isaac_physx headless:=true rviz:=false "$@"
        ;;
    moveit)
        exec ros2 launch genie_sim_moveit wbc.launch.py \
            use_ros2_control:=false use_rviz:="${WBC_USE_RVIZ:-true}" pad_joint_limits:=false \
            rviz_config:="$repo_root/wbc_demo/fixed_base.rviz" "$@"
        ;;
    demo)
        exec python wbc_demo/dual_arm_ik.py "$@"
        ;;
    stress)
        exec python wbc_demo/experiments.py "$@"
        ;;
    compare)
        exec python wbc_demo/experiments.py --compare "$@"
        ;;
    interactive)
        exec python wbc_demo/interactive_lab.py "$@"
        ;;
    umi)
        exec python wbc_demo/umi_demo.py "$@"
        ;;
    all|lab|umi-all|umi-live)
        if [[ ! "${OMNI_KIT_ACCEPT_EULA:-}" =~ ^([Yy][Ee][Ss]|[Yy]|1)$ ]]; then
            echo "Isaac Sim requires your acceptance of the NVIDIA Omniverse EULA." >&2
            echo "If you agree, run: export OMNI_KIT_ACCEPT_EULA=YES" >&2
            exit 1
        fi
        mkdir -p wbc_demo/logs
        backend_logs=wbc_demo/logs
        if [[ "$mode" == "umi-all" || "$mode" == "umi-live" ]]; then
            backend_logs="wbc_demo/logs/umi-backend-$(date +%Y%m%d-%H%M%S)"
            mkdir "$backend_logs"
            export WBC_BACKEND_LOG_DIR="$backend_logs"
            echo "UMI backend evidence: $backend_logs"
        fi
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
        if [[ "$mode" == "umi-live" ]]; then export WBC_USE_RVIZ=false; fi
        setsid bash wbc_demo/run.sh sim > "$backend_logs/sim.log" 2>&1 &
        children+=("$!")
        setsid bash wbc_demo/run.sh moveit > "$backend_logs/moveit.log" 2>&1 &
        children+=("$!")
        if [[ "$mode" == "umi-live" ]]; then
            sleep 1
            for child in "${children[@]}"; do
                if ! kill -0 "$child" 2>/dev/null; then
                    echo "Simulation/MoveIt exited during startup. Check $backend_logs before opening the Web viewer." >&2
                    exit 1
                fi
            done
            python wbc_demo/umi_demo.py live "$@" 2>&1 | tee "$backend_logs/live.log"
            exit 0
        elif [[ "$mode" == "umi-all" ]]; then
            python wbc_demo/umi_demo.py replay "$@" 2>&1 | tee "$backend_logs/controller.log"
            exit 0
        elif [[ "$mode" == "lab" ]]; then
            python wbc_demo/interactive_lab.py "$@" 2>&1 | tee wbc_demo/logs/interactive.log
            exit 0
        else
            python wbc_demo/experiments.py "$@" 2>&1 | tee wbc_demo/logs/demo.log
        fi
        echo "Demo complete. RViz and simulation stay open; press Ctrl+C to stop."
        wait
        ;;
    *)
        echo "Usage: bash wbc_demo/run.sh {all|lab|umi-all|umi-live|umi-web|umi|sim|moveit|demo|stress|compare|interactive} [arguments]" >&2
        exit 2
        ;;
esac
