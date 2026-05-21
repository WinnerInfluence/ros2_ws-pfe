#!/usr/bin/env bash
# Shared Gazebo / gnome-terminal environment for PFE launchers (train_menu, start_pfe, …).
#
# Reliable GUI:  pkill -9 gzserver gzclient; then train_menu / start_pfe launchers
# Avoid leaving gzserver running without gzclient (thesis EVAL_KILL_GUI=1) — blocks the next launch.

# Default Gazebo master (gzclient freezes on "Preparing your world" if port is stale).
pfe_gazebo_master_uri() {
    printf '%s' "${GAZEBO_MASTER_URI:-http://localhost:11345}"
}

pfe_gazebo_free_master_port() {
    export GAZEBO_MASTER_URI="$(pfe_gazebo_master_uri)"
    if command -v fuser >/dev/null 2>&1; then
        fuser -k 11345/tcp 2>/dev/null || true
    fi
    sleep 1
}

pfe_gazebo_kill_all() {
    local _i
    for ((_i = 0; _i < 3; _i++)); do
        pkill -f 'ros2 launch gazebo_ros' 2>/dev/null || true
        pkill -f 'gazebo --verbose' 2>/dev/null || true
        pkill -f 'gazebo .*\.world' 2>/dev/null || true
        pkill -x gzclient 2>/dev/null || true
        pkill -x gzserver 2>/dev/null || true
        pkill -f pfe_reset_simulation.py 2>/dev/null || true
        sleep 1
        pgrep -x gzserver &>/dev/null || break
    done
    pkill -9 -x gzclient 2>/dev/null || true
    pkill -9 -x gzserver 2>/dev/null || true
    pkill -9 -f 'ros2 launch gazebo_ros' 2>/dev/null || true
    pkill -9 -f 'gazebo .*\.world' 2>/dev/null || true
    pfe_gazebo_free_master_port
    rm -rf "${HOME}/.gazebo/server-"* 2>/dev/null || true
    sleep 1
}

pfe_kill_gazebo() {
    pfe_gazebo_kill_all
}

pfe_export_gazebo_runtime_env() {
    export IGN_IP="${IGN_IP:-127.0.0.1}"
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
    export PFE_GAZEBO_STABLE="${PFE_GAZEBO_STABLE:-1}"
    if [[ "${PFE_GAZEBO_STABLE}" == "1" && -n "${DISPLAY:-}" ]]; then
        export QT_X11_NO_MITSHM=1
    fi
}

# gzclient hangs on "Preparing your world" / "Waiting for model database update…"
# when /usr/share/gazebo/setup.sh points at models.gazebosim.org (slow or offline).
pfe_gazebo_disable_online_model_db() {
    if [[ "${PFE_GAZEBO_MODEL_DB:-0}" == "1" ]]; then
        return 0
    fi
    export GAZEBO_MODEL_DATABASE_URI=""
    local ros_d="${ROS_DISTRO:-humble}"
    local tb3="/opt/ros/${ros_d}/share/turtlebot3_gazebo/models"
    if [[ -d "$tb3" ]]; then
        case ":${GAZEBO_MODEL_PATH:-}:" in
            *":${tb3}:"*) ;;
            *) export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:+$GAZEBO_MODEL_PATH:}${tb3}" ;;
        esac
    fi
}

# ROS + Gazebo paths (call after ROS setup.bash). Do NOT replace GAZEBO_RESOURCE_PATH with
# only custom textures — that drops Gazebo media/ and breaks gzclient (shadow_caster, Camera).
pfe_gazebo_source_all() {
    set +u
    # shellcheck disable=SC1091
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
    if [[ -f "${HOME}/ros2_ws/install/setup.bash" ]]; then
        source "${HOME}/ros2_ws/install/setup.bash"
    fi
    if [[ -f /usr/share/gazebo/setup.sh ]]; then
        # shellcheck disable=SC1091
        source /usr/share/gazebo/setup.sh
    fi
    pfe_gazebo_disable_online_model_db
    export GAZEBO_PLUGIN_PATH="/opt/ros/${ROS_DISTRO:-humble}/lib:${GAZEBO_PLUGIN_PATH:-}"
    set -u
}

pfe_gazebo_prepend_resource_path() {
    local extra="${1:-}"
    [[ -n "$extra" ]] || return 0
    pfe_gazebo_source_all
    case ":${GAZEBO_RESOURCE_PATH:-}:" in
        *":${extra}:"*) ;;
        *) export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:+$GAZEBO_RESOURCE_PATH:}${extra}" ;;
    esac
}

pfe_gazebo_reset_client_state() {
    rm -rf "${HOME}/.gazebo/client-"* 2>/dev/null || true
}

pfe_term_env_prefix() {
    printf '%s' \
        'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"; export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"; export IGN_IP="${IGN_IP:-127.0.0.1}"; '
}

# Extra ros2 launch args for gazebo.launch.py (leading space if non-empty).
# Default stable: gui:=false — ros2's gzclient uses libgazebo_ros_eol_gui.so which
# crashes (Camera assertion) on many Humble installs; start plain gzclient separately.
pfe_gazebo_gui_suffix() {
    if [[ "${PFE_GAZEBO_GUI:-1}" == "0" ]]; then
        printf '%s' ' gui:=false'
    elif [[ "${PFE_GAZEBO_STABLE:-1}" == "1" ]]; then
        printf '%s' ' gui:=false'
    else
        printf '%s' ''
    fi
}

# Shell snippet: ros2 launch gzserver, then plain gzclient (for gnome-terminal).
pfe_gazebo_stable_one_terminal_cmd() {
    local world_quoted="$1"
    # world_quoted must be printf %q — use double quotes in echo (single quotes break bash -c "…").
    printf '%s' \
        "pfe_gazebo_source_all; pfe_gazebo_reset_client_state; " \
        "echo \"[GAZEBO] world=${world_quoted}\"; " \
        "gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so " \
        "-s libgazebo_ros_force_system.so ${world_quoted}; exec bash"
}

# Start gzserver only (ros2 launch gui:=false). Avoids integrated `gazebo` GUI freezes.
pfe_gazebo_ros_daemon_refresh() {
    ros2 daemon stop 2>/dev/null || true
    sleep 1
    ros2 daemon start 2>/dev/null || true
    sleep 1
}

pfe_gazebo_start_server_bg() {
    local world="$1"
    world="$(readlink -f "$world" 2>/dev/null || realpath "$world")"
    pfe_gazebo_source_all
    pfe_gazebo_reset_client_state
    pfe_gazebo_ros_daemon_refresh
    echo "[gazebo] gzserver → $world"
    echo "[gazebo] textured worlds may take 60–90s to load on a laptop…"
    ros2 launch gazebo_ros gazebo.launch.py "world:=$world" gui:=false verbose:=false &
    echo $!
}

pfe_gazebo_wait_gzserver() {
    local max_sec="${1:-180}"
    local grace_sec="${PFE_GZSERVER_GRACE_SEC:-25}"
    local t=0
    local seen=0
    local refresh_every="${PFE_ROS_DAEMON_REFRESH_EVERY:-20}"
    local last_refresh=0
    export GAZEBO_MASTER_URI="$(pfe_gazebo_master_uri)"
    echo "[gazebo] waiting for gzserver (up to ${max_sec}s, loading textures)…"
    while (( t < max_sec )); do
        if pgrep -x gzserver >/dev/null; then
            seen=1
            if (( t - last_refresh >= refresh_every )); then
                pfe_gazebo_ros_daemon_refresh
                last_refresh=$t
            fi
            if ros2 service list 2>/dev/null | grep -q '/spawn_entity'; then
                ros2 service call /unpause_physics std_srvs/srv/Empty "{}" 2>/dev/null || true
                echo "[gazebo] gzserver ready (${t}s)."
                return 0
            fi
        elif (( seen )); then
            echo "[gazebo] ERROR: gzserver exited during load. Run: pkill -9 gzserver gzclient" >&2
            return 1
        elif (( t >= grace_sec )); then
            echo "[gazebo] ERROR: gzserver never started (port 11345 busy?). Run: pkill -9 gzserver gzclient" >&2
            return 1
        fi
        sleep 3
        t=$((t + 3))
    done
    echo "[gazebo] ERROR: gzserver not ready after ${max_sec}s (try: kill_gazebo.sh, then retry)" >&2
    return 1
}

# One plain gzclient (never stack a second on top of integrated `gazebo`).
pfe_gazebo_record_env() {
    pfe_export_gazebo_runtime_env
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
    export LIBGL_DRI3_DISABLE="${LIBGL_DRI3_DISABLE:-1}"
    pfe_gazebo_disable_online_model_db
}

pfe_gazebo_server_warmup() {
    local sec="${1:-12}"
    echo "[gazebo] server warmup ${sec}s (textures load on gzserver first)…"
    sleep "$sec"
}

pfe_gazebo_start_gzclient_bg() {
    local delay="${PFE_GZCLIENT_DELAY_SEC:-5}"
    pfe_gazebo_stable_gui_env
    pfe_gazebo_source_all
    export GAZEBO_MASTER_URI="$(pfe_gazebo_master_uri)"
    pkill -x gzclient 2>/dev/null || true
    sleep 1
    echo "[gazebo] starting gzclient in ${delay}s (after server finished loading)…"
    sleep "$delay"
    gzclient --verbose &
    sleep 4
}

pfe_gazebo_start_gzclient_fg() {
    local delay="${PFE_GZCLIENT_DELAY_SEC:-5}"
    pfe_gazebo_stable_gui_env
    pfe_gazebo_source_all
    export GAZEBO_MASTER_URI="$(pfe_gazebo_master_uri)"
    pkill -x gzclient 2>/dev/null || true
    sleep 1
    echo "[gazebo] opening viewer in ${delay}s (offline model DB — no network wait)…"
    sleep "$delay"
    exec gzclient --verbose
}

# Open world: split gzserver + gzclient (stable on Humble; no eol_gui plugin).
pfe_gazebo_open_stable() {
    local world="$1"
    local wait_sec="${2:-20}"
    world="$(readlink -f "$world" 2>/dev/null || realpath "$world")"
    pfe_gazebo_start_server_bg "$world" >/dev/null
    if ! pfe_gazebo_wait_gzserver "$wait_sec"; then
        echo "[gazebo] ERROR: gzserver did not become ready" >&2
        return 1
    fi
    pfe_gazebo_start_gzclient_fg
}

# Legacy split launch (gui:=false + gzclient). Prefer open_gazebo.sh / train_menu _launch_sim.
# Set PFE_GAZEBO_STABLE=0 to skip QT_X11_NO_MITSHM if the GUI fails to open.
pfe_gazebo_stable_gui_env() {
    pfe_export_gazebo_runtime_env
    if [[ "${PFE_GAZEBO_STABLE}" == "1" && -n "${DISPLAY:-}" ]]; then
        export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
    fi
}

pfe_gazebo_launch_stable_shell() {
    local world_quoted="$1"
    printf '%s' \
        "pfe_gazebo_stable_gui_env; source /opt/ros/\${ROS_DISTRO:-humble}/setup.bash; " \
        "[[ -f \"\$HOME/ros2_ws/install/setup.bash\" ]] && source \"\$HOME/ros2_ws/install/setup.bash\"; " \
        "echo \"[GAZEBO] stable launch world=${world_quoted}\"; " \
        "ros2 launch gazebo_ros gazebo.launch.py world:=${world_quoted} gui:=false; " \
        "exec bash"
}

pfe_gazebo_launch_gzclient_shell() {
    printf '%s' \
        "pfe_gazebo_stable_gui_env; sleep 6; " \
        "echo '[GAZEBO] starting gzclient (record this window)…'; " \
        "gzclient --verbose; exec bash"
}

# Thread caps for NumPy/PyTorch CPU backends when launching training in a child shell.
# Override with PFE_CPU_MATH_THREADS (integer). Default: all logical CPUs (nproc).
pfe_training_cpu_math_env() {
    local n="${PFE_CPU_MATH_THREADS:-}"
    if [[ -z "$n" ]]; then
        n="$(nproc 2>/dev/null || echo 4)"
        n="${n//[^0-9]/}"
        [[ -z "$n" ]] && n=4
        (( n < 1 )) && n=1
    fi
    printf 'export OMP_NUM_THREADS=%s MKL_NUM_THREADS=%s OPENBLAS_NUM_THREADS=%s NUMEXPR_NUM_THREADS=%s VECLIB_MAXIMUM_THREADS=%s; ' \
        "$n" "$n" "$n" "$n" "$n"
}
