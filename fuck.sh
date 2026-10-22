#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"

PROJECT_ROOT="$(pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"
SCRIPT_PATH="$(readlink -f "$0")"
BIN_PATH="$(readlink -f ./build/sentry)"
RUN_ARGS=(configs/demo.yaml -d)
CAN_BRIDGE_CONFIG="/home/lab/Desktop/can_bridge/config/sp_vision_raw_aim.yaml"

if [[ -n "${AUTOSTART_LOG:-}" ]]; then
	mkdir -p "${HOME}/.cache"
	LOG_FILE="${HOME}/.cache/sp_vision_autostart.log"
	exec >>"$LOG_FILE" 2>&1
	echo "===== $(date '+%F %T') sp_vision autostart ====="
	sleep 5
fi

cleanup() {
	local exit_code=$?
	trap - INT TERM EXIT
	if [[ -n "${CHILD_PID:-}" ]] && kill -0 "$CHILD_PID" 2>/dev/null; then
		stop_process_group "sentry" "$CHILD_PID"
	fi
	exit "$exit_code"
}

trap cleanup INT TERM EXIT

CURRENT_PGID="$(ps -o pgid= -p "$$" | tr -d ' ')"
SELF_PID="$$"

stop_process_group() {
	local name="$1"
	local pid="$2"
	local pgid
	pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')" || pgid=""
	if [[ -z "${pgid}" ]]; then
		return
	fi

	if [[ "${pgid}" == "${CURRENT_PGID}" ]]; then
		echo "[cleanup] stopping ${name} pid ${pid}"
		kill -TERM "$pid" 2>/dev/null || true
		sleep 0.5
		kill -KILL "$pid" 2>/dev/null || true
		wait "$pid" 2>/dev/null || true
		return
	fi

	echo "[cleanup] stopping ${name} process group -${pgid}"
	kill -TERM -- "-${pgid}" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true

	for _ in {1..10}; do
		if ! kill -0 "-${pgid}" 2>/dev/null; then
			wait "$pid" 2>/dev/null || true
			return
		fi
		sleep 0.1
	done

	echo "[cleanup] force killing ${name} process group -${pgid}"
	kill -KILL -- "-${pgid}" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
	wait "$pid" 2>/dev/null || true
}

kill_stale_process_group() {
	local pid="$1"
	local signal="${2:-TERM}"
	local pgid
	pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')" || return 0
	if [[ -n "${pgid}" ]] && [[ "${pgid}" != "${CURRENT_PGID}" ]]; then
		kill "-${signal}" -- "-$pgid" 2>/dev/null || kill "-${signal}" "$pid" 2>/dev/null || true
	fi
}

find_stale_script_pids() {
	local pid
	local script_arg
	local -a cmd_args
	for pid in $(pgrep -x bash || true); do
		[[ "$pid" == "${SELF_PID}" ]] && continue
		[[ -r "/proc/$pid/cmdline" ]] || continue
		cmd_args=()
		{ mapfile -d '' -t cmd_args <"/proc/$pid/cmdline"; } 2>/dev/null || continue
		script_arg="${cmd_args[1]:-}"
		[[ -z "${script_arg}" ]] && continue
		if [[ "$(readlink -f "${script_arg}" 2>/dev/null || true)" == "${SCRIPT_PATH}" ]]; then
			echo "$pid"
		fi
	done
}

echo "[check] cleanup stale script instances..."
mapfile -t STALE_SCRIPT_PIDS < <(find_stale_script_pids)
for pid in "${STALE_SCRIPT_PIDS[@]}"; do
	if [[ "$pid" != "$$" ]]; then
		echo "[check] kill stale script pid=$pid"
		kill_stale_process_group "$pid" TERM
		kill -TERM "$pid" 2>/dev/null || true
	fi
done
sleep 0.5
for pid in "${STALE_SCRIPT_PIDS[@]}"; do
	if [[ "$pid" != "$$" ]]; then
		kill_stale_process_group "$pid" KILL
		kill -KILL "$pid" 2>/dev/null || true
	fi
done

if [[ -n "${BIN_PATH}" ]]; then
	echo "[check] cleanup stale sentry process: $BIN_PATH ${RUN_ARGS[*]}"
	mapfile -t STALE_SENTRY_PIDS < <(
		{
			pgrep -f "$BIN_PATH ${RUN_ARGS[*]}" || true
			pgrep -f "./build/sentry ${RUN_ARGS[*]}" || true
			pgrep -f "build/sentry ${RUN_ARGS[*]}" || true
		} | sort -u
	)
	for pid in "${STALE_SENTRY_PIDS[@]}"; do
		echo "[check] kill stale sentry pid=$pid"
		kill_stale_process_group "$pid" TERM
	done
	sleep 0.5
	for pid in "${STALE_SENTRY_PIDS[@]}"; do
		kill_stale_process_group "$pid" KILL
	done
fi

echo "[check] cleanup stale can_bridge process: ${CAN_BRIDGE_CONFIG}"
mapfile -t STALE_BRIDGE_PIDS < <(pgrep -f "${CAN_BRIDGE_CONFIG}" || true)
for pid in "${STALE_BRIDGE_PIDS[@]}"; do
	echo "[check] kill stale can_bridge pid=$pid"
	kill_stale_process_group "$pid" TERM
done
sleep 0.5
for pid in "${STALE_BRIDGE_PIDS[@]}"; do
	kill_stale_process_group "$pid" KILL
done

if [[ -f /opt/ros/humble/setup.bash ]]; then
	source /opt/ros/humble/setup.bash
fi

export LD_LIBRARY_PATH="${PROJECT_ROOT}/io/mindvision/lib/amd64:${PROJECT_ROOT}/io/damiao/lib/linux:${LD_LIBRARY_PATH:-}"
export DM_CAN_DEVICE_INDEX="${DM_CAN_DEVICE_INDEX:-1}"

if [[ -f "${BUILD_DIR}/CMakeCache.txt" ]]; then
	cached_source="$(awk -F= '/^CMAKE_HOME_DIRECTORY:INTERNAL=/{print $2; exit}' "${BUILD_DIR}/CMakeCache.txt")"
	if [[ -n "${cached_source}" ]] && [[ "$(readlink -f "${cached_source}")" != "${PROJECT_ROOT}" ]]; then
		echo "[warn] stale CMake cache detected: ${cached_source}"
		echo "[fix] reset build cache for current project: ${PROJECT_ROOT}"
		rm -rf "${BUILD_DIR}/CMakeCache.txt" "${BUILD_DIR}/CMakeFiles"
	fi
fi

cmake -S "${PROJECT_ROOT}" -B "${BUILD_DIR}"
cmake --build "${BUILD_DIR}" --target sentry -- -j"$(nproc)"

if [[ ! -s ./build/sentry ]]; then
	echo "[warn] ./build/sentry is empty, force relink..."
	rm -f ./build/sentry
	cmake --build "${BUILD_DIR}" --target sentry -- -j"$(nproc)"
fi

echo "[run] DM_CAN_DEVICE_INDEX=${DM_CAN_DEVICE_INDEX}"
echo "[run] ./build/sentry configs/demo.yaml -d"
setsid ./build/sentry "${RUN_ARGS[@]}" &
CHILD_PID=$!
wait "$CHILD_PID"
