#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
DEMO_DIR="${PROJECT_ROOT}/assets/demo"
BUILD_DIR="${DEMO_DIR}/build"
TARGET_PATH="${BUILD_DIR}/video_standard_demo"
USER_ARGS=("$@")
SELECTED_VIDEO=""

prompt_for_demo_video() {
  local choice=""

  echo "请选择测试视频："
  echo "  1) assets/demo/1.mp4"
  echo "  2) assets/demo/2.mp4"
  echo "  3) assets/demo/3.mp4"
  echo "  4) assets/demo/4.mp4"

  while true; do
    read -r -p "请输入 1/2/3/4: " choice || {
      echo "未读取到输入，启动已取消。" >&2
      exit 1
    }

    case "${choice}" in
      1|2|3|4)
        SELECTED_VIDEO="assets/demo/${choice}.mp4"
        return 0
        ;;
      *)
        echo "输入无效，请输入 1、2、3 或 4。" >&2
        ;;
    esac
  done
}

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi

export LD_LIBRARY_PATH="${PROJECT_ROOT}/io/hikrobot/lib/amd64:${PROJECT_ROOT}/io/mindvision/lib/amd64:${PROJECT_ROOT}/io/damiao/lib/linux:${LD_LIBRARY_PATH:-}"

cmake -S "${DEMO_DIR}" -B "${BUILD_DIR}"
cmake --build "${BUILD_DIR}" --target video_standard_demo -- -j"$(nproc)"

cd "${PROJECT_ROOT}"
if [[ ${#USER_ARGS[@]} -eq 0 ]]; then
  prompt_for_demo_video
  exec "${TARGET_PATH}" "${SELECTED_VIDEO}" -c configs/demo.yaml
fi

exec "${TARGET_PATH}" "${USER_ARGS[@]}"
