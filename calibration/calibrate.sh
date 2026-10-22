#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
KALIBR_CONFIG="${KALIBR_CONFIG:-$SCRIPT_DIR/kalibr_config.yaml}"

yaml_get() {
  local file="$1"
  local key="$2"
  local section="${key%%.*}"
  local field="${key#*.}"
  awk -v section="$section" -v field="$field" '
    BEGIN { in_section = 0 }
    $0 ~ "^" section ":[[:space:]]*$" { in_section = 1; next }
    in_section && $0 ~ "^[[:alpha:]][^:]*:[[:space:]]*$" { in_section = 0; next }
    in_section {
      line = $0
      sub(/[[:space:]]+#.*$/, "", line)
      if (line ~ "^[[:space:]]*" field ":[[:space:]]*") {
        sub("^[[:space:]]*" field ":[[:space:]]*", "", line)
        gsub(/^[[:space:]]+|[[:space:]]+$/, "", line)
        if (line ~ /^".*"$/) { line = substr(line, 2, length(line)-2) }
        print line
        exit
      }
    }
  ' "$file"
}

BUILD_DIR="$(yaml_get "$KALIBR_CONFIG" "build.build_dir")"
if [[ "$BUILD_DIR" != /* ]]; then
  BUILD_DIR="$ROOT_DIR/$BUILD_DIR"
fi

CAMERA_CONFIG="$(yaml_get "$KALIBR_CONFIG" "camera.config_path")"
if [[ "$CAMERA_CONFIG" != /* ]]; then
  CAMERA_CONFIG="$ROOT_DIR/$CAMERA_CONFIG"
fi

IMAGE="$(yaml_get "$KALIBR_CONFIG" "docker.image")"
KALIBR_SOURCE_DIR="$(yaml_get "$KALIBR_CONFIG" "docker.kalibr_source_dir")"
OUTPUT_PREFIX="$(yaml_get "$KALIBR_CONFIG" "dataset.output_dir_prefix")"

usage() {
  cat <<'EOF'
Usage:
  bash calibration/calibrate.sh all [--duration SEC] [--image-hz HZ] [--output DIR]
  bash calibration/calibrate.sh record [--duration SEC] [--image-hz HZ] [--output DIR]
  bash calibration/calibrate.sh bag [DATASET_DIR]
  bash calibration/calibrate.sh calibrate [BAG_FILE]
  bash calibration/calibrate.sh extract [CAMCHAIN_YAML]   # 换算外参并打印(不写文件)
  bash calibration/calibrate.sh apply   [CAMCHAIN_YAML] [DEMO_YAML]  # 换算并直接写进 demo.yaml
  bash calibration/calibrate.sh docker-build

Environment:
  KALIBR_CONFIG=calibration/kalibr_config.yaml   # only entry point
EOF
}

apply_record_defaults() {
  local args=("$@")
  local has_duration=false
  local has_image_hz=false
  local has_output=false
  local has_preview=false
  local i

  for ((i = 0; i < ${#args[@]}; ++i)); do
    case "${args[i]}" in
      --duration|--image-hz|--output|--target-config)
        ((i++))
        ;;
    esac
    case "${args[i]}" in
      --duration)
        has_duration=true
        ;;
      --image-hz)
        has_image_hz=true
        ;;
      --output)
        has_output=true
        ;;
      --preview)
        has_preview=true
        ;;
    esac
  done

  if [[ -f "$KALIBR_CONFIG" ]]; then
    if [[ "$has_duration" == false ]]; then
      local duration
      duration="$(yaml_get "$KALIBR_CONFIG" "dataset.record_duration_s")"
      if [[ -n "$duration" ]]; then
        args+=(--duration "$duration")
      fi
    fi

    if [[ "$has_image_hz" == false ]]; then
      local image_hz
      image_hz="$(yaml_get "$KALIBR_CONFIG" "dataset.camera_image_hz")"
      if [[ -n "$image_hz" ]]; then
        args+=(--image-hz "$image_hz")
      fi
    fi

    if [[ "$has_preview" == false ]]; then
      local preview
      preview="$(yaml_get "$KALIBR_CONFIG" "dataset.preview")"
      if [[ "$preview" == "true" ]]; then
        args+=(--preview)
      fi
    fi
  fi

  if [[ "$has_output" == false ]]; then
    local output_base
    if [[ "$OUTPUT_PREFIX" == /* ]]; then
      output_base="$OUTPUT_PREFIX"
    else
      output_base="$ROOT_DIR/$OUTPUT_PREFIX"
    fi
    args+=(--output "${output_base}$(date +%Y%m%d_%H%M%S)")
  fi

  printf '%s\n' "${args[@]}"
}

docker_cmd() {
  if docker info >/dev/null 2>&1; then
    docker "$@"
  else
    sudo docker "$@"
  fi
}

build_recorder() {
  cmake -S "$ROOT_DIR" -B "$BUILD_DIR"
  cmake --build "$BUILD_DIR" --target calibrate -j"$(nproc)"
}

docker_build() {
  if [[ ! -d "$KALIBR_SOURCE_DIR/.git" ]]; then
    rm -rf "$KALIBR_SOURCE_DIR"
    git clone --depth 1 https://github.com/ethz-asl/kalibr.git "$KALIBR_SOURCE_DIR"
  fi
  cp "$SCRIPT_DIR/docker/kalibr_sb.patch" "$KALIBR_SOURCE_DIR/"
  docker_cmd build \
    -t "$IMAGE" \
    -f "$SCRIPT_DIR/docker/Dockerfile.kalibr-cn" \
    "$KALIBR_SOURCE_DIR"
}

latest_dataset() {
  local base
  base="$ROOT_DIR/$(dirname "$OUTPUT_PREFIX")"
  find "$base" -maxdepth 1 -type d -name 'dataset_*' | sort | tail -n 1
}

# 从 dataset 目录或 bag 文件名推出对应的 camchain yaml（Kalibr 输出在数据集同目录）。
camchain_for() {
  local dataset="${1:-}"
  if [[ -z "$dataset" ]]; then
    dataset="$(latest_dataset)"
  fi
  local base
  base="$ROOT_DIR/$(dirname "$OUTPUT_PREFIX")"
  local name
  name="$(basename "$(realpath "$dataset" 2>/dev/null || echo "$dataset")")"
  # 去掉 .bag 后缀
  name="${name%.bag}"
  local camchain="$base/$name-camchain-imucam.yaml"
  echo "$camchain"
}

cleanup_old_datasets() {
  local data_dir
  data_dir="$ROOT_DIR/$(dirname "$OUTPUT_PREFIX")"
  mkdir -p "$data_dir"
  echo "Cleaning up old datasets in $data_dir ..."
  find "$data_dir" -maxdepth 1 -type d -name 'dataset_*' -exec rm -rf {} +
  find "$data_dir" -maxdepth 1 -type f -name 'dataset_*' -delete
}

record() {
  cleanup_old_datasets
  build_recorder
  mapfile -t record_args < <(apply_record_defaults "$@")
  "$BUILD_DIR/calibrate" --config "$CAMERA_CONFIG" --target-config "$KALIBR_CONFIG" "${record_args[@]}"
}

bag() {
  local dataset="${1:-}"
  if [[ -z "$dataset" ]]; then
    dataset="$(latest_dataset)"
  fi
  if [[ -z "$dataset" || ! -d "$dataset" ]]; then
    echo "No dataset found under $OUTPUT_PREFIX" >&2
    exit 1
  fi

  local dataset_abs
  dataset_abs="$(realpath "$dataset")"
  local dataset_name
  dataset_name="$(basename "$dataset_abs")"

  local data_dir
  data_dir="$ROOT_DIR/$(dirname "$OUTPUT_PREFIX")"

  docker_cmd run --rm \
    -v "$data_dir:/data" \
    "$IMAGE" \
    kalibr_bagcreater --folder "/data/$dataset_name" --output-bag "/data/$dataset_name.bag"

  echo "$dataset_abs.bag"
}

calibrate() {
  local bag_file="${1:-}"
  if [[ -z "$bag_file" ]]; then
    local dataset
    dataset="$(latest_dataset)"
    bag_file="$dataset.bag"
  fi
  if [[ -z "$bag_file" || ! -f "$bag_file" ]]; then
    echo "Bag file not found: $bag_file" >&2
    exit 1
  fi

  local bag_abs
  bag_abs="$(realpath "$bag_file")"
  local bag_name
  bag_name="$(basename "$bag_abs")"

  local data_dir
  data_dir="$ROOT_DIR/$(dirname "$OUTPUT_PREFIX")"

  docker_cmd run --rm \
    -v "$data_dir:/data" \
    "$IMAGE" \
    bash -lc "cd /data; kalibr_calibrate_imu_camera --bag /data/$bag_name --cam /data/camchain_prior.yaml --imu /data/imu.yaml --target /data/target.yaml"
}

# 把 Kalibr 的 T_cam_imu 换算成 demo.yaml 的外参（R_camera2gimbal / t_camera2gimbal /
# imu_time_offset_s），并做三轴方向自检。见 calibration/extract_extrinsics.py。
extract() {
  local camchain="${1:-}"
  if [[ -z "$camchain" ]]; then
    camchain="$(camchain_for "$(latest_dataset)")"
  fi
  if [[ -z "$camchain" || ! -f "$camchain" ]]; then
    echo "camchain yaml not found: ${camchain:-<none>}" >&2
    echo "用法: bash calibration/calibrate.sh extract <camchain-imucam.yaml>" >&2
    exit 1
  fi
  python3 "$SCRIPT_DIR/extract_extrinsics.py" "$camchain"
}

# 同 extract，但换算后直接写进 demo.yaml（先自检+备份）。见 calibration/apply_extrinsics.py。
apply() {
  local camchain="${1:-}"
  local demo="${2:-}"
  if [[ -z "$camchain" ]]; then
    camchain="$(camchain_for "$(latest_dataset)")"
  fi
  if [[ -z "$camchain" || ! -f "$camchain" ]]; then
    echo "camchain yaml not found: ${camchain:-<none>}" >&2
    echo "用法: bash calibration/calibrate.sh apply <camchain-imucam.yaml> [demo.yaml]" >&2
    exit 1
  fi
  if [[ -n "$demo" ]]; then
    python3 "$SCRIPT_DIR/apply_extrinsics.py" "$camchain" "$demo"
  else
    python3 "$SCRIPT_DIR/apply_extrinsics.py" "$camchain"
  fi
}

cmd="${1:-all}"
shift || true

case "$cmd" in
  record)
    record "$@"
    ;;
  bag)
    bag "$@"
    ;;
  calibrate)
    calibrate "$@"
    ;;
  all)
    cleanup_old_datasets
    record "$@"
    dataset="$(latest_dataset)"
    bag "$dataset"
    calibrate "$dataset.bag"
    apply "$(camchain_for "$dataset")"
    ;;
  extract)
    extract "$@"
    ;;
  apply)
    apply "$@"
    ;;
  docker-build)
    docker_build
    ;;
  help|-h|--help)
    usage
    ;;
  *)
    echo "Unknown command: $cmd" >&2
    usage
    exit 2
    ;;
esac
