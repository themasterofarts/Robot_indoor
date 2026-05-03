#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-robot-indoor:jazzy}"
CONTAINER_NAME="${CONTAINER_NAME:-robot-indoor-dev}"
WORKSPACE_DIR="${WORKSPACE_DIR:-$ROOT_DIR}"
XAUTH_FILE="${XAUTH_FILE:-/tmp/.docker-${CONTAINER_NAME}.xauth}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker n'est pas installe ou n'est pas disponible dans le PATH." >&2
  exit 1
fi

if [[ -z "${DISPLAY:-}" ]]; then
  echo "La variable DISPLAY n'est pas definie. Lancez la session graphique avant d'utiliser ce script." >&2
  exit 1
fi

if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
  echo "Image ${IMAGE_NAME} absente, construction en cours..."
  docker build \
    -f "${ROOT_DIR}/docker/Dockerfile" \
    -t "${IMAGE_NAME}" \
    "${ROOT_DIR}"
fi

USED_XHOST=0
if command -v xauth >/dev/null 2>&1; then
  touch "${XAUTH_FILE}"
  xauth nlist "${DISPLAY}" 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "${XAUTH_FILE}" nmerge - >/dev/null 2>&1 || true
  chmod 600 "${XAUTH_FILE}"
else
  echo "xauth indisponible sur l'hote, fallback vers xhost." >&2
  xhost +local:docker >/dev/null 2>&1 || true
  USED_XHOST=1
fi

cleanup() {
  if [[ "${USED_XHOST}" -eq 1 ]]; then
    xhost -local:docker >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

DOCKER_ARGS=(
  --rm
  -it
  --name "${CONTAINER_NAME}"
  --network host
  --ipc host
  --env DISPLAY="${DISPLAY}"
  --env QT_X11_NO_MITSHM=1
  --env XAUTHORITY=/tmp/.docker.xauth
  --volume "${WORKSPACE_DIR}:/workspaces/robot_indoor"
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
  --volume "${XAUTH_FILE}:/tmp/.docker.xauth:rw"
  --volume /etc/localtime:/etc/localtime:ro
  --workdir /workspaces/robot_indoor
)

if [[ -d /dev/dri ]]; then
  DOCKER_ARGS+=(--device /dev/dri:/dev/dri)
fi

if command -v nvidia-smi >/dev/null 2>&1; then
  DOCKER_ARGS+=(--gpus all)
fi

if [[ $# -eq 0 ]]; then
  set -- bash
fi

docker run "${DOCKER_ARGS[@]}" "${IMAGE_NAME}" "$@"
