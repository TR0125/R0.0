#!/bin/bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/raybot/raybot_chassis_ws/install/setup.bash

BASE_URL="${RAYBOT_BASE_URL:-http://192.168.1.55:8283}"
ROBOT_ID="${RAYBOT_ROBOT_ID:-ubot-001}"
EVENT_PATH="${RAYBOT_AIRMOUSE_EVENT_PATH:-/dev/input/by-id/usb-XING_WEI_2.4G_USB_USB_Composite_Device-if02-event-kbd}"
WAIT_INTERVAL_SEC="${RAYBOT_WAIT_INTERVAL_SEC:-3}"
MAX_WAIT_SEC="${RAYBOT_MAX_WAIT_SEC:-300}"
MODE_REFRESH_INTERVAL_SEC="${RAYBOT_MODE_REFRESH_INTERVAL_SEC:-30}"
STATUS_REPORT_DURATION_SEC="${RAYBOT_STATUS_REPORT_DURATION_SEC:-300}"

protocol_post_ok() {
    local path="$1"
    local payload="$2"
    local response
    response="$(curl -sS --connect-timeout 5 -X POST "${BASE_URL}${path}" \
        -H 'Content-Type: application/json' \
        -d "${payload}" 2>&1)" || return 1
    echo "$response" | grep -qE '"code"[[:space:]]*:[[:space:]]*0'
}

set_automatic_mode() {
    protocol_post_ok '/robot/mode' \
        "{\"robot_id\":\"${ROBOT_ID}\",\"operation_mode\":\"AUTOMATIC\"}"
}

request_status_report() {
    protocol_post_ok '/command/status' \
        "{\"robot_id\":\"${ROBOT_ID}\",\"report_type\":\"path\",\"report_duration\":${STATUS_REPORT_DURATION_SEC}}"
}

refresh_mode_loop() {
    while true; do
        if set_automatic_mode; then
            echo "Refreshed operation_mode=AUTOMATIC"
        else
            echo "WARN: failed to refresh operation_mode=AUTOMATIC"
        fi
        sleep "$MODE_REFRESH_INTERVAL_SEC"
    done
}

wait_for_input_device() {
    local elapsed=0
    while [ "$elapsed" -lt "$MAX_WAIT_SEC" ]; do
        if [ -e "$EVENT_PATH" ]; then
            echo "Input device ready: ${EVENT_PATH}"
            return 0
        fi
        sleep "$WAIT_INTERVAL_SEC"
        elapsed=$((elapsed + WAIT_INTERVAL_SEC))
    done
    echo "WARN: input device not found after ${MAX_WAIT_SEC}s: ${EVENT_PATH}"
    return 1
}

wait_for_chassis() {
    local elapsed=0
    local code
    while [ "$elapsed" -lt "$MAX_WAIT_SEC" ]; do
        code="$(curl -s --connect-timeout 2 -o /dev/null -w '%{http_code}' "${BASE_URL}/" 2>/dev/null || true)"
        if [[ "$code" =~ ^(200|302|401|404)$ ]]; then
            echo "Chassis reachable: ${BASE_URL} (${elapsed}s)"
            return 0
        fi
        sleep "$WAIT_INTERVAL_SEC"
        elapsed=$((elapsed + WAIT_INTERVAL_SEC))
    done
    echo "WARN: chassis not reachable after ${MAX_WAIT_SEC}s: ${BASE_URL}"
    return 1
}

ensure_automatic_mode() {
    local elapsed=0
    while [ "$elapsed" -lt "$MAX_WAIT_SEC" ]; do
        if set_automatic_mode; then
            echo "Set operation_mode=AUTOMATIC (${elapsed}s elapsed)"
            return 0
        fi
        sleep "$WAIT_INTERVAL_SEC"
        elapsed=$((elapsed + WAIT_INTERVAL_SEC))
    done
    echo "WARN: failed to set AUTOMATIC after ${MAX_WAIT_SEC}s, starting airmouse anyway"
    return 1
}

ensure_status_report() {
    if request_status_report; then
        echo "Requested path status report for ${STATUS_REPORT_DURATION_SEC}s"
    else
        echo "WARN: failed to request path status report"
    fi
}

refresh_mode_loop &
REFRESHER_PID=$!
trap 'kill "$REFRESHER_PID" 2>/dev/null || true' EXIT

wait_for_input_device || true
wait_for_chassis || true
ensure_automatic_mode || true
ensure_status_report || true

exec ros2 launch chassis_move protocol_airmouse_bringup.launch.py \
    robot_base_url:="${BASE_URL}" \
    robot_id:="${ROBOT_ID}" \
    input_event_path:="${EVENT_PATH}"
