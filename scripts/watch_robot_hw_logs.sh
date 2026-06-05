#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: watch_robot_hw_logs.sh [--tag TAG] [--event EVENT] [--follow] [--file PATH]

Filter ROBOT_HW_LOG lines from a file or stdin.

Options:
  --tag TAG       Match tag=TAG, for example TF, SENSOR, BASE.
  --event EVENT   Match event=EVENT, for example tf_publish.
  --follow        Follow the file with tail -F. Requires --file.
  --file PATH     Read from a log file instead of stdin.
  -h, --help      Show this help.

Examples:
  ./scripts/watch_robot_hw_logs.sh --tag TF --follow --file ~/ws/logs/robot_hw/latest.log
  ./scripts/watch_robot_hw_logs.sh --tag SENSOR --event scan_publish < bringup.log
USAGE
}

TAG_FILTER=""
EVENT_FILTER=""
FOLLOW=false
LOG_FILE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --tag)
      [[ $# -ge 2 ]] || { echo "--tag requires a value" >&2; exit 2; }
      TAG_FILTER="$2"
      shift 2
      ;;
    --event)
      [[ $# -ge 2 ]] || { echo "--event requires a value" >&2; exit 2; }
      EVENT_FILTER="$2"
      shift 2
      ;;
    --follow)
      FOLLOW=true
      shift
      ;;
    --file)
      [[ $# -ge 2 ]] || { echo "--file requires a path" >&2; exit 2; }
      LOG_FILE="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "$FOLLOW" == true && -z "$LOG_FILE" ]]; then
  echo "--follow requires --file" >&2
  exit 2
fi

filter_logs() {
  awk -v tag_filter="$TAG_FILTER" -v event_filter="$EVENT_FILTER" '
    /ROBOT_HW_LOG/ {
      if (tag_filter != "" && $0 !~ ("(^| )tag=" tag_filter "( |$)")) next
      if (event_filter != "" && $0 !~ ("(^| )event=" event_filter "( |$)")) next
      print
      fflush()
    }
  '
}

if [[ -n "$LOG_FILE" ]]; then
  if [[ ! -f "$LOG_FILE" ]]; then
    echo "Log file not found: $LOG_FILE" >&2
    exit 1
  fi

  if [[ "$FOLLOW" == true ]]; then
    tail -F "$LOG_FILE" | filter_logs
  else
    filter_logs < "$LOG_FILE"
  fi
else
  filter_logs
fi
