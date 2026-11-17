#!/usr/bin/env bash
# Usage: scripts/run.sh [drive|train] [client options]
# Client options default to "host:localhost port:3001".
# Extra JVM options can be passed through JAVA_OPTS, e.g. JAVA_OPTS="-Dtorcs.data=data/other.csv"
set -euo pipefail
cd "$(dirname "$0")/.."

MODE="${1:-drive}"
if [ "$#" -gt 0 ]; then shift; fi
if [ "$#" -eq 0 ]; then set -- host:localhost port:3001; fi

[ -d out ] || scripts/build.sh
# shellcheck disable=SC2086
java -Dfile.encoding=UTF-8 -Dtorcs.mode="$MODE" ${JAVA_OPTS:-} -cp out scr.Client scr.SimpleDriver "$@"
