#!/usr/bin/env bash
# Compiles all sources into ./out (run from anywhere).
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p out
javac -encoding UTF-8 -d out src/scr/*.java
echo "Build OK -> out/"
