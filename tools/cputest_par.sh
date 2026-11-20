#!/usr/bin/env bash
# Copyright 2026 Franz Schoning
# Project: https://github.com/Schoning-Labs/pipe68k
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Runs every folder of the WinUAE cputester data in parallel, one runner process per folder.
#
# usage: tools/cputest_par.sh OUTDIR [JOBS]   JOBS processes at a time (default 12); the result of each folder goes to
# OUTDIR/NAME.txt. The environment of tools/cputest.sh applies (CPUTEST_STRIDE runs a slice of every folder). A full run takes
# about 40 minutes, and the largest folder alone about 30. Sum up afterwards with tools/cputest_sum.sh OUTDIR/*.txt.
# data = the generated 68020 data (CPUTEST_DATA overrides it), out = first argument, jobs = second argument or 12.
root="$(cd "$(dirname "$0")/.." && pwd)"; data="${CPUTEST_DATA:-$root/build/cputest-gen/data/2_BASIC}"; out="$1"; jobs="${2:-12}"
# Create the result folder.
mkdir -p "$out"
# Start the biggest folders first, so that the slowest job does not begin last and stretch the total time.
# The order: folders sorted by disk usage of their data, largest first.
ls "$data" | grep -v '\.dat$' | while read n; do echo "$(du -sk "$data/$n" | cut -f1) $n"; done | sort -rn | cut -d' ' -f2 > "$out/.names"
# For every name run "tools/cputest.sh run 0 NAME" with up to JOBS at once and write its output to a file of that name.
xargs -P "$jobs" -I{} sh -c "'$root/tools/cputest.sh' run 0 {} > '$out/{}.txt' 2>&1" < "$out/.names"
# Marker file: the whole run is finished.
echo done > "$out/.finished"
