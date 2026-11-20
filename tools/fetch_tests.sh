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
# Downloads the SingleStepTests 68000 data and nothing else; no test is run here.
#
# usage: tools/fetch_tests.sh [dest]   downloads about 200 MB of gzip files (one per instruction and size, 124 listed in the
# folder) into dest (default build/sst-data/68000). The download is pinned to one commit of github.com/SingleStepTests/680x0 so
# that every run uses the same data. Files that exist are kept, so an interrupted fetch can be repeated. The repository states
# no license, which is why the data is fetched on demand and never copied into this repository.
set -e
# dest = target folder (first argument or the default), created if it does not exist.
root="$(cd "$(dirname "$0")/.." && pwd)"; dest="${1:-$root/build/sst-data/68000}"; mkdir -p "$dest"
SHA=e0d5ece9670205cc84a0101081837deb446f86a3  # the commit of the data set, folder 68000/v1
# Ask GitHub's contents API for the file list of that commit and keep the names ending in .json.gz.
names=$(curl -sL "https://api.github.com/repos/SingleStepTests/680x0/contents/68000/v1?ref=$SHA" | grep '"name"' | sed 's/.*: "//;s/".*//' | grep '\.json\.gz$')
# Counter of files handled.
n=0
for f in $names; do
  # Skip a file that already exists and is not empty.
  if [ ! -s "$dest/$f" ]; then
    # Download to a name ending in .part and rename it only when curl finished without error, so a broken download is never mistaken for a complete file.
    curl -sfL -o "$dest/$f.part" "https://raw.githubusercontent.com/SingleStepTests/680x0/$SHA/68000/v1/$f" && mv "$dest/$f.part" "$dest/$f"
  fi
  # Count the file (downloaded or kept).
  n=$((n+1))
done
# Report how many files are in the folder.
echo "$n files in $dest"
