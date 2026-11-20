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
# Downloads the Musashi 68000-68040 emulator (by Karl Stenerud, MIT license, github.com/kstenerud/Musashi) into
# third_party/musashi. The lockstep bench uses it as the reference CPU. Musashi is not part of this repository; this script
# fetches the exact revision the bench was written against. It does nothing when the folder is already filled.
#
# usage: tools/get_musashi.sh
set -e
root="$(cd "$(dirname "$0")/.." && pwd)"; dest="$root/third_party/musashi"
# The Musashi revision the patches in tools/musashi_patch.py were checked against.
REV=313ebf1bd9f4d0d93341eb5ce21fd8a119e9dbdd
[ -f "$dest/m68k_in.c" ] && { echo "Musashi is already in $dest"; exit 0; }
mkdir -p "$dest"; cd "$dest"
git init -q . && git remote add origin https://github.com/kstenerud/Musashi.git
git fetch -q --depth 1 origin "$REV" && git checkout -q FETCH_HEAD
echo "Musashi $REV is in $dest"
