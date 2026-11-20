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
# Runs one command inside the Microsoft Visual Studio 2019 Build Tools environment (cl, link, cmake) in a given folder.
#
# usage: tools/msvc.sh DIR COMMAND...
#
# The compiler only works after vcvars64.bat has set up its search paths, and that batch file belongs to cmd.exe, not to Git
# Bash. The script therefore writes a small .cmd file (call vcvars64.bat, change to the folder, run the command) and starts it
# with cmd. Every call writes its own temporary file with a unique name, because earlier versions shared one file and two
# parallel runs overwrote each other's commands.
# DIR as a Windows path (the first argument), then drop it so that "$*" below is the command alone.
d=$(cygpath -w "$1"); shift
# The Visual Studio batch file that sets up the 64-bit compiler environment (VCVARS64 in settings.ini; empty: cl is on the PATH).
root="$(cd "$(dirname "$0")/.." && pwd)"; . "$root/tools/env.sh"; VC="$VCVARS64"
# A unique temporary file name (mktemp makes the name; .cmd is added so that cmd.exe will run it).
f=$(mktemp "${TEMP:-/tmp}/p68_msvc_XXXXXX").cmd
# Up to four lines of batch code: silence echo, set up the environment (output hidden), change drive and folder, run the command.
{ printf '@echo off\r\n'; [ -z "$VC" ] || printf 'call "%s" >nul\r\n' "$VC"; printf 'cd /d "%s"\r\n%s\r\n' "$d" "$*"; } > "$f"
# Run the batch file and keep its exit code.
cmd //c "$(cygpath -w "$f")"; rc=$?
# Delete both temporary files (mktemp also created one without the .cmd ending) and pass the exit code on.
rm -f "$f" "${f%.cmd}"; exit $rc
