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
# Tool locations for the scripts in tools/ and hw/. Source this file (". tools/env.sh"); it sets the variables below.
#
# The values come from settings.ini in the repository root when that file exists (copy settings.example.ini to settings.ini
# and edit it; settings.ini is ignored by git). It is a shell file of NAME=value lines. Anything not set there is taken from
# the PATH. Environment variables of the same names also work.
#   OSS_CAD_SUITE  folder of the OSS CAD Suite (Verilator, Icarus Verilog). Empty: verilator and iverilog come from the PATH.
#   PERL           Perl interpreter that runs Verilator's script (default perl).
#   GOWIN_SH       Gowin's command line tool gw_sh (default gw_sh on the PATH).
#   VCVARS64       Visual Studio's vcvars64.bat. Empty: cl and link are already on the PATH (a Developer Command Prompt).
#   PY             Python interpreter (default python).
# Variables set here: VERILATOR (the command), VINC (Verilator's include folder), GOWIN_SH, VCVARS64, PY.
r="${root:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
[ -f "$r/settings.ini" ] && . "$r/settings.ini"
PERL="${PERL:-perl}"; PY="${PY:-python}"; GOWIN_SH="${GOWIN_SH:-gw_sh}"; VCVARS64="${VCVARS64:-}"
if [ -n "${OSS_CAD_SUITE:-}" ]; then
  VERILATOR="$PERL $OSS_CAD_SUITE/bin/verilator"; VINC="$OSS_CAD_SUITE/share/verilator/include"
else
  VERILATOR="verilator"; VINC="$(verilator --getenv VERILATOR_ROOT 2>/dev/null)/include"
fi
