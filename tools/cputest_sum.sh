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
# Adds up result files of tools/cputest.sh run (or of tools/cputest_par.sh).
#
# usage: tools/cputest_sum.sh OUTFILE...
# Each runner call ends with one line per folder ("NAME groups .. tests .. run .. pass .. fail .. undef .. skip .."). The awk
# program below adds the columns over all files, prints a line for every folder that has failures or undefined results, and
# prints the total at the end. Passing folders are not listed, which keeps the output short for a run over 178 folders.
# The pattern matches the summary lines; fields 5, 7, 9, 11, 13 and 15 are tests, run, pass, fail, undef and skip.
cat "$@" | awk '/^[A-Za-z0-9.]+ +groups/ { run+=$7; pass+=$9; fail+=$11; undef+=$13; skip+=$15; tests+=$5; if ($11>0||$13>0) printf "%-9s run %7d pass %7d fail %6d undef %5d skip %6d\n",$1,$7,$9,$11,$13,$15 }
  END { printf "TOTAL tests %d run %d pass %d fail %d undef %d skip %d\n", tests, run, pass, fail, undef, skip }'
