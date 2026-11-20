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
# Adds up the result files of tools/cputest.sh run-fpu. That command cuts large data folders into 4 shards and runs each shard in
# its own process, so one folder has up to four summary blocks, and this script merges them into one block per folder.
#
# Input: for every shard one summary line "NAME groups .. tests .. run .. pass .. fail .. undef .. skip .." followed by its
# "  skip N reason" lines (tests the runner skipped, with the reason) and "  cause N keys" lines (failing tests by cause). Any
# other line (a FAIL line with the details of a failing test) is passed through unchanged.
# Output: one summary line per folder with the counts added over all shards, in the order the folders first appeared, each followed
# by its skip and cause lines with the counts added per distinct text.
# Summary line: field 1 is the folder name, field 2 is "groups"; the following fields alternate between a word and a number, so the
# loop reads the pairs (field 2 and 3 up to field 14 and 15) and adds the number to a total kept per folder and word.
# n = the current folder; the first time it is seen it is remembered in names and its position in nord.
# The skip and cause lines belong to the folder of the summary line above them.
$2 == "groups" { n = $1; if (!(n in names)) { names[n] = 1; nord[++nn] = n }; for (i = 2; i <= 14; i += 2) t[n, $i] += $(i + 1); next }
# k = "skip" or "cause", c = the count; $1 and $2 are removed so that the rest of the line, the reason text, is left in $0.
$1 == "skip" || $1 == "cause" {
  k = $1; c = $2; $1 = ""; $2 = ""; sub(/^ +/, "")
  # The key combines folder, kind and text, so equal reasons of one folder are added together.
  key = n SUBSEP k SUBSEP $0
  # The first time a key appears it gets the next position of this folder, which keeps the output order stable.
  if (!(key in x)) { cnt[n]++; ord[n, cnt[n]] = key }
  # Add the count and go on to the next line.
  x[key] += c; next
}
# Everything else (FAIL lines) is printed as it is.
{ print }
# At the end print the merged result.
END {
  # For every folder in order of first appearance: ...
  for (j = 1; j <= nn; j++) { n = nord[j]
    # ... the summary line with the added counts ...
    printf "%-10s groups %6d tests %7d run %7d pass %7d fail %6d undef %5d skip %6d\n", n, t[n, "groups"], t[n, "tests"], t[n, "run"], t[n, "pass"], t[n, "fail"], t[n, "undef"], t[n, "skip"]
    # ... and its skip and cause lines.
    for (i = 1; i <= cnt[n]; i++) { split(ord[n, i], q, SUBSEP); printf "  %s %6d %s\n", q[2], x[ord[n, i]], q[3] }
  }
}
