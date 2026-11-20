# Memory port

The memory port is the core's only connection to memory. Anyone who attaches the core to a bus, a memory controller or a
simulation model needs this page. The module that drives the port is `rtl/core/p68_mem.v`, the signals are declared in
`rtl/core/p68_core.v`.

The core makes one transaction at a time. It never raises `m_req` again before `m_done`.

| Signal | Dir | Meaning |
|---|---|---|
| `m_req` | out | One clock: a transaction starts. `m_wr`, `m_addr`, `m_len`, `m_unc`, `m_ins`, `m_be` and `m_wdata` stay valid until `m_done`. |
| `m_wr` | out | 1 write, 0 read |
| `m_addr[31:2]` | out | Line read: line aligned (D-cache 16 bytes, I-cache 32 bytes). Uncached read and write: the word. |
| `m_len[1:0]` | out | Read beats minus 1: 0 uncached word, 1 D-cache line, 3 I-cache line. Beats come in ascending address order, no wrap. |
| `m_unc` | out | Uncached (data in the `unc_map` range): do exactly this access with the bytes in `m_be`, nothing more (I/O). Writes carry it too. |
| `m_ins` | out | Instruction fetch (line read for the I-cache) |
| `m_be[3:0]` | out | Byte enables, bit 3 = address + 0. Line reads: 1111. Uncached reads: the bytes the load needs. |
| `m_wdata[31:0]` | out | Write data, big-endian (byte + 0 in 31:24) |
| `m_rdy` | in | A read beat is on `m_rdata` this clock. Any number of idle clocks before and between beats; the earliest is the clock after `m_req`. |
| `m_rdata[63:0]` | in | Beat: the aligned 8 bytes, byte + 0 in 63:56. An uncached word comes in its natural lane of the 8-byte block. |
| `m_done` | in | The transaction ends. Reads: in the clock of the last beat. Writes: memory is written, or every later read on the port is ordered behind it. |
| `m_snoop`, `m_snoop_addr[31:2]` | in | A write by someone else (chip-bus DMA) to that word: the core invalidates the I-cache and D-cache lines that hold it. At most one pulse every second clock. It must come no later than the `m_done` of a line read that returned data older than that write. |
| `unc_map[15:0]` | in | Bit k = 1: data at address bits 23-20 = k is uncached. It applies only when address bits 31-24 are 00 or FF (the 24-bit area and its sign-extended absolute-short copy). Above 16 MB the map does not apply. Instruction fetch is always cached; code in chip RAM stays right through the snoop. |
| `unc_hi[7:0]` | in | When bit 7 is 1, data at address bits 31-25 = `unc_hi[6:0]` is uncached (a 32 MB window above 16 MB, for example a graphics board). |
| `cac_hi[6:0]` | in | Bit 6 = 1: data above 16 MB is cached only inside one RAM window (64 MB blocks selected by bits 5-3 at address bits 31-29, size by bits 2-0); everywhere else up there it is uncached. 0: everything above 16 MB is cached. |

## Ordering

Writes leave the store buffer in program order. A read (line fill or uncached word) is issued only when the store buffer is
empty and no store enters it in that clock, so reads never overtake writes. An adapter needs no reordering logic and no
write buffer of its own.

## Coherency

- **Self-modifying code.** A store that commits in EX and hits a valid I-cache line, the I line buffer, or one of the last
  four lines that fed the fetch queue replays the instruction behind it. The replay flushes the queue and fetches again from
  that instruction. The I line is invalidated in the next clock, and the refetch waits for the store buffer to drain. A store
  therefore takes effect before any later instruction executes. Limit: with the I-cache off or frozen, code more than four
  fetched lines back that is still in the pipeline is not checked.
- **External snoop (`m_snoop`).** The core registers the address and compares it with the line buffers and fills (first
  clock), then compares the snoop tag copy and invalidates the line (second clock). D-side loads of that line during those two
  clocks replay. A load that commits in the clock after the first one or later sees the written data; one that commits in
  the first clock or earlier sees the old data. The lockstep bench (`sim/lockstep/lockstep.cpp`) applies exactly this rule
  to the reference CPU. A snoop does not flush the fetch queue.
