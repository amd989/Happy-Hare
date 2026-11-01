# Shift register bitbang architecture and USB CDC timing

## The bitbang

SHR16 = two cascaded 74HC595. A full register write shifts 16 bits out MSB
first, then latches:

```
per bit (MSB→LSB):  DATA = bit; CLOCK 1; CLOCK 0     (3 commands)
then:              LATCH 1; LATCH 0                  (2 commands)
= 50 queue_digital_out commands for 16 bits
```

- Commands are spaced `dt = 8000 ticks / 16MHz = 500µs` apart → ~25ms per
  write. The spacing absorbs USB CDC jitter.
- All 50 commands are sent as a **`minclock=0` burst** (see
  `_RawDigitalOut.set_digital`) so they arrive at the MCU before their
  scheduled times. With `minclock=last_clock` the host would pace sends to
  the schedule and USB CDC latency (~1ms) would push commands past their
  times → **"Timer too close"** shutdown.
- `_last_write_end` is published *before* the commands are generated
  (race safety: a request arriving mid-generation must see the write as
  in-flight).

## Failure modes observed live

1. **Two back-to-back bitbangs** (e.g. idler and selector homing ms apart):
   the MCU timer dispatcher saw backwards clock values →
   **"Rescheduled timer in the past"** shutdown at every boot (pre-serialization).
2. **2-3 writes piled around a gear move** (12:17 AM on the reference
   machine: `SR write` + two `SR defer` lines in the log, then
   **"Move queue overflow"**): the ~25ms timer-event storms + step execution
   together backed up the move pool.

## The four-layer fix (current design)

All in `extras/shift_register.py` (+ `mmu_stepper.py:_pre_set_dir_pin`).

### 1. Coalescing (bounded write count)

`_request_write(print_time, bit_num, old_state)` keeps `_pending_writes`, a
queue of scheduled-but-not-generated writes; only the last is open (reads
`self.state` when generated → latest-state-wins).

- A change merges into the open write only if that write starts after every
  motor's steps (`start >= get_steps_end()`). Otherwise a move between them
  needs the earlier state: the open write is frozen with `old_state` and a new
  write is queued after it. (Before, a gear buzz +0.8/-0.8 queued back to back
  merged the second flip into the first move's write and ran it backwards.)
- `_set_bit_at_time` schedules nothing if the state didn't change.

### 2. One bitbang on the MCU at a time

Each write is generated `_WRITE_LEAD` (250ms) before its start, by its own
reactor timer, and never before the previous write has run: its start is at
least previous end + 250ms. So at most ~50 SR nodes are on the 32u4 at once.
Live failure this prevents (first T0 load, gear buzz): with 250ms lead and no
such rule, bitbang N+1 arrived while bitbang N's 50 nodes were still queued,
plus the gear's queue_step nodes → "Move queue overflow".

### 3. All-motor gating (no bitbang during ANY motor's steps)

Every `MmuStepper` registers with the SR on its MCU at `klippy:connect`
(`register_motion_source(self.get_steps_end)`; `_steps_end` is the end of its
last `_submit_move`, trimmed to "now" after a homing drip an endstop cut
short; extruder-mode steppers also count `toolhead.print_time`).

- `_set_bit_at_time` schedules a bitbang no earlier than
  `sr.get_steps_end() + 0.005` — after the last step of EVERY motor on the
  MCU, not just the one asking.
- `_pre_set_dir_pin` (mmu_stepper.py) enables a disabled motor, writes a
  changed DIR at `max(now_pt + 0.100, base + 0.005)`, then delays the move
  (DIR change or not) to `sr.get_writes_end() + 0.010`. `get_writes_end()`
  covers the pending write, the in-flight one and the deferred initial write.

Originally only the requesting motor's previous move was cleared: at the end
of a load the idler's release move (no DIR change, so no wait) stepped through
another motor's write and the 32u4 shut down with "Timer too close". The
stopgap `idler_release_to_away: 0` was reverted once this landed.

### 4. LED deferral (no cosmetic writes mid-move)

`led_mask` config option (MMU3: `0xFF00`, bits 8-15). A write touching only
masked bits while the toolhead is ahead of the MCU (`print_time >
est_print_time + 0.050`) is pushed to `est_print_time + 0.300`. LED frames
then cost at most one ~25ms bitbang per ~300ms during a move.

## USB CDC timing hazards

- **Send lead (`_WRITE_LEAD` = 250ms)**: a write's 50 commands go out as ~9
  messages (~530 bytes), more than the 32u4's 192-byte receive window, so the
  host sends them in windows and one lost/late ACK costs a 25ms+ RTO. Any pin
  whose first command reaches the MCU after its clock (its queue empty) trips
  `sched_add_timer` → **"Timer too close"**. Live failure (T1→T0 toolchange,
  50ms pending lead): the latch pin's first command sat in the last message,
  was retransmitted, and arrived 12ms late (shutdown clock 11.9ms after the
  latch clock; `bytes_retransmit` 361→809). Test:
  `test_sr_writes_are_sent_well_ahead_of_their_first_command`.

- **Clock-sync convergence**: after MCU connect, `estimated_print_time` can
  be off by 1.5-2s (Caterina/USB CDC latency). The initial SR write is
  deferred **30s** after `klippy:connect` (`_handle_connect` →
  `_delayed_sr_write`); bootup autohome likewise waits for a reliable clock
  (commit `04f93269`).
- **Timed commands vs serial latency**: never use `minclock=last_clock`
  pacing for bitbang commands; keep the burst + 500µs spacing.
- **RX window**: a 50-command burst ≈ 300 bytes vs the 192-byte RX buffer —
  overflow triggers CRC-retransmit storms (`bytes_retransmit` in Stats).
  Coalescing keeps bursts *few*, which matters more than their size.

## Tests

`test/test_mmu_selector.py` → `TestPrusaIdlerSelector`:

- `test_idler_moves_go_through_the_mmu_stepper_with_dir_pre_set` — SR bit
  state follows moves.
- `test_sr_writes_coalesce_into_a_single_pending_bitbang` — merge semantics,
  one timer, single generation on fire.
- `test_sr_led_writes_defer_while_steps_execute` — cosmetic deferral via a
  simulated busy toolhead.
- `test_sr_writes_wait_for_every_motor_on_the_mcu` — idler moving + selector
  DIR write → write lands after the idler's move; both later moves wait for it.
- `test_sr_writes_are_sent_well_ahead_of_their_first_command` — >=200ms lead.
- `test_sr_buzz_keeps_one_bitbang_on_the_mcu_and_each_move_its_direction` —
  buzz queued ahead: SR nodes in flight <= 50, each move its own direction.

Harness quirk: the fake MCU's print clock is offset negative and never
advances during moves, so moves leave writes and step horizons behind — call
`quiet_shift_register()` before asserting timing.
