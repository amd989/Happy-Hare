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

`_request_write(print_time, bit_num)`:

- If a write is **pending** (scheduled, not yet generated): merge — return
  the pending start, do nothing else. The pending write reads `self.state`
  when it generates → latest-state-wins.
- If a write is **in flight** (`print_time < _last_write_end`): schedule ONE
  follow-up at `_last_write_end + 0.001`.
- Otherwise: if `print_time` is far enough out, defer generation via a
  reactor timer (`_WRITE_LEAD` = 250ms lead, `_fire_pending_write`); else
  generate now (still at least 250ms ahead).

A DIR-flip storm + LED frames collapse to 1-2 bitbangs.

### 2. Real-time gating (no overlapping writes)

The in-flight branch above never overlaps bitbangs; generation happens no
earlier than `_WRITE_LEAD` (250ms) before the scheduled start.

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

Harness quirk: the fake MCU's print clock is offset negative and never
advances during moves, so moves leave writes and step horizons behind — call
`quiet_shift_register()` before asserting timing.
