# Leakwatch — Design Document

**Per-enclosure leak detection for BlueROV2 / Navigator.**

A design study. Ninety parts, 82 nets, five schematic sheets, four-layer board.
Netlist-validated and simulated. **Never fabricated.** Section 11 lists what is
still wrong with it.

---

## 0. Who did what

This project was a way to learn two things at once: how to actually drive ECAD
tools, and how analog design decisions get made. Being precise about the split
matters more than looking good, so:

**Mine.** Schematic capture in KiCad — five hierarchical sheets, ninety parts —
including selecting the specific orderable parts and packages and checking each
symbol's pinout against its datasheet. The four-layer PCB layout, which I did
twice. Reading the Blue Robotics forum thread that started this. Every "why is
this here" question that changed the design.

**Claude Opus (Anthropic).** Read the ArduSub source and identified the specific
lines in `AP_LeakDetector`. Proposed the whole architecture: the three-tone
excitation scheme, the transimpedance front end, the four switched gain ranges,
the calibration channels, the independent hardware backstop, and the board
floorplan. Recommended the component families. **Ran all of the LTspice
simulation work**, including the frequency-window analysis the architecture rests
on. Wrote the netlist validator and this documentation.

The useful part was the gap between those two. You cannot lay out a board without
knowing why every part on it is there, so getting from the architecture to a
finished board meant asking why, repeatedly. Occasionally the answer didn't
survive contact — the multiplexer topology in §5.4 is in its current form because
I asked whether it had ever been checked against a reference design, and it
hadn't.

I have not taken the analog electronics sequence yet. There is a companion
document, `LEARNING.md`, which is the version of this written to teach me the
concepts rather than to present them.

---

## 1. The problem

BlueROV2 leak detection today is a probe pair, a voltage, and a threshold.

ArduSub's `AP_LeakDetector` supports **three** sensor instances and keeps a
per-instance result in `_state[i].status`. But the public accessor is:

```cpp
bool get_status() const { return _status; }
```

`_status` is the OR of all three. **The per-channel information exists inside the
class and is thrown away at the boundary.** A pilot is told the vehicle is
leaking, not which of the two enclosures is leaking, and on a vehicle where the
4″ electronics tube and the 3″ battery tube are opened, sealed and vacuum-tested
separately, that is the difference between a recoverable surface and a guess.

This was raised on the Blue Robotics forum in 2018. Jacob Walser answered it
properly in the thread, pointing at the `LeakDetector` class and at
`ArduSub/failsafe.cpp`. Nobody implemented it.

**Underneath that there is a second limitation, and it is the reason this is a
board and not a pull request.** The analog backend does:

```cpp
state.status = source->voltage_average() > 2.0f;
```

It reads a float and collapses it to a boolean in the same statement. The number
is gone before anything can use it. So even with per-channel reporting fixed in
firmware, each channel can still only say *wet* or *dry*, at a threshold fixed at
compile time, with a 3-second cooldown (`LEAKDETECTOR_COOLDOWN_MS`) on top.
There is no way to see water arriving and no trend to act on.

To be precise about what a number buys, because it is less than it first sounds:
the *onset* of conduction is fairly abrupt — below some amount of water there is
no continuous path at all — and that part the existing threshold already catches.
What it discards is everything **after** onset, where resistance falls
continuously as the wetted area grows. Salinity moves water resistivity by about
six decades on its own, so an absolute reading cannot separate *more water* from
*saltier water*. But salinity is essentially constant during a single event, so a
probe reading 1 MΩ, then 200 kΩ, then 20 kΩ over ten minutes describes a
trajectory — and a boolean cannot represent a trajectory.

`FS_LEAK_ENABLE` then chooses between ignoring it (0), warning (1), or entering
SURFACE mode (2). None of those options cut power, and none of them are informed
by *how bad* it is, because that information was never measured.

**The hardware never produced a number, so no firmware fix can recover one.**

---

## 2. Why this is not a trivial fix

### 2.1 DC does not work

Put a steady voltage across two electrodes in water and you are running
electrolysis. Metal leaves one electrode, gas forms at the other, and within
hours the reading drifts because the electrodes are no longer the electrodes you
calibrated. The measurement also stops meaning "how much water" and starts
meaning "how much chemistry has happened so far."

So the excitation has to be AC, with **zero average current** — enforced by a
series capacitor, which physically cannot pass DC, rather than by trying to
balance a waveform in firmware.

(Honesty: this matters less than it first appears, because the probes are dry
almost all the time and no charge transfers through dry probes. It is still the
right call — it costs one capacitor — but it is not the load-bearing argument.)

### 2.2 No single frequency covers the range

Two effects fight each other, and both move when the water resistance moves.

**At the low end**, the electrode/water interface behaves like a capacitor in
parallel with a resistor — the electrochemical double layer, modelled as a
Randles cell. Below some frequency this capacitance is a high impedance and the
measurement stops seeing the water at all. Raising the frequency gets past it.

**At the high end**, the tether and internal wiring have capacitance to ground —
roughly 200 pF for the run involved, using TI's SNLA164 figure of 12–16 pF/ft for
low-capacitance twisted pair. Above some frequency that capacitance is a lower
impedance than the water and shorts the measurement out. Lowering the frequency
avoids it.

Both corners scale with the water resistance, so the usable window **slides** as
the probe wets. A nearly dry probe (megohms) needs a low frequency; a flooded
probe (hundreds of ohms) needs a high one. The windows for the two extremes do
not overlap.

**Therefore: three tones — 10 Hz, 1 kHz, 20 kHz.** Not because more data is
nicer, but because no one frequency is valid across the range that has to be
measured. Each tone is valid over part of the range, and the firmware picks the
reading from whichever tone is in its valid window.

> Caveat, stated plainly: the double-layer values used in the analysis
> (Cdl ≈ 0.5 µF, Rct ≈ 100 kΩ) are plausible order-of-magnitude placeholders, not
> measured values for these electrodes in this water. The *shape* of the argument
> — two opposing limits that both move — is solid. The exact corner frequencies
> are not, and would need measurement on real probes.

---

## 3. Architecture

```
   ┌───────────────────────────────────────────────────────────────┐
   │   probe ──┐                                                   │
   │   probe ──┼──► FRONT END ────► RP2040 ────► I²C ──► Navigator │
   │   probe ──┘    mux, TIA,       measure,                       │
   │  (3 hulls)     4 gain ranges   log, report                    │
   │                                                               │
   │   probe ─────► BACKSTOP ──────────────────► leak line ──►     │
   │  (separate)    oscillator, peak det,       (no firmware)      │
   │                comparator                                     │
   └───────────────────────────────────────────────────────────────┘
```

**Two independent paths.** The front end is the informative one and depends on a
long chain — mux, op-amp, ADC, microcontroller, firmware — any link of which can
fail quietly. The backstop is nearly useless as information and very hard to
stop: an oscillator, a divider, a peak detector and a comparator, on its own
electrodes, powered upstream of the regulator.

Building both means a firmware fault cannot silently disable leak detection,
while a working system still gives the pilot something actionable. The two paths
fail for unrelated reasons. That is the entire point.

| Sheet | Parts | Role |
|---|---|---|
| Front end | 31 | Three-channel quantitative measurement |
| Backstop | 20 | Firmware-independent flood alarm |
| Power | 9 | Reverse protection, 3.3 V regulation, ADC supply |
| MCU | 18 | RP2040, boot flash, debug |
| I/O | 12 | I²C to Navigator, humidity, FRAM log, RTC |
| **Total** | **90** | 82 nets |

---

## 4. Front end

```
  GPIO ─► R4 ─► C1 ─► [drive bus] ─► probes ─► R1-R3 ─► MUX ─► TIA ─► filter ─► ADC
        100R  10µF        │                            8:1    4 ranges  1k/1nF
                          └──► cal references (100R, 1k, 1M)
```

### 4.1 Excitation

An RP2040 GPIO square-waves at the selected tone, through **100 Ω** (limits
current into a shorted probe, damps the edge so the drive trace radiates less)
and a **10 µF series capacitor** (guarantees zero DC through the water, and
converts the 0–3.3 V unipolar square into a ±1.65 V swing about whatever level
the network settles at).

10 µF is 1.6 kΩ at the lowest tone — negligible against the 3 MΩ range, not
negligible against the 100 Ω range, but the 100 Ω range only gets used on flooded
probes where the double layer rules out 10 Hz anyway. Adequate for every *valid*
tone/range pair.

### 4.2 Probes, protection, multiplexing

Each of the three probe returns passes a series resistor and a TVS/PESD part
before reaching a **CD4051B 8:1 multiplexer**. Three channels are probes, three
are on-board calibration references (100 Ω, 1 k, 1 M), and two are left
deliberately open.

The calibration channels are the reason the multiplexer's on-resistance doesn't
corrupt the result. A CMOS switch's Ron varies with temperature and with the
signal on it, so it can't simply be subtracted as a constant. Measuring a known
resistor **through the same switch, on the same sweep** gives the current Ron and
lets it be divided out. This requires re-calibrating every sweep, not once at
manufacture.

### 4.3 Transimpedance amplifier — the central decision

The instinct is to measure the probe as a voltage divider. Don't. A divider's
output node sits at a high, varying impedance, and the cable capacitance hanging
off it has to be charged and discharged through the water resistance — which is
exactly the high-frequency limit from §2.2, brought inside the board.

A **transimpedance amplifier** instead holds its summing node at a fixed
potential (VMID) and measures the *current* flowing into it. Because that node
never moves, the capacitance to ground on it never charges. The bandwidth
penalty from cable capacitance largely disappears: **965 Hz → 108 kHz** in
simulation.

Op-amp is the **OPA2333** — chopper-stabilised, microvolt-level offset, which
matters because on the 3 MΩ range the signal currents are picoamps and an
ordinary op-amp's offset would swamp them.

### 4.4 Four switched gain ranges

A single feedback resistor cannot span 100 Ω to 3 MΩ — four decades — without
either saturating on a flooded probe or vanishing into the noise on a dry one.
Four feedback resistors (**100 Ω / 3 k / 100 k / 3 M**) are selected by a
**TMUX1104** (8.8 Ω Ron at 3.3 V). The firmware autoranges: start high, step down
until the reading is on-scale.

These are **instrument ranges, not wetness categories** — chosen to tile the
four-decade span in roughly decade-and-a-half steps with overlap at each boundary,
the same way a multimeter's range switch works. No value here encodes a physical
meaning, and there is no calibration from ohms to how wet a probe is — see §11, item 2.

The 8.8 Ω of the range switch sits in the feedback path and is cancelled the same
way as the input mux, by the calibration channels.

### 4.5 VMID — the artificial zero

The board runs on a single 3.3 V supply, so there is no negative rail and a
signal that swings both ways has nowhere to go. **VMID is a buffered 1.65 V
reference that the analog section treats as zero.** A signal at 1.9 V is
"positive," 1.4 V is "negative." Everything analog references VMID, not ground.

This is also why the guard ring on the layout is tied to VMID rather than ground
(§8.4) — a guard has to sit at the same potential as the node it protects, or it
becomes a leakage path itself.

### 4.6 Anti-alias filter

A 1 kΩ / 1 nF single-pole RC ahead of the ADC input, placed at the ADC pin, not at
the op-amp. Its job is to stop content above half the sample rate from folding
down into the band of interest.

---

## 5. Hardware backstop

A completely separate circuit on its own probe pair and its own connector,
powered from **+5 V upstream of the LDO**, so a regulator failure that kills the
3.3 V rail does not kill the alarm.

```
  74LVC1G14 relaxation osc ──► DC block ──► 3.3k divider ──► BAT54W peak det
        ~4.46 kHz                              (probe)         (~100 ms hold)
                                                                    │
                                          leak line ◄── open-drain ◄┘
                                          (10k pull-up to +3V3)   TLV3011
                                                                  1.242 V ref
                                                                  69 mV hysteresis
```

- **Relaxation oscillator** from a single Schmitt-trigger inverter, one resistor
  and one capacitor — ~4.46 kHz. A Schmitt trigger has two different switching
  thresholds, so the RC charges to the upper one, flips, discharges to the lower,
  flips back, forever. Three parts, no crystal, no code.
- **DC block** — same reasoning as §4.1, zero average current through the water.
- **Divider** — the probe forms the lower leg against 3.3 kΩ. As the probe wets,
  its resistance falls and the AC amplitude at the tap falls with it. Trip point
  lands around **2.3 kΩ** of probe resistance.
- **Peak detector** — BAT54W and a capacitor turn the AC amplitude into a slowly
  decaying DC level, holding roughly 100 ms, so the comparator sees a steady
  voltage instead of a waveform.
- **Comparator** — TLV3011, with a 1.242 V internal reference and 69 mV of
  hysteresis so a level sitting near the threshold doesn't chatter. Open-drain
  output with a 10 k pull-up, which means it can only ever pull the leak line
  low — it cannot drive it high and cannot fight the Navigator.

The MCU also taps the backstop's output node (`LEAK_N`) on a GPIO, **read-only,
for logging.** It cannot drive it, disable it or acknowledge it.

Separate electrodes are non-negotiable here: sharing probes with the front end
would put the two circuits' excitation on the same node and make the
"independent" claim false.

---

## 6. Power

- **Reverse-polarity protection** by P-channel MOSFET in the supply path (drain
  toward the input). Lower drop than a series diode, and it fails open on
  reversal rather than dissipating.
- **5 V in → 3.3 V LDO** for the digital and analog sections.
- **Separate filtered supply for the ADC reference/supply pin** via ferrite bead
  placed at the pin, decoupled locally.
- **Backstop deliberately taps +5 V ahead of the LDO** (§5).

---

## 7. Microcontroller and I/O

**RP2040**, chosen for PIO — generating three precise excitation tones and
sampling in a fixed phase relationship to them is exactly what PIO state machines
are for, without tying up the CPU or fighting interrupt jitter. Also: two cores,
well-documented, and a boot flash rather than fused-in firmware.

Supporting parts: QSPI boot flash, 12 MHz crystal, SWD header.

On the I²C bus to the Navigator:

- **SHT45** — humidity and temperature. Humidity rising inside a sealed tube is
  the earliest available warning, before any probe is wet. Sits on a thermal
  island so the board's own dissipation doesn't bias it.
- **MB85RC64TA FRAM** — event log. Writes at bus speed with no erase cycle and no
  wear-out concern, so a leak event can be recorded in the moments before things
  go wrong.
- **PCF8563 RTC** — timestamps for that log.

---

## 8. Board

### 8.1 Stackup

Four layers on a Raspberry Pi HAT outline, **65.0 × 56.5 mm**, JLC04161H-7628.
Signal / **solid ground** / power / signal.

### 8.2 Floorplan

```
        0         14         28        40         52        65
      ┌─────────────────────────────────────────────────────────┐
 56.5 │  BACKSTOP STRIP                                         │
      │  J7 ──► osc ──► DC block ──► divider ──► peak ──► cmp ──► J6
 38.5 ├──────────────────┬───────────┬──────────────────────────┤
      │ WET  │ FRONT END │  POWER    │  MCU                     │
      │ EDGE │ mux, TIA, │  P-FET    │  RP2040, flash, crystal  │
      │ J1-3 │ ranges,   │  LDO      ├──────────────────────────┤
      │ TVS  │ VMID      │  ferrite  │  I/O   SHT45 FRAM RTC J5 │
    0 └──────┴─╌╌╌╌──────┴───────────┴──────────────────────────┘
               SHT45 thermal island
```

Signal flows left to right: outside world → analog → power as a physical buffer →
digital → Navigator. The backstop gets its own strip across the top with one
connector in and one out, so its independence is visible on the board rather than
asserted in a document.

### 8.3 The ground plane is not split

The instinct is to cut the ground sheet so the noisy digital half can't
contaminate the quiet analog half. That makes it worse. Return current does not
head for a star point; above a few tens of kilohertz it flows in the plane
directly beneath the trace it came along, because that is the lowest-inductance
path. Cut the plane and you force a detour, and the loop area that creates both
radiates and picks up.

**One solid, unbroken In1 ground.** Separation comes from distance and from
placement discipline, not from cutting copper.

### 8.4 Guard ring at VMID

The TIA summing node is a high-impedance node where the currents of interest can
be picoamps. Surface leakage across the board — flux residue, humidity — can
exceed them. A guard ring is a ring of copper around that node held at the *same
potential* as the node, so there is no voltage across the leakage path and
therefore no leakage current.

It is tied to **VMID, not ground**, because VMID is what the summing node sits
at. Grounding the guard would guarantee 1.65 V across exactly the gap it is meant
to protect.

### 8.5 Other layout rules applied

- SHT45 on a thermal island, slotted, away from the LDO.
- Analog nets (the summing node in particular) do not take vias; a via is a stub
  and an inductance on a node that cannot afford either.
- Drive-bus 10 µF and 100 Ω at the connector end, not the MCU end.
- The 1 k / 1 nF anti-alias pair at the ADC pin.
- Ferrite for the ADC supply at the ADC pin.

---

## 9. What this asks of firmware

The board does not fix anything by itself. It makes a fix possible. To use it,
firmware would need to:

1. Expose per-instance status out of `AP_LeakDetector` — the thing Jacob
   described in 2018.
2. Carry a *value* rather than a boolean across that boundary, so threshold and
   trend are policy decisions instead of compile-time constants.
3. Drive the three tones and autorange, re-reading calibration channels each
   sweep.
4. Report per-enclosure resistance and humidity over I²C, and log events to FRAM.

The backstop needs none of this and works if all of it is broken.

---

## 10. Verification

What was actually checked, and by what means:

| Item | How | By |
|---|---|---|
| Component pinouts | Read against manufacturer datasheets, symbol by symbol | Me |
| Netlist connectivity | Python validator over the 90-part / 82-net definition | Claude |
| Frequency-window argument | LTspice sweeps | Claude |
| TIA bandwidth claim | LTspice, 965 Hz → 108 kHz | Claude |
| Backstop trip point | LTspice | Claude |
| Layout rules | KiCad DRC, plus manual review against mixed-signal practice | Me |

**Not checked:** anything involving real water, real electrodes, or real
hardware. No board was fabricated.

---

## 11. Open items

Ranked by how much they would hurt.

1. **Double-layer parameters are placeholders.** Cdl ≈ 0.5 µF and Rct ≈ 100 kΩ
   are order-of-magnitude guesses. Real electrodes in real water would move the
   tone selection.
2. **There is no calibration from resistance to physical wetness**, and none is
   possible from a single probe pair, because salinity confounds it. A
   known-geometry conductivity reference — a second cell in a location that is
   supposed to stay dry — would fix this and is not on the board. As it stands the
   claim is limited to *trend within one event*.
3. **Cable capacitance is estimated**, not measured on a real tether.
4. **The multiplexer topology was never validated against a production reference
   design.** TI's SBOA521 takes a different approach to switched-gain TIAs.
5. **Per-sweep calibration has not been shown to converge** under realistic
   temperature drift.
6. **The 100 Ω range's interaction with the 10 µF drive capacitor** is argued
   away rather than measured.
7. **Backstop trip point (~2.3 kΩ) is arbitrary** — nobody has established what
   probe resistance corresponds to an actual emergency.
8. **No EMC consideration** beyond edge damping and layout hygiene.
9. **The board draws power from the vehicle** and nobody has budgeted it.
10. **Connector and probe mechanical design is out of scope entirely.**
11. **Thermal behaviour of the SHT45 island is asserted, not simulated.**
12. **No consideration of what happens on partial submersion of the board
    itself.**
13. **Three tones may be more than necessary** — two might cover the range if the
    real electrode parameters are friendlier than assumed.

---

## 12. References

- **Blue Robotics forum**, 2018 thread on per-sensor leak identification,
  including Jacob Walser's response.
- **ArduPilot / ArduSub source** — `libraries/AP_LeakDetector/`,
  `ArduSub/failsafe.cpp`.
- **TI SNLA164** — twisted-pair cable capacitance (12–16 pF/ft).
- **TI SBOA521** — switched-gain transimpedance amplifier reference.
- **Gamry Instruments** — electrochemical double-layer capacitance,
  20–60 µF/cm².
- **Altium Academy** — mixed-signal grounding and layout.
- Datasheets: OPA2333, CD4051B, TMUX1104, TLV3011, 74LVC1G14, BAT54W, RP2040,
  SHT45, MB85RC64TA, PCF8563.

---

*Design study. Not fabricated. See `LEARNING.md` for the concepts behind each
decision, `COMPONENTS.md` for the parts, and `DESIGN-FULL.md` for the long
version of this document.*
