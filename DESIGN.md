# Leakwatch — Design Document

**Per-enclosure leak detection for BlueROV / Navigator.**

A design study. Ninety parts, five schematic sheets, four-layer board. Simulated,
netlist-validated, and **never built**. Section 13 lists everything that is wrong
with it.

---

## Contents

1. [What this is](#1-what-this-is)
2. [The problem](#2-the-problem)
3. [Why it is not a trivial fix](#3-why-it-is-not-a-trivial-fix)
4. [Architecture](#4-architecture)
5. [Front end](#5-front-end)
6. [Hardware backstop](#6-hardware-backstop)
7. [Power](#7-power)
8. [Microcontroller](#8-microcontroller)
9. [I/O, logging, environment](#9-io-logging-environment)
10. [PCB design](#10-pcb-design)
11. [Requirements this hardware places on firmware](#11-requirements-this-hardware-places-on-firmware)
12. [Verification: what was checked and how](#12-verification-what-was-checked-and-how)
13. [Open items](#13-open-items)
14. [Decisions that changed, and why](#14-decisions-that-changed-and-why)
15. [Bill of materials](#15-bill-of-materials)
16. [Net list](#16-net-list)
17. [References](#17-references)

---

## 1. What this is

A sensor board that tells a BlueROV pilot **which** enclosure is leaking, and how
badly, instead of only that something somewhere is wet.

It plugs into the Navigator's I²C port. It measures three enclosures
independently and quantitatively. It carries a second, completely separate
circuit that trips the vehicle's leak line in hardware, with no firmware in the
path at all.

The design is unbuilt. It exists to demonstrate that the problem in section 2 is
solvable and to work out what solving it actually costs.

---

## 2. The problem

### 2.1 The request

In November 2018 a user asked on the Blue Robotics forum how to report leaks per
hull — "Leak detected in Electronics Hull" versus "Leak detected in Battery Hull."

Jacob Walser of Blue Robotics replied that the firmware supports three leak
detector instances but that there is no support for distinguishing which one has
fired on the user end, and suggested modifying firmware or filing a feature
request.

In December 2023 another user asked whether anything had changed. Nothing had.

From that thread: *"A leak detected warning is a showstopper for ROV ops."*

### 2.2 The code

`libraries/AP_LeakDetector/AP_LeakDetector.cpp`:

```cpp
bool AP_LeakDetector::update()
{
    uint32_t tnow = AP_HAL::millis();
    for (int i = 0; i < LEAKDETECTOR_MAX_INSTANCES; i++) {
        if (_drivers[i] != NULL) {
            _drivers[i]->read();
            if (_state[i].status) {
                _last_detect_ms = tnow;
            }
        }
    }
    _status = tnow < _last_detect_ms + LEAKDETECTOR_COOLDOWN_MS;
    return _status;
}
```

Three instances are read. Any one of them sets the same `_last_detect_ms`.
`_status` is a single boolean.

The channel identity exists at the sensor, travels up the wire, and is discarded
here. Parameters `LEAK1_PIN`/`LEAK1_LOGIC` through `LEAK3_*` configure three
independent inputs whose independence is then collapsed.

**This is a software decision, not a pin shortage.**

### 2.3 Why it matters operationally

A BlueROV2 with a battery enclosure and a camera enclosure has three volumes that
flood independently. The correct response differs in each case:

| Enclosure | Consequence of flooding | Correct response |
|---|---|---|
| Camera | Lost camera | Finish the dive if the mission allows |
| Battery | Lost battery, possible thermal event | Surface promptly |
| Electronics | Lost vehicle | Surface immediately, expect to lose control |

Today all three produce the same alarm, so every leak is treated as maximum
severity. After recovery there is no record of which enclosure was wet, which
makes root-causing a leak a matter of opening everything and looking.

A second limitation follows from the boolean: there is no way to see a leak
*developing*. A slow ingress that will become a flood in twenty minutes reads
identically to a dry vehicle until the moment it bridges the probe.

---

## 3. Why it is not a trivial fix

If the answer were "add two more connectors," it would already exist. The
interesting part is that measuring *how wet*, per channel, is physically harder
than measuring *wet or dry*.

### 3.1 DC does not work

> **In plain terms.** Push electricity one steady direction through water and two things go wrong. Ions pile up on the metal contacts within a few thousandths of a second, so the reading drifts while you're taking it. And a steady one-way current slowly dissolves the metal. This is why every leak probe you can buy just answers wet-or-dry: measuring *how* wet with DC is not a hard problem, it's an impossible one.

Apply a steady voltage across two electrodes immersed in water and ions migrate
to the electrode surfaces within milliseconds, forming an electrical double
layer. Two consequences:

- The measured resistance **walks** as the layer builds. There is no stable
  reading to take.
- A net DC current through an electrolyte **electrolyses** it and slowly
  dissolves the electrode metal. A probe that spends months energised in
  saltwater will not survive.

This is why existing leak probes are threshold devices. The limitation is
physical, not a design shortcut.

### 3.2 AC works, but no single frequency does

> **In plain terms.** So you wiggle the electricity back and forth instead. But now you have to pick how fast to wiggle, and neither extreme works. Wiggle slowly and the ion layer on the electrodes gets in the way. Wiggle quickly and the probe cable — which behaves a little like a tiny battery between its two wires — passes the signal around the water instead of through it, so a bone-dry probe looks flooded. There's a usable window in between, and it *moves* depending on how wet the probe is.

Model the probe as:

```
                  Rct              Rct
   drive ──┬── [ 100k ]─┬── Rwater ─┬─[ 100k ]──┬── sense
           │            │           │           │
          Cdl         (the        (the         Cdl
        [0.5µF]       thing        thing      [0.5µF]
           │        you want)    you want)     │
   electrode 1                            electrode 2
```

Each electrode contributes a double-layer capacitance `Cdl` (order 0.5 µF) in
parallel with a charge-transfer resistance `Rct` (order 100 kΩ). Separately, the
probe cable contributes roughly 200 pF between its two conductors, in parallel
with the water.

That produces a squeeze:

```
   LOW frequency   →  Cdl has high impedance and blocks the measurement.
                      At 100 Hz, 0.5 µF is 3.2 kΩ per electrode — 6.4 kΩ in
                      series with a 200 Ω flood you are trying to resolve.

   HIGH frequency  →  the cable's 200 pF shunts around the water.
                      At 200 kHz that is 4 kΩ, which reads as a permanent
                      flood on a perfectly dry probe.

   USABLE WINDOW   →  roughly a few hundred Hz to a few tens of kHz —
                      and the window MOVES depending on how wet the probe is,
                      because Rwater changes which term dominates.
```

**Design response:** three excitation tones — 10 Hz, 1 kHz, 20 kHz — each with a
validity band. Firmware measures at all three and uses whichever reading is
trustworthy for the impedance currently present. Conceptually identical to
auto-ranging on a multimeter, applied to frequency instead of gain.

### 3.3 The cable is worse than it looks

> **In plain terms.** The obvious way to measure the returning current is to run it through a resistor and read the voltage. That fails in a sneaky way: the voltage at that point moves around as the water changes, and the cable charges and discharges chasing that movement, stealing current that should have been measured. Worse, how much it steals depends on frequency — so the thing corrupting your measurement varies with the setting you're measuring at.

Cable capacitance does not merely add a parallel path — in a naive measurement
it actively steals the signal:

```
   NAIVE:  sense resistor to ground

   drive ──~~~~── [ WATER ] ──┬── Rsense ── GND
                              │
                     this node MOVES as Rwater changes
                              │
                         ═╪═  the cable capacitance sees that movement,
                          │   charges and discharges, and diverts current
                         GND  that should have flowed through Rsense
```

The amount stolen depends on frequency, which corrupts exactly the measurement
being made. The fix is in section 5.3.

---

## 4. Architecture

```
   ┌───────────────────────────────────────────────────────────────┐
   │  ELECTRONICS ENCLOSURE                                        │
   │                                                               │
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

Two independent paths. The front end is the informative one and needs firmware.
The backstop is the reliable one and needs nothing.

| Sheet | Parts | Role |
|---|---|---|
| Front end | 31 | Three-channel quantitative measurement |
| Backstop | 20 | Firmware-independent flood alarm |
| Power | 9 | Reverse protection, 3.3 V regulation, ADC supply |
| MCU | 18 | RP2040, boot flash, debug |
| I/O | 12 | I²C to Navigator, humidity, FRAM log, RTC |
| **Total** | **90** | 82 nets |

### 4.1 Why two paths instead of one good one

> **In plain terms.** One circuit tells you a lot but depends on a long chain — microcontroller, software, converter, switch, amplifier — and any link can quietly fail. The other tells you almost nothing but is very hard to stop. Building both means a software bug can't silently switch off leak detection, and when everything works you still get the useful version. The two fail for completely unrelated reasons, which is the point.

A measurement system that reports resistance is far more useful than a boolean,
but it depends on a microcontroller, firmware, an ADC, a mux and an op-amp — a
long chain, any link of which can fail silently.

A hardware comparator that trips a line is nearly useless as information and very
hard to stop working.

Building both, physically separated, means a firmware fault cannot silently
disable leak detection, and a working system still gives the pilot something
actionable. The two failure modes are uncorrelated, which is the entire point.

---

## 5. Front end

Signal flow:

```
  GPIO ─► R4 ─► C1 ─► [drive bus] ─► probes ─► R1-R3 ─► MUX ─► TIA ─► filter ─► ADC
        100R  10µF        │                            8:1     4 ranges  1k/1nF
                          └──► cal references (100R, 1k, 1M)
```

### 5.1 Excitation

> **In plain terms.** This is the push. A pin on the microcontroller flips between 0 V and 3.3 V thousands of times a second. A resistor softens the sharp edges of that flipping so the wire carrying it across the board doesn't act like a small radio transmitter. Then a capacitor sits in the path, and because a capacitor physically cannot pass a steady one-way push, it guarantees the electrodes never corrode.

**Decision: drive from a GPIO through 100 Ω and a 10 µF series capacitor.**

- The **100 Ω** (R4) limits current if a probe is shorted and damps the GPIO's
  edge, reducing radiated noise.
- The **10 µF series capacitor** (C1) guarantees zero average DC current through
  the water regardless of duty cycle, supply tolerance or GPIO asymmetry. A
  capacitor cannot pass DC — not "mostly doesn't," physically cannot. This is
  what prevents electrode corrosion, and it is not something that can be achieved
  by trying to balance the drive waveform in firmware.
- It also converts the 0–3.3 V unipolar square into a **±1.65 V bipolar** swing
  about whatever DC level the downstream network settles at, self-correcting for
  drift.

**Why 10 µF:** its impedance must be small compared to the loop it sits in. At
the lowest tone (10 Hz) a 10 µF capacitor is 1.6 kΩ. Against the 3 MΩ range that
is negligible; against the 100 Ω range it is not, but the 100 Ω range is used for
flooded probes where the double layer already rules out 10 Hz. The value is
adequate for every *valid* range/tone combination.

### 5.2 Probe protection and multiplexing

> **In plain terms.** Wires that leave the board and run out to metal contacts pick up static electricity, so every one of them gets a clamp that dumps anything dangerous to ground before it reaches the delicate chips, plus a small resistor as a second line of defence. Then a switch chip lets one measuring circuit look at each probe in turn — which is what makes it possible to say *which* enclosure is wet rather than just *that* one is.

Each probe return gets a **100 Ω series resistor** (R1–R3) and a
**PESD5V0S1BA bidirectional TVS** (D1–D3). The drive bus gets its own clamp (D5).

**Why bidirectional TVS:** the probe lines genuinely swing negative — the DC
block centres the waveform on zero. A unidirectional part would clamp the signal
itself. **Why 5 V standoff:** the swing is ±1.65 V at the front end and ±2.5 V at
the backstop; a 3.3 V part would clip, a 5 V part will not.

**Why the series resistor is a compromise:** larger is better protection and
worse measurement, since it adds directly to the resistance being measured. 100 Ω
against a 200 Ω flood is a 50% error — which is why the calibration channel in
5.5 exists.

**Multiplexer: CD4051B, 8:1.** Channel assignment:

| Channel | Connected to | Purpose |
|---|---|---|
| Y0 | Probe 1 (electronics) | measurement |
| Y1 | Probe 2 (battery) | measurement |
| Y2 | Probe 3 (camera) | measurement |
| Y3 | *open* | spare |
| Y4 | R5, 100 Ω | short-cal: emulates a zero-ohm probe |
| Y5 | R6, 1 MΩ 0.1% | high-range reference |
| Y6 | R7, 1 kΩ 0.1% | low-range reference |
| Y7 | *open* | spare |

**Why CD4051B:** cheap, available, and it has a `VEE` pin, which lets it pass
signals below its ground reference — necessary for a bipolar swing on a single
supply. **What it costs:** on-resistance of ~470 Ω typical at 5 V and unspecified
below 3 V. Section 5.5 explains why that is survivable.

### 5.3 Transimpedance amplifier — the central decision

> **In plain terms.** This is the clever bit. Instead of letting the measuring point move around (and letting the cable steal from it), the amplifier watches that point and constantly pushes back exactly hard enough to hold it perfectly still. A point that never moves can't charge the cable, so the cable stops stealing. And the amount of pushing-back the amplifier has to do *is* the measurement. This one idea is what lets the board work at useful frequencies at all.

The probe produces a **current**. The ADC reads **voltage**. The naive converter
is a resistor to ground, which fails for the reason in 3.3.

```
   TIA:                            ┌──── Rf ────┐
                                   │            │
   probe current ────────── SUM ───┤     -      │
                             │     ├───►  \     │
                        ═╪═  │     │       \____┴──► TIA_OUT
                  cable  │   │     ├───►   /
                        GND  │     │      +
                             │     └──────┘
                             │        │
                            GND      VMID
```

The op-amp's entire job is to hold `SUM` equal to `VMID`, which it does by
sourcing or sinking whatever current is needed through `Rf`. Consequences:

- `SUM` **never moves**, so the cable capacitance never charges, so it steals
  nothing. The cable's effect on the measurement is removed rather than
  compensated.
- All probe current flows through `Rf`.
- `TIA_OUT = VMID − (I_probe × Rf)`.

**Measured benefit (simulated):** the frequency at which cable capacitance
corrupts the reading moved from **965 Hz to 108 kHz — a factor of 112.** That
single change is what makes a 20 kHz tone usable at all, and therefore what makes
the three-tone scheme possible.

The node is a *virtual ground*: it behaves like ground without being connected to
it.

### 5.4 Four switched gain ranges

> **In plain terms.** A flooded probe and a dry probe differ by about fifteen thousand to one. That's like asking one set of bathroom scales to weigh both a letter and a car. Turn the sensitivity up enough to see a dry probe and a flooded one goes off the end of the scale; turn it down enough to read the flood and the dry probe registers as nothing. So there are four sensitivity settings and the microcontroller picks whichever keeps the reading on the dial — exactly how a multimeter auto-ranges.

A flooded probe is ~200 Ω. A dry probe is several MΩ. Four decades of current.
One feedback resistor cannot span it — too small and the dry reading is buried in
noise, too large and the flooded reading slams into the rail.

| Range | Rf | Target condition |
|---|---|---|
| 1 | 100 Ω | flood |
| 2 | 3 kΩ | wet |
| 3 | 100 kΩ | damp |
| 4 | 3 MΩ | dry |

Switched by a **TMUX1104** (U3), selected on `RANGE_0`/`RANGE_1`.

**Why a TMUX1104 and not another CD4051B:** this switch is inside the feedback
loop, so its on-resistance **adds directly to the gain resistor**. The TMUX1104
measures 8.8 Ω typical at 3.3 V; a CD4051B would add hundreds of ohms and destroy
the 100 Ω range entirely. Its 4:1 width is exactly what four ranges need.

**Why the mux is on the output side, not the summing-node side:** placing it
between the range resistors and `TIA_OUT` keeps the summing node itself free of
the switch's capacitance, which matters for the reason in 5.7.

### 5.5 Calibration channels — how the mux's resistance is cancelled

> **In plain terms.** The measuring circuit has resistance of its own, and it drifts as the board warms up. That means you can't tell "the water changed" from "my own electronics warmed up." The fix is three fake probes soldered onto the board — resistors of known value. Measure one of those with the exact same circuit and whatever you read that *isn't* the known value is your circuit's own contribution. Subtract it. These resistors are the reason the numbers mean anything at all.

Three of the eight mux channels connect to on-board references rather than
probes. The short-cal channel is the important one:

```
   cal path (Y4):    drive ── R5 100Ω ── mux ── SUM      reads  R5 + Ron
   probe path (Y0):  drive ── water ── R1 100Ω ── mux ── SUM   reads  Rwater + R1 + Ron

   R1 = R5 = 100 Ω, so:

       probe reading − cal reading = Rwater      exactly
```

The mux's on-resistance, the series protection resistor, and any drift in either
subtract out. **This is what makes the CD4051B acceptable despite its high
on-resistance** — and it is a firmware requirement, not a free property: the cal
must be re-measured every sweep, because on-resistance drifts several tenths of a
percent per °C.

The 1 kΩ and 1 MΩ 0.1% references (R6, R7) serve a different purpose: they verify
the gain ranges and the ADC scale, and they detect a stuck or leaky range switch.

**Why 0.1% for those two and 1% for everything else:** they define measurement
accuracy. Everything else only needs to be repeatable.

### 5.6 VMID — the artificial zero

> **In plain terms.** The signal needs to swing both up and down, but the board only has 0 V and 3.3 V — there's no such thing as "below zero" here. So the circuit simply redefines zero as 1.65 V, exactly halfway up. Now "down" means "toward 0 V" instead of "below ground," and everything works. Two resistors split the supply in half, a capacitor cleans it up, and an amplifier holds it steady so nothing can drag it off centre.

The board runs on a single 3.3 V supply. The signal is AC. Without a negative
rail the bottom half of every waveform would be clipped at 0 V.

```
     +3.3V ────────────────────────────  ceiling

     1.65V ~~~~/\~~~~\/~~~~/\~~~~\/~~~   signal lives here
           ─ ─ ─ the new "zero" ─ ─ ─
        0V ────────────────────────────  floor
```

`VMID` is made by a **10 kΩ/10 kΩ divider** (R12/R13) filtered by **C6** and
buffered by the **second half of the OPA2333** (U2B) as a unity-gain follower.

**Why buffer it:** a bare divider has 5 kΩ of output impedance. Anything drawing
current from it drags the reference, and the reference *is* the measurement's
zero. The follower makes it stiff.

**Why the filter matters more than it looks:** the ADC measures `TIA_OUT` against
**ground**, not against `VMID`. So if `VMID` moves, the whole output moves with
it and the ADC reads that displacement as signal. There is no common-mode
rejection anywhere in the chain.

```
                     before      after
        VMID         1.650  ──►  1.655
        TIA_OUT      1.650  ──►  1.655       rides along
        ADC reads:   1.650  ──►  1.655       ...and calls it signal
```

VMID noise is measurement error, one for one, and it must be quiet at every
frequency being measured — including 10 Hz. See open item in 13.

### 5.7 Feedback capacitor

> **In plain terms.** Left alone, the amplifier overshoots when the signal changes suddenly, corrects, overshoots the other way, and rings like a struck bell. That happens because the information telling it to stop arrives slightly late. This capacitor gives that information a shortcut so it arrives on time.

`C5` (2.2 pF) sits across the feedback path.

The summing node carries capacitance to ground — cable, mux, op-amp input,
PCB stray. That capacitance together with `Rf` delays the feedback, and delayed
feedback makes the amplifier overshoot, correct, and overshoot again. A small
capacitor across `Rf` gives the feedback a fast path around the resistor.

```
   without Cf:        ___
                     /   \      /\
        step ───────/     \____/  \___    ringing

   with Cf:          ______________
                    /
        step ───────/                     clean
```

**This is a known-deficient part of the design.** One fixed capacitor cannot
serve four decades of `Rf`. See section 13, item 2.

### 5.8 Anti-alias filter

> **In plain terms.** A resistor and a capacitor smoothing the signal before it reaches the microcontroller's measuring input, so the digitiser sees a clean value instead of something jittery. The resistor also protects the amplifier, which doesn't like driving a capacitor directly.

`R14` (1 kΩ) and `C7` (1 nF) form a single-pole low-pass at 159 kHz ahead of the
ADC. **This value was never dimensioned against a chosen sample rate.** See
section 13, item 3.

### 5.9 Op-amp choice: OPA2333

> **In plain terms.** The amplifier itself. It was picked for being unusually stable — it doesn't drift as the board heats up, which matters when a measurement runs for hours. The price is that it's slow, and that slowness is the direct cause of two of the known problems at the end of this document.

| Property | Value | Why it matters here |
|---|---|---|
| Zero-drift (chopper) | 10 µV offset, 0.05 µV/°C | No offset trim, no thermal drift in a measurement that runs for hours |
| Rail-to-rail in and out | 0.1 V beyond rails | VMID at half supply works, and output can swing the full range |
| Input bias current | ±70 pA | Against 3 MΩ that is 210 µV of error — negligible |
| Supply | 1.8–5.5 V | Runs from 3.3 V directly |
| Dual | two amps in MSOP-8 | One package does the TIA and the VMID buffer |

**What it costs:** 350 kHz gain-bandwidth and 0.16 V/µs slew rate are both slow.
The GBW limits usable bandwidth on high gain ranges; the slew rate limits
amplitude at the 20 kHz tone (section 11).

**Noise:** at 3 MΩ the feedback resistor's own Johnson noise is 223 nV/√Hz
against the op-amp's 10 nV/√Hz. The resistor dominates by 22×. That is inherent
to any transimpedance amplifier at high gain and is not a defect — over an 8.6 kHz
bandwidth it works out to ~21 µV RMS against a signal of order volts.

---

## 6. Hardware backstop

An entirely separate circuit sharing only the ground plane and the 5 V rail. Six
stages, left to right:

```
  U10 osc ──► C29 DC block ──► J7 probe ──► R24 divider ──► D8 peak ──► U11 cmp ──► J6
  ~4.5 kHz      1µF            in/out        3.3k          C30/R25    TLV3011    leak line
```

### 6.1 Oscillator

> **In plain terms.** The backstop needs its own back-and-forth push, and crucially it needs one that exists without any software. A single logic chip with one resistor and one capacitor does it: the chip charges the capacitor until it's full enough, flips, drains it, flips back, forever. No clock, no configuration, nothing to program. It runs the moment it has power, which is the entire reason the backstop exists.

**74LVC1G14 Schmitt inverter with one resistor and one capacitor.**

The inverter's output charges C27 through R21. At the upper Schmitt threshold the
output flips low, the capacitor discharges to the lower threshold, and it flips
again. Period ≈ 1.015 × R × C.

With R21 = 47 kΩ and C27 = 4.7 nF: **T ≈ 224 µs, f ≈ 4.46 kHz.**

**Why this and not a timer IC or an MCU pin:** it has no clock input, no
configuration, no register, and nothing to program. It oscillates whenever it has
power. That is the property the whole backstop exists to have.

**Why 47 kΩ:** bounded below by the gate's output impedance (~50 Ω — R must
dominate so timing is set by the resistor, not the chip) and above by input
leakage (1 µA across 47 kΩ is 47 mV against a ~700 mV threshold swing, about 7%).
Anywhere from 10 kΩ to 100 kΩ works; 47 kΩ is central.

**Why ~4.5 kHz specifically:** it sits in the same window described in 3.2.

```
   at 4.5 kHz:  2 × Cdl  =  141 Ω    negligible against the 3.3 kΩ divider
                cable    =  177 kΩ   negligible in parallel with the probe

   at 200 kHz:  2 × Cdl  =    3 Ω    better
                cable    =    4 kΩ   LARGER than the divider — a dry probe
                                     would read as a permanent flood
```

The frequency is not arbitrary and cannot be raised for convenience.

### 6.2 DC block

> **In plain terms.** Same job as the capacitor in the front end's excitation stage — guarantee that as much electricity goes one way as the other, so the backstop's own electrodes don't corrode either.

`C29`, 1 µF in series, for the same reason as the front end's C1: zero average
current, no electrode corrosion. It also converts the 0–5 V square into a
**±2.5 V** swing.

**Why 1 µF and not 100 nF:** its impedance must be small against the loop it sits
in. The loop at the trip point is roughly 2.2 kΩ + 200 Ω + 3.3 kΩ ≈ 5.7 kΩ. One
twentieth of that is 285 Ω, requiring ≥70 nF at 4.5 kHz. 100 nF technically
passes; 1 µF gives margin for free.

### 6.3 Divider — where "how wet" becomes a voltage

> **In plain terms.** Two resistances in a line share the voltage between them in proportion to their size, like a see-saw. One side is the water (unknown), the other is a resistor we chose. Wet water is a small resistance so it takes almost none of the voltage and our resistor gets nearly all of it — a big signal. A dry probe hogs virtually everything and our resistor gets almost nothing. The size of the wiggle across our resistor is the measurement.

```
   BS_DRIVE ── R22 100R ── [ WATER ] ── R23 100R ──┬── BS_DIV
                                                    │
                                                  R24 3.3k
                                                    │
                                                   GND

   V_DIV = 2.5 V × R24 / (Rwater + R22 + R23 + R24)
```

| Water | R24's share | Voltage |
|---|---|---|
| 300 Ω (flooded) | 3300 of 3800 = 87% | 2.17 V |
| 2200 Ω (threshold) | 3300 of 5700 = 58% | 1.45 V |
| 10 MΩ (dry) | ~0.03% | 0.0008 V |

**R24 = 3.3 kΩ is the one component that sets the specification.** It was chosen
from a simulated sweep:

| R24 | Trips below | Releases above |
|---|---|---|
| 2.2 kΩ | 1.39 kΩ | 1.69 kΩ |
| **3.3 kΩ** | **2.16 kΩ** | **2.60 kΩ** |
| 4.7 kΩ | 3.16 kΩ | 3.85 kΩ |

3.3 kΩ places the trip in the empty gap between condensation on a cold surface
and actual liquid water. It is a 1% thin-film part; everything else in the
backstop is ordinary 1% thick film.

### 6.4 Peak detector

> **In plain terms.** The decision-making chip further along can only compare steady voltages, but what we have is a wiggle. So: a diode, which is a one-way valve, and a capacitor, which is a bucket. Every time the wiggle swings positive it pushes a little charge through the valve into the bucket. The bucket then sits at roughly the height of the wiggle. A resistor slowly drains it so it can fall again when things dry out.

`D8` (BAT54W Schottky) charges `C30` (100 nF) on positive half-cycles; `R25`
(1 MΩ) bleeds it back down.

**Why a Schottky:** its forward drop (~0.2 V at these currents) subtracts
directly from the measured peak, so a silicon diode's 0.7 V would waste a third
of the signal. **The forward drop is the least predictable term in the trip
threshold** — it is why the trip resistance is quoted as a range (2.0–2.3 kΩ)
rather than a number.

**Why R25 × C30 = 100 ms:** two things at once. At 4.5 kHz the capacitor is
refreshed every 222 µs and droops only ~2 mV between peaks, so the DC level
tracks the AC amplitude cleanly. And once an alarm starts, the hold guarantees a
**minimum 100 ms assertion** even for a momentary bridge — long enough that the
flight controller cannot miss it.

### 6.5 Comparator and hysteresis

> **In plain terms.** A comparator is a chip that answers one question: is this voltage bigger than that one? Feed it the bucket level and a fixed threshold and it decides wet or dry. The extra trick is hysteresis: a little of the output is fed back to the threshold, so once the alarm fires the bar for switching it off is lower than the bar for switching it on. Without that, a probe sitting exactly at the threshold would flicker the alarm on and off many times a second.

**TLV3011**, chosen because it contains a **1.242 V ±1% reference** on its own
pin. The alternative — dividing the 5 V rail to make a threshold — would work on
a bench supply and fail on a vehicle, where the 5 V rail sags when thrusters draw
current.

```
              U11 TLV3011
            ┌──────────────┐
   BS_PK ───┤ IN−      OUT ├─── LEAK_N ──┬── R28 10k ── +3V3
            │              │             ├── J6.1 ──► Navigator
      ┌─────┤ IN+          │             └── GPIO7 (monitor only)
      │     │ REF ├ 1.242V │             │
      ├─ R26 47k ── REF    │             │
      └─ R27 2.2M ─────────────────────┘
```

Peak on `IN−`, threshold on `IN+`. Wet → peak rises above threshold → output
pulls low. **Active low**, hence `LEAK_N`. Swapping the inputs would make the
hysteresis push the wrong way and produce the chatter it exists to prevent.

**Hysteresis** via R27 from the output back to `IN+`:

```
   output released (pulled to 3.3 V):  VTH = 1.285 V
   output pulled low (0 V):            VTH = 1.216 V
                                       hysteresis = 69 mV
```

which translates to roughly a 300 Ω dead band in probe resistance. Without it, a
probe sitting exactly at threshold would toggle the leak line at audio rate, and
ArduSub's failsafe would see a leak appearing and disappearing many times a
second — arguably worse than either a steady alarm or none.

**Why 47 kΩ and 2.2 MΩ:** large enough not to load the reference, small enough
not to be dominated by input bias. The ratio sets the hysteresis depth.

### 6.6 Output stage

> **In plain terms.** The comparator's output can pull the alarm line down to zero but cannot push it up — it's half an output. A resistor to 3.3 V does the pushing-up. That sounds like a limitation but it's deliberate: several of these can share one wire with any of them able to raise the alarm, the resistor can go to a different voltage than the chip runs on, and — the important one — if the 3.3 V regulator dies the chip can still pull the line down and shout.

The TLV3011 output is **open-drain** — internally one NMOS transistor between the
pin and ground. It can pull low or let go; it cannot drive high. `R28` (10 kΩ to
3.3 V) defines the released state.

Three reasons this is the right output:

1. **Wired-OR.** Several open-drain outputs can share one line. Any one pulling
   low pulls the line low. A second backstop for another enclosure needs no logic
   gate — just connect it.
2. **Level domain.** The comparator runs on 5 V; the pull-up is on 3.3 V. That is
   what makes the RP2040 monitoring tap legal, since RP2040 pins are **not** 5 V
   tolerant.
3. **Asymmetric failure.** If the 3.3 V regulator dies, the pull-up disappears
   but the comparator — running from 5 V — can still pull low. **The
   alarm-asserting direction survives a regulator failure.**

### 6.7 Why the backstop needs its own electrodes

> **In plain terms.** An earlier version had the backstop sharing the front end's probes. That turned out to break the entire project: joining the sense wires together to make one alarm also joins the three channels, so water in the camera enclosure would show up on all three measurements. And you can't fix it with resistors in between, because anything big enough to separate the channels is far too big for the backstop to work through. Independence had to be physical, not clever.

This was the single largest correction made during the project.

An earlier revision had the backstop sharing the front end's three probes, with
all three sense lines commoned into one node as a deliberate wired-OR. That is
incompatible with the front end in two independent ways:

1. **Commoning the sense lines shorts the three channels together**, destroying
   exactly the channel identity the project exists to recover. Water in the
   camera enclosure would read on all three front-end channels.
2. **Two low-impedance sources on one drive node fight each other**, and the
   backstop's 4.5 kHz would land inside the front end's measurement band.

Isolation resistors do not fix (1): the backstop trips against a 3.3 kΩ divider,
so any series resistance large enough to isolate the channels (≥100 kΩ) would
prevent it from ever tripping.

**Conclusion: independence has to be physical.** The backstop gets its own
two-electrode probe (J7) in the electronics enclosure, its own clamps, and its
own drive.

**Why only the electronics enclosure:** it is the one where flooding is
immediately fatal to the vehicle. The battery and camera enclosures are covered
by the front end with full channel identity. Extending the backstop to all three
is possible — J7's nets simply parallel onto additional connectors — at the cost
of two more penetrator conductors per enclosure and four more TVS.

---

## 7. Power

```
   J5.1 ──► +5V_IN ──► Q1 ──► +5V ──┬──► U4 LDO ──► +3V3 ──► FB1 ──► 3V3_ADC
                    reverse         │
                    protection      └──► backstop (own branch from a star point)
```

### 7.1 Reverse polarity protection

> **In plain terms.** If someone plugs the power in backwards, this transistor blocks it. The orientation is the whole trick and it's the opposite of what looks natural — get it the wrong way round and it conducts happily under reverse power, which is exactly the situation it was fitted to prevent.

`Q1`, a **DMG3415U P-channel MOSFET**, with `R15` (100 kΩ) holding the gate at
ground.

**Orientation is critical and counterintuitive: drain faces the input, source
faces the load.**

A P-channel MOSFET's body is N-type and tied to the source; the drain is P-type.
The body diode therefore has its anode at the **drain** and conducts D→S.

```
   CORRECT (drain to input)           reverse polarity applied
                                      D = −5 V, S floats ≈ 0 → Vgs = 0 → OFF
   IN ──D ─┤├─ S── LOAD               body diode: anode −5, cathode 0
           G                          → REVERSE BIASED, blocks ✓
           └── R15 ── GND

   WRONG (source to input)            reverse polarity applied
                                      Vgs = +5 → FET OFF, but...
   IN ──S ─┤├─ D── LOAD               body diode: anode ≈ 0, cathode −5
           G                          → FORWARD BIASED
           └── R15 ── GND             → current flows through the load backwards
                                      → protection fails
```

Normal operation with the correct orientation still works: the body diode
conducts first, pulling the source to ~4.4 V, which makes Vgs ≈ −4.4 V and turns
the channel fully on, shorting out the diode drop.

**Why a P-FET and not a series diode:** a Schottky would drop 0.3 V continuously
and dissipate power; the FET drops millivolts.

### 7.2 Regulation

> **In plain terms.** Converts the incoming 5 V down to the 3.3 V most of the board runs on, with capacitors either side to keep it steady.

**AP2112K-3.3**, SOT-23-5, 600 mA. `C9` and `C10` at the input and output pins.
`EN` tied to `VIN` for always-on.

Dissipation is (5 − 3.3) × I. At 50 mA that is 85 mW and a ~17 °C junction rise;
at 150 mA it is 255 mW and ~51 °C. The real load should sit near the bottom of
that range — see section 13, item 8.

### 7.3 ADC supply isolation

> **In plain terms.** The measuring converter uses its own supply voltage as the yardstick it measures against, so noise on that supply becomes noise in every reading. A ferrite bead — which behaves like a plain wire for steady current but like a resistor for fast noise — gives the converter a quieter version of the same rail.

`FB1`, a 600 Ω @ 100 MHz ferrite bead, separates `3V3_ADC` from the main 3.3 V
rail, with `C11` (1 µF) and `C12` (100 nF) on the isolated side.

**Why:** the RP2040's ADC reference is its `ADC_AVDD` pin. Digital switching
noise on the main rail appears directly as measurement noise. A ferrite is a
resistor at high frequency and a wire at DC, which is exactly the required
behaviour.

### 7.4 Backstop supply — a deliberate choice

> **In plain terms.** The backstop runs from the incoming 5 V rather than the regulated 3.3 V, deliberately taken from before the regulator. If that regulator ever fails, the microcontroller goes dark but the backstop keeps running and can still raise the alarm. The failure mode leans in the safe direction.

The backstop runs from **+5 V, upstream of the regulator**, taken as its own
branch from a star point at the FET drain rather than daisy-chained through the
LDO input.

**Why:** if the LDO fails, the MCU dies but the backstop keeps oscillating and
comparing, and — because pulling low is the alarm direction and needs no pull-up
— it can still assert. The regulator is removed from the backstop's critical
path. This strengthens the independence claim materially, and it costs one
trace.

**Why the pull-up stays on 3.3 V despite that:** pulling up to 5 V would put 5 V
on `LEAK_N`, which the RP2040's GPIO7 tap cannot tolerate and the Navigator's
3.3 V input should not see. Splitting the domains this way gets the failure-mode
benefit without the voltage hazard.

---

## 8. Microcontroller

**RP2040** (U5) with **W25Q128JVSIQ** 16 MB QSPI flash (U6) and a 12 MHz crystal.

### 8.1 Why the RP2040

> **In plain terms.** The microcontroller. Chosen mainly because it has dedicated hardware for generating precisely timed waveforms without the processor having to babysit them, which is exactly what generating three measurement tones needs. Its weakness is a fairly poor built-in measuring converter — but the four sensitivity ranges mean it never has to cover a wide span at once, so a poor converter is good enough.

| Reason | Detail |
|---|---|
| **PIO** | The programmable I/O blocks generate the excitation waveforms with deterministic timing and zero CPU load — the single strongest argument for this part in this application |
| Dual core | One core can run the measurement loop while the other handles I²C |
| Cost and availability | Cheap, widely stocked, exceptionally well documented |
| 3.3 V native | Matches the rest of the board |

**What it costs:** the on-chip ADC is mediocre — roughly 8.7 effective bits with
a documented differential-nonlinearity problem.

**Why that is acceptable here:** the four gain ranges mean each range covers only
about 1.5 decades, so roughly 8 usable bits per range is sufficient. **The
ranging architecture is what rescues the ADC.** A single-range design would need
an external 16-bit converter.

Only GPIO26–29 reach the ADC; `ADC_IN` uses GPIO26 (ADC0).

### 8.2 Support circuitry

> **In plain terms.** The bits every microcontroller needs: a crystal to keep time, a memory chip holding its program, a reset button, a connector for programming it, and a small capacitor beside every power pin acting as a local reservoir.

| Element | Detail | Reason |
|---|---|---|
| RUN | 10 kΩ to +3V3, 100 nF to ground, button across the cap | Raspberry Pi's reference reset circuit. The cap debounces and provides a clean POR ramp |
| Crystal | 12 MHz, 15 pF loads, 1 kΩ in series with XOUT | Series resistor limits crystal drive level, preventing long-term ageing |
| BOOTSEL | Button pulling QSPI_SS low through 1 kΩ | Standard entry to the USB bootloader |
| SWD | 3-pin header | Programming and debug |
| Decoupling | 100 nF per supply pin | The RP2040 has multiple IOVDD and DVDD pins; each needs its own |
| VREG_VOUT | 1 µF reservoir, feeding DVDD | The internal 1.1 V core regulator's output |


In quad mode all four lines are bidirectional, but the **boot sequence starts in
single-SPI**, where SD0 is output-only and SD1 is input-only. Crossing them means
the RP2040 drives commands into the flash's output pin and listens on its input
pin, and the second-stage bootloader never loads. An earlier revision of this
netlist had exactly that error.

### 8.3 GPIO Assignments

> **In plain terms.** Which microcontroller pin does what. Worth reading for one entry — the pin watching the backstop's alarm line must be configured as an input forever. If software ever made it an output it would fight the backstop, which defeats the purpose of having one.

| Signal | Direction |
|---|---|
| EXC_DRIVE | out |
| MUX_A / B / C | out |
| RANGE_0 / RANGE_1 | out |
| **BACKSTOP_N monitor** | **in — permanently** |
| RTC_INT_N | in |
| I2C_SDA / SCL | bidir |
| ADC_IN | analog in |

GPIO7 taps the backstop's output line. **It must never be configured as an
output** — driving it high against the comparator pulling low would short the
RP2040's driver into the comparator's. RP2040 pins come out of reset as inputs,
so the default is safe, but this is a hard firmware constraint.

**Why the tap exists at all:** so firmware can notice the backstop fired and
write a timestamped entry to the log. It has no authority over the alarm; if the
MCU is hung, crashed, or unprogrammed, the leak line still trips and only the log
entry is lost.

---

## 9. I/O, logging, environment

Connected to the Navigator over I²C through a 4-pin JST-GH (J5): +5 V, SCL, SDA,
GND.

### 9.1 Bus

> **In plain terms.** The communication link to the host. Two wires plus two resistors — the resistors are necessary because devices on this kind of bus can only pull the wires down, never push them up.

`R19` and `R20`, 4.7 kΩ to +3V3. I²C devices can only pull a line *down*; nothing
on the bus drives high. Without pull-ups the bus floats and nothing communicates.

Device addresses: SHT45 at 0x44, FRAM at 0x50, RTC at 0x51 — mutually distinct.
See section 13, item 5 for the collision risk with the Navigator's existing bus.

### 9.2 Humidity and temperature — SHT45

> **In plain terms.** The only part of the board that can warn you *before* there's a leak. A probe needs an actual puddle to react to; the humidity inside a box starts climbing the moment a seal begins to fail, often long before liquid water reaches the bottom.

**Why it is on the board at all:** humidity trending detects a slow ingress
*before* liquid water bridges a probe. A probe is a threshold on a puddle; the
enclosure's relative humidity starts climbing the moment a seal begins to fail.
This is the only part of the design that can warn *before* the leak.

It also enables temperature correction of the water measurement — water
conductivity changes roughly 2%/°C — though that correction is not yet specified
(section 13).

Its placement on the PCB is more constrained than its schematic: see 10.6.

### 9.3 Event log — MB85RC64TA FRAM

> **In plain terms.** A memory chip that records what happened. This particular type was chosen because it can be rewritten essentially forever — ordinary memory chips wear out after enough writes, and this one gets written on every single measurement.

**Why FRAM and not EEPROM or flash:** the log is written on every sweep. EEPROM
endures ~1 million write cycles; flash fewer. FRAM's write endurance is
effectively unlimited (10^12+ cycles), it writes at bus speed with no erase
cycle, and it needs no wear levelling.

`A0`, `A1`, `A2` are tied to ground, setting address 0x50.  

`WP` is tied to ground — write-enabled — which is the required state for a
logging device. Grounding it explicitly records the choice rather than relying on
the internal pull-down.

### 9.4 Real-time clock and backup

> **In plain terms.** A clock chip with a supercapacitor and a one-way valve, so it keeps time for about five days after the vehicle loses power. Without it, every log entry would be stamped "42 seconds after boot" instead of a real date and time — which is the difference between evidence and a shrug when you're working out what went wrong.

**PCF8563T** with a 32.768 kHz crystal, backed by a **0.1 F supercapacitor**
through a **BAT54 Schottky**.

```
   +3V3 ──► D4 anode ─►│─ cathode ──┬── C26  0.1 F supercap
                                     ├── U9.VDD (the RTC's ONLY supply)
                                     └── C25  100 nF
```

**Powered up:** current flows through D4, charging C26 and running the RTC.

**Power lost:** the 3.3 V side collapses. Without the diode, C26 would dump its
charge backwards into the dead rail and every other load on it, and the clock
would stop within milliseconds. D4 blocks that path, stranding C26 on the RTC's
side with nowhere to go but into the RTC.

**Runtime:** the PCF8563 draws ~0.25 µA keeping time. Draining 0.1 F from 3.0 V
to the part's 1.8 V floor is 0.12 C of usable charge:

```
   t = 0.12 C / 0.25 µA ≈ 5.5 days
```

and that is pessimistic, since the part draws less at lower voltage.

**Why a Schottky specifically:** ~0.3 V forward drop instead of ~0.7 V. Every
tenth of a volt not thrown away is backup time gained, and the RTC's 1.8 V floor
makes the headroom matter.

**Why this matters at all:** every FRAM entry carries a real date and time instead
of "42 seconds after boot." When correlating a leak event against a dive log,
that is the difference between evidence and a shrug.

`CLKOUT` is deliberately left unconnected with an explicit no-connect flag. It
defaults to *enabled* at 32.768 kHz out of power-on reset, and on a board doing
microvolt-scale measurement a free-running 32 kHz square wave is exactly the kind
of thing that appears in an ADC trace as a mystery tone. It is open-drain with no
pull-up so it cannot actually drive, and firmware disables it at init — but the
intent belongs on the schematic.

`INT` is routed to GPIO8, using the RP2040's internal pull-up so no external
resistor is needed. It costs one pin and turns the RTC from a clock into a
scheduler.

---

## 10. PCB design

Four layers, 1.6 mm, Raspberry Pi HAT outline (65.0 × 56.5 mm), mounting holes on
the 58 × 49 mm pattern.

**Why the HAT outline:** the board is not a HAT — it connects by cable, not by
header — but matching the outline means it bolts to standoffs already present in
the BlueRobotics electronics tray.

### 10.1 Stackup

> **In plain terms.** The board is four layers of copper stacked with insulator between. The top and bottom carry the wiring; the two inner layers are solid sheets, one of ground and one of power. Four layers rather than two because the main chip has 56 pins on a very fine pitch, and because the sensitive analog part needs an uninterrupted sheet underneath it to work properly.

| Layer | Function | Copper |
|---|---|---|
| F.Cu | Signal, all components | 1 oz |
| *prepreg 0.2104 mm* | | |
| In1.Cu | **Solid ground** | ½ oz |
| *core 1.065 mm* | | |
| In2.Cu | Power zones | ½ oz |
| *prepreg 0.2104 mm* | | |
| B.Cu | Signal + ground pour | 1 oz |

This is a stock JLCPCB 4-layer stackup, chosen so it is free rather than a custom
build.

**Why four layers rather than two:** the RP2040 is QFN-56 at 0.4 mm pitch, which
is impractical to escape on two layers without via-in-pad; and the analog front
end needs a continuous, unbroken reference plane. Four layers is the minimum that
lets both be done properly.

**Why ground on In1 rather than power:** the tight 0.21 mm prepreg belongs between
the signal layer and its return, not between signal and a power zone. Return
current then sits directly beneath every trace on F.Cu.

### 10.2 Floorplan

> **In plain terms.** Where each part of the circuit physically sits. Signal flows left to right — outside world, then analog, then power as a buffer, then digital. The backstop gets its own strip across the top with one connector in and one out, so anyone looking at the board can see it's independent rather than having to take that claim on trust.

```
        0         14         28        40         52        65
      ┌─────────────────────────────────────────────────────────┐
 56.5 │  BACKSTOP STRIP                                         │
      │  J7 ──► osc ──► DC block ──► divider ──► peak ──► cmp ──► J6
 38.5 ├──────────────────┬───────────┬──────────────────────────┤
      │ WET  │ FRONT END │  MCU      │  POWER                   │
      │ EDGE │ mux, TIA, │  flash    │  P-FET, ADO              │
      │ J1-3 │ ranges,   │  crystal  ├──────────────────────────┤
      │ TVS  │ VMID      │  ferrite  │  I/O   SHT45 FRAM RTC J5 │
    0 └──────┴─╌╌╌╌──────┴───────────┴──────────────────────────┘
                                            SHT45 thermal island
```

Signal flows left to right: outside world → analog → power buffer → digital →
Navigator.

The **backstop occupies a strip across the top**, with its probe connector at one
end and its alarm output at the other. A reader can trace one wire in and one
wire out and see that nothing from the MCU touches it. The independence claim is
visible in the layout rather than argued in prose.

The **power block sits between the analog and digital regions** and functions as
a physical buffer.

### 10.3 The ground plane — and why it is not split

> **In plain terms.** The most counterintuitive thing on this board. The instinct is to cut the ground sheet in two so the noisy digital half can't contaminate the quiet analog half. That makes it worse. Current returning to its source doesn't head for some central point — it flows in the sheet directly underneath the wire it came along. Cut the sheet and you force it to detour, and the loop that creates radiates noise and picks it up. Separation here comes from distance and discipline, never from cutting copper.

In1.Cu is **one solid, unbroken ground plane** across the entire board, including
under the backstop.

This is deliberate and runs against a common instinct. Return current does not
"flow back to a star point"; above a few kHz it flows in the path of least
*impedance*, which is directly underneath the outgoing trace. Cutting the plane to
"isolate" the analog section forces the return to detour around the slot, and the
resulting loop area radiates and picks up. **Splitting the ground would make the
isolation worse, not better.**

Separation on this board is achieved by **physical distance and routing
discipline**: 20 mm between the summing node and the RP2040, the power block as a
buffer, and defined single crossing points for control signals entering the
analog region.


### 10.4 SHT45 thermal island

> **In plain terms.** The humidity sensor sits on a small peninsula of board joined by a narrow neck with no copper crossing it. Copper carries heat, and any warmth reaching a humidity sensor makes the air around it read drier than it really is.

A ~10 × 6 mm region isolated by two routed slots, joined to the board by a
2–3 mm neck. **No copper pour crosses the neck** — only the four minimum-width
traces the part needs.

**Why:** PCB copper is the dominant heat path to a humidity sensor. Every
milliwatt of self-heating that reaches it reads as artificially low relative
humidity. The island is placed at the bottom edge, the furthest point from both
the LDO and the RP2040.

Marked "NO CONFORMAL COATING" on silkscreen and on the fab drawing — coating over
a humidity sensor makes it a paperweight.

### 10.5 Trace widths

> **In plain terms.** How wide each wire is. Worth knowing these are not chosen for how much current they carry. They're chosen for how they behave electrically and for whether the factory can reliably make them.

| Net class | Width | Rationale |
|---|---|---|
| General digital | 0.20 mm | |
| I²C, QSPI | 0.20 mm | QSPI kept together, same layer, unbroken reference |
| High-Z analog | 0.25 mm | Width is electrically irrelevant here — length and neighbours matter |
| Power branch | 0.40 mm | |
| Power trunk | 0.60 mm | |

The whole board draws ~150 mA; a 0.4 mm trace on 1 oz copper carries ~1 A at a
10 °C rise. **None of these widths are chosen for current capacity.** They are
chosen for inductance and manufacturability.


## 11. Requirements this hardware places on firmware

These are consequences of hardware choices, and they belong with the hardware
documentation rather than being discovered later.

| Requirement | Because |
|---|---|
| **Run the calibration channel every sweep**, not once at boot | Mux on-resistance drifts several tenths of a percent per °C. The cal is what cancels it (5.5) |
| **Keep the 20 kHz tone below ~1.27 V peak** at the TIA output | The OPA2333 slews at 0.16 V/µs. Full-power bandwidth is 15.9 kHz at 1.6 V. Beyond it the waveform distorts toward a triangle and throws harmonics into the demodulator |
| **GPIO7 is input-only, permanently** | It taps the backstop's open-drain output. Driving it would fight the comparator (8.3) |
| **Disable PCF8563 CLKOUT at init** | It defaults to enabled at 32.768 kHz (9.4) |
| **Set `LEAK1_LOGIC` to match active-low** | The backstop asserts by pulling low (6.6) |
| **Respect the range/tone validity matrix** | Not every gain range works at every frequency; the TIA's bandwidth falls as Rf rises |

---

## 12. Verification: what was checked and how

### 12.1 The premise

Checked against the ArduSub source directly, and against the 2018 forum thread in
which a Blue Robotics engineer confirms the limitation. Both are primary sources.

### 12.2 Component pinouts

Every IC pinout was verified against the manufacturer's datasheet rather than a
symbol library or a secondary site. This caught a wrong PCF8563 pinout from one
secondary source and a package error (TMUX1104 is VSSOP-10, not TSSOP-8).

Where sources conflicted and could not be resolved — the SHT4x DFN-4 pin
numbering returned three different orderings from three sites — **the document
declines to state a number** and defers to the manufacturer datasheet and the
KiCad symbol.

### 12.3 The backstop

Threshold chain, hysteresis and trip resistance computed by hand and
cross-checked against an ngspice sweep using a Schottky model (Is=2e-8, N=1.05,
Rs=2). Behaviour at power-up was checked separately: C30 starts at 0 V, so the
peak is below threshold and the output releases. **No false alarm at boot.**

### 12.4 The frequency argument

Derived by simulation. The 112× improvement from the transimpedance topology
(965 Hz → 108 kHz) and the range/tone validity matrix both come from AC sweeps.

### 12.5 The netlist

Every schematic, BOM and diagram in this project derives from a **single Python
file** (`design.py`) that holds the parts and nets and asserts on:

- duplicate reference designators
- any pin appearing on two nets
- nets with fewer than two connections (against an explicit whitelist of
  intentional single-ended labels)
- parts appearing in no net

Current state: **90 parts, 82 nets, validation clean.**

This is a software habit applied to hardware, and it earned its keep. It caught a
real error immediately on merging the backstop sheet — `BACKSTOP_N` and `LEAK_N`
had been declared as separate nets when they are the same node — and it makes it
structurally impossible for a drawing to disagree with the netlist.

**What it cannot catch:** it validates *topology*, not *intent*. A supply pin
connected to `+3V3` when it should be `+5V` is perfectly valid to the checker and
was in fact a real error made during this project (section 14).

---

## 13. Open items

Nothing here has been built or bench-tested. The following are known problems,
most of them found by reviewing the design against itself.

| # | Item | Severity | Status |
|---|---|---|---|
| 1 | **Never fabricated or tested** | — | The largest caveat by far |
| 2 | **TIA compensation on high ranges** | High | One fixed 2.2 pF capacitor cannot serve four decades of Rf. At Cin ≈ 250 pF the 100 kΩ range needs ~34 pF and the 3 MΩ range ~6.2 pF; the low ranges need none, because their noise-gain zero sits above the op-amp's GBW. Fix is a capacitor per range resistor. **Not implemented.** May invalidate cells in the range/tone matrix |
| 3 | **Anti-alias corner vs. sample rate** | Medium | The 159 kHz corner was never dimensioned against a chosen ADC sample rate. Square-wave excitation carries harmonics falling as 1/n; at a 200 ksps sample rate the 9th harmonic of the 20 kHz tone (180 kHz, 11% amplitude) folds back to exactly 20 kHz. On high ranges the TIA's own rolloff covers this; on low ranges it does not |
| 4 | **A broken probe wire reads as "dry"** | High | Neither circuit distinguishes dry from disconnected. A leak detector that reports "all clear" because its sensor fell off is the failure a safety reviewer asks about first. Fix identified: a **10 MΩ across the electrodes at the probe end** makes "open" a distinct, detectable state — 10 MΩ is above the top measurement range and four orders above the backstop's divider, so it costs nothing. **Not implemented** |
| 5 | **I²C address collision** | Medium | 0x50 is the base address of every 24Cxx EEPROM and the single most commonly used I²C address in existence. Needs a bus scan against a real Navigator. The FRAM has eight strappable addresses if it collides. Also: the 4.7 kΩ pull-ups are in parallel with whatever the Navigator already has |
| 6 | **No TVS or fuse on the 5 V input** | Medium | The board shares a rail with thruster ESCs. One SMAJ5.0A and a polyfuse would be cents |
| 7 | **VMID filter corner** | Low | C6 = 1 µF against 5 kΩ gives a 31.8 Hz corner, *above* the 10 Hz tone. Because the ADC measures against ground rather than VMID, unfiltered rail noise arrives one-for-one as measurement error. Should be 10 µF (3.2 Hz) |
| 8 | **LDO current budget not computed** | Low | 51 °C junction rise at 150 mA, next to a humidity sensor. The real load is probably well under that, but nobody has added it up |
| 9 | **Electrode material and geometry** | High | Unspecified. These *set* Cdl and Rct, which the entire frequency analysis in section 3 rests on. Right now that analysis uses assumed values |
| 10 | **EMC from thruster ESCs** | High | Unanalysed. Brushless ESCs switching amps at tens of kHz, centimetres from a 3 MΩ transimpedance amplifier. The most likely reason a first prototype would misbehave |
| 11 | **Temperature correction unspecified** | Low | Water conductivity changes ~2%/°C. The SHT45 measures temperature and the correction is not defined. A free accuracy improvement sitting unclaimed |
| 12 | **Calibration and trim procedure** | Low | What is measured at test, what is stored in FRAM, how often it is re-run |

---

## 14. Decisions that changed, and why

Recorded because the reasoning is more instructive than the conclusions, and
because knowing which ground is firm matters.

| Decision | History | Settled at |
|---|---|---|
| Probe sharing | Backstop originally shared the front end's probes with all three sense lines commoned | **Separate electrodes.** Commoning destroyed channel identity; isolation resistors could not fix it without preventing the backstop from tripping (6.7) |
| Backstop connector | Briefly a 4-pin shared connector, argued on penetrator count | **Its own 2-pin connector.** The premise was wrong — the electronics-enclosure probe is *inside* the enclosure with the board, so there is no penetrator to save |
| Backstop supply | Silently changed to 3.3 V during a netlist merge | **5 V**, as originally designed. The 3.3 V error broke the threshold arithmetic downstream and produced two further phantom "fixes" before it was caught |
| R24 divider | 3.3k → 1.5k → 3.3k | **3.3 kΩ**, the simulated value. The 1.5k excursion was chasing the supply error above |
| Threshold divider lower leg | A 62 kΩ resistor was added, then removed | **Not present.** It was only "needed" under the incorrect 3.3 V assumption |
| Second calibration channel on Y3 | Proposed, then withdrawn | **Y3 stays open**, as originally specified. The proposal contradicted the spec with no new information behind it |
| Connector ownership | All connectors on one sheet | **Each sheet owns its own.** Co-locating only made sense while the probes were shared |
| Backstop internal labels | Global | **Local.** Global labels are what allowed the two circuits to silently merge in the first place; local labels make that class of error structurally impossible |
| BACKSTOP_N | Declared as a separate net from LEAK_N | **One net.** They are the same node. The validator caught this on the first merge |
| Backstop placement | Probe connector at the strip's end | **Probe connector in the middle.** Placing it mid-strip lets the drive chain and sense chain each stay short instead of doubling back across the oscillator |
| QSPI SD0/SD1 | Crossed to the flash's DI/DO | **SD0→IO0, SD1→IO1.** Crossed, the board does not boot |

Several of these were corrections of errors rather than responses to new
information. The pattern is consistent: every one occurred where a value was
recalled from memory instead of read from `design.py` or a datasheet.

---

## 15. Net list

82 nets. Cross-sheet nets are joined by name.

### Front end

```
+3V3          U1.VDD U2.V+ U3.VDD C2.1 C3.1 C4.1 R12.1 R28.2
EXC_DRIVE     R4.1                                   (label in, from MCU)
DRIVE         R4.2 C1.1
DRIVE_AC      C1.2 J1.1 J2.1 J3.1 R5.1 R6.1 R7.1 D5.1
PROBE1_RTN    J1.2 D1.1 R1.1
PROBE2_RTN    J2.2 D2.1 R2.1
PROBE3_RTN    J3.2 D3.1 R3.1
MUX_Y0        R1.2 U1.Y0                             probe 1
MUX_Y1        R2.2 U1.Y1                             probe 2
MUX_Y2        R3.2 U1.Y2                             probe 3
MUX_Y3        U1.Y3                                  deliberately open
MUX_Y4        R5.2 U1.Y4                             short cal
MUX_Y5        R6.2 U1.Y5                             1M reference
MUX_Y6        R7.2 U1.Y6                             1k reference
MUX_Y7        U1.Y7                                  spare, open
MUX_A/B/C     U1.A / U1.B / U1.C                     labels in
SUM           U1.Z U2.INA- R8.1 R9.1 R10.1 R11.1 C5.1
RNG1..RNG4    R8.2 U3.S1 / R9.2 U3.S2 / R10.2 U3.S3 / R11.2 U3.S4
TIA_OUT       U2.OUTA U3.COM C5.2 R14.1
VMID_RAW      R12.2 R13.1 C6.1 U2.INB+
VMID          U2.OUTB U2.INB- U2.INA+
RANGE_0/1     U3.SEL0 / U3.SEL1                      labels in
ADC_IN        R14.2 C7.1                             label out
```

### Backstop

```
BS_RC         U10.A R21.2 C27.1
BS_OSC        U10.Y R21.1 R22.1
BS_DRIVE_DC   R22.2 C29.1
BS_DRIVE      C29.2 J7.1 D6.1
BS_SENSE      J7.2 D7.1 R23.1
BS_DIV        R23.2 R24.1 D8.1
BS_PK         D8.2 C30.1 R25.1 U11.IN-
BS_VREF       U11.REF R26.1
BS_TH         U11.IN+ R26.2 R27.1
LEAK_N        U11.OUT R27.2 R28.1 J6.1 U5.GPIO7
```

### Power

```
+5V_IN        Q1.S                                   label in, from I/O sheet
+5V           Q1.D C8.1 U4.IN C9.1 U10.VCC U11.V+ C28.1 C31.1
Q1_GATE       Q1.G R15.1
+3V3_P        U4.OUT C10.1 FB1.1                     label out
+3V3_ADC      FB1.2 C11.1 C12.1
```

### MCU

```
+3V3_M        U5.IOVDD U5.USB_VDD U5.VREG_IN U6.VCC R17.1 C17.1 C19.1 C21.1 C22.1
+3V3_ADC_M    U5.ADC_AVDD C20.1
VREG_1V1      U5.VREG_VOUT U5.DVDD C16.1 C18.1
XIN           U5.XIN C13.1 Y1.1
XOUT          U5.XOUT R16.1
XOUT_R        R16.2 C14.1 Y1.2
RUN           U5.RUN R17.2 C15.1
QSPI_SS       U5.QSPI_SS U6.CS R18.2
QSPI_SCLK     U5.QSPI_SCLK U6.CLK
QSPI_SD0      U5.QSPI_SD0 U6.DI                      SD0 -> IO0
QSPI_SD1      U5.QSPI_SD1 U6.DO                      SD1 -> IO1
QSPI_SD2      U5.QSPI_SD2 U6.WP
QSPI_SD3      U5.QSPI_SD3 U6.HOLD
BOOTSEL       R18.1 SW1.1
SWCLK/SWDIO   U5.SWCLK J4.1 / U5.SWD J4.2
```

### I/O

```
+5V_IO        J5.1                                   label out, to power sheet
+3V3_IO       R19.1 R20.1 U7.VDD U8.VDD C23.1 C24.1 D4.1
I2C_SCL       J5.2 R20.2 U7.SCL U8.SCL U9.SCL
I2C_SDA       J5.3 R19.2 U7.SDA U8.SDA U9.SDA
VBAT          D4.2 C26.1 U9.VDD C25.1                backup domain
RTC_OSCI/O    U9.OSCI Y2.1 / U9.OSCO Y2.2
RTC_INT_N     U9.INT U5.GPIO8
RTC_CLKOUT    U9.CLKOUT                              deliberate no-connect
GND_IO        J5.4 U7.VSS U8.VSS U9.VSS C23.2 C24.2 C25.2 C26.2
              U8.A0 U8.A1 U8.A2 U8.WP
```

---

## 16. References

**Primary**

- ArduSub / ArduPilot, `libraries/AP_LeakDetector/AP_LeakDetector.cpp`
- Blue Robotics Community Forums, *"Leak detection using multiple sensors and
  separate leak reporting"*, November 2018
- Blue Robotics Community Forums, *"Leak detection in multiple enclosure"*

---

*This is a design study. It has not been built. Section 13 is not an appendix.*
