# Pressure-distribution haptics

`pressure_distribution_haptics.ino` is based on the supplied M5Unified / ESP32
TDM sketch. It keeps the original pin assignment, tactile-sensor API, 44.1 kHz
audio rate, eight TDM slots, and maximum signal amplitude.

## What changed

The old sketch selected the largest taxel in each 3 x 3 quadrant. This causes a
hard boundary when contact crosses between quadrants and ignores most of the
6 x 6 pressure distribution.

The new sketch gives every taxel a bilinear weight for all four actuator
corners. A contact therefore moves continuously between actuators. It combines:

- spatially weighted mean pressure for smooth localization;
- quadrant peak pressure for a crisp fingertip response;
- low-pass pressure for stable sustained feedback;
- pressure change over 100 ms for impact and deformation transients;
- weaker, opposite-polarity release transients;
- center-of-pressure movement mapped to opposing left/right and upper/lower
  actuator pairs;
- a delayed asymmetric waveform for material presets.

## First test

1. Flash the sketch without the device touching the body.
2. Open Serial Monitor at 115200 baud.
3. Press each physical position separately and verify that `P` responds in the
   order `[fingertip-left, fingertip-right, joint-left, joint-right]`.
4. If the displayed corner is reversed, change `FLIP_SENSOR_X`,
   `FLIP_SENSOR_Y`, or `SWAP_SENSOR_XY`.
5. If the correct pressure value is shown but the wrong motor vibrates, edit
   `SLOT_FOR_CORNER`.
6. Start at the existing electrical level. Do not increase
   `MAX_OUTPUT_AMPLITUDE` until actuator temperature and amplifier clipping have
   been checked.

## Material commands

Send one character through Serial Monitor:

- `c`: direct contact, 120 Hz, no delay, medium response;
- `w`: water, a pressure-driven virtual liquid mass that emits a 160 Hz burst
  only when it reaches a wall, followed by a 150 ms reverse rebound and a
  300 ms ripple; movement between walls produces a weak velocity-dependent
  flow cue around 110 Hz, which stops after the virtual liquid settles. Fast
  alternating motion adds 80 ms resonating impact bursts on all four
  actuators for a jaba-jaba feel;
- `y`: yogurt, a highly damped virtual mass with a 100 ms output delay,
  sustained low-frequency flow resistance, a softer wall impact, and longer
  rebound timing than water;
- `r`: rubber, 40 Hz, 200 ms delay, slow response with weak direction;
- `d`: toggle directional pseudo-shift for an immediate A/B comparison.
- `s`: toggle NORMAL / LIGHT-GRIP sensitivity. The sketch starts in
  LIGHT-GRIP mode.
- `i`: cycle through 24 IMU axis/sign mappings and recalibrate the current pose
  as neutral. The default for the present mounting is `XY--`: thumb to index
  and fingertip to joint both read as positive after the sign correction.

## Direction display

The serial output includes `C=[x y]`, the normalized center of pressure, and
`d=[dx dy]`, its movement over approximately 100 ms. Positive `dx` means the
contact moved right; positive `dy` means it moved down. When directional mode
is on, opposing actuator sides receive opposite asymmetric-wave polarity.
`T=` is the raw total pressure across all 36 taxels and is useful for deciding
whether weak response comes from the sensor/mechanics or the software mapping.
`e=[ex ey]` is the fast fingertip-deformation signal relative to the slowly
adapting grip baseline. This signal is weighted more heavily than CoP movement
so that a lightly held device can respond to small side-wall deformation.
In WATER and YOGURT modes, `q=[qx qy]` is the wall-impact pulse sent to the
directional mapper. It should stay near zero while held still and spike briefly
when the virtual material hits or rebounds from a wall. `f=[fx fy]` is the
weaker flow cue derived from virtual-material velocity. Faster tilting makes
`f` larger; holding the device still lets it decay back to zero. Yogurt keeps
this flow cue longer and reaches the wall more slowly than water.
When an internal M5 IMU is available, `g=[gx gy]` is tilt relative to the pose
at first light contact. `k=[kx ky]` is fast acceleration with gravity removed,
and `j=[jx jy]` is the resulting splash/jaba-jaba pulse. `u=[ux uy]` is the
fused input used by the virtual fluid: IMU tilt supplies 72%, pressure motion
and deformation supply 70%, and the sum is clamped to full scale. If no IMU
is detected, `u` and the splash detector automatically fall back to pressure.
The tilt scale is deliberately sensitive: about 0.12 g from the neutral pose
reaches full `g`. The `k` signal uses a separate, slower moving baseline so
that repeated back-and-forth shaking continues to produce pulses. It monitors
all three accelerometer axes. Motion along the axis not used by `g` becomes an
opposing diagonal splash, so shaking in depth also drives the four actuators.
`b=[b0 b1 b2 b3]` shows the strength of the current 80 ms impact burst. A fast
shake should produce four similar non-zero values; a slower side-wall impact
should make the actuators on the corresponding side stronger.
Because four simultaneous actuators produce a much stronger whole-device
sensation than one actuator, the final impact-burst output is scaled to 52%
for both water and yogurt. This scaling does not reduce the slower
tilt/flow cue.

## Research-informed impact waveform

The shake rendering follows three findings from prior work:

- Vibr-eau measured physical liquid/container impacts at an average duration
  of 90.75 ms and selected 80 ms actuator pulses. Its two-microphone recordings
  found similar amplitudes on both sides during shaking, while swaying produced
  a stronger signal on the struck side. This sketch therefore uses uniform
  four-actuator bursts for fast shaking and spatially weighted bursts for wall
  impacts: https://arxiv.org/abs/2501.18755
- Cirio et al. render splashing-fluid impacts as short noise excitations passed
  through a resonator with an attack/decay envelope, and relate impact amplitude
  to the cube of impact speed. This sketch uses a resonating-noise/cavity-tone
  mixture, a 6 ms attack, an exponential decay within 80 ms, and a cubic
  intensity curve above the trigger threshold:
  https://doi.org/10.1109/TOH.2012.34
- ShakeSense reports that cyclic shaking becomes distinctly slosh-like at more
  vigorous motion around and above 2 Hz, and that synchronized multidirectional
  fingertip stimulation is more effective than a center-of-mass-only cue:
  https://doi.org/10.1145/3772318.3791974

## Index-finger mounting map

The sensor coordinate map remains:

- fingertip-left = sensor upper-right;
- fingertip-right = sensor lower-right;
- joint-left = sensor upper-left;
- joint-right = sensor lower-left.

The pressure signals are routed to the physical actuator grid as follows:

- fingertip-left -> thumb upper actuator = SLOT2;
- fingertip-right -> index-finger upper actuator = SLOT0;
- joint-left -> thumb lower actuator = SLOT3;
- joint-right -> index-finger lower actuator = SLOT1.

In the serial `C`, `d`, and `e` coordinates, positive X points from the left
side of the finger to the right side. Positive Y points from the fingertip
toward the joint.
If the perceived pull is consistently opposite to the finger movement, change
`DIRECTION_SIGN` from `1.0f` to `-1.0f`.

The water, yogurt, and rubber frequency/delay pairs come from Figure 1 of the
CHI 2026 paper. The pressure gains and steady/change mixtures are new starting
values for this device and require user testing; they are not values validated
by that paper.

## Important interpretation

The paper senses motion with an accelerometer and drives paired actuators with
asymmetric vibration to create a pseudo-attraction force. This sketch now uses
an optional hybrid input: IMU tilt provides gravity direction while pressure
detects contact and fingertip deformation. On hardware without an IMU it falls
back to the pressure-driven adaptation.

The current four-actuator mounting may also produce a spatial vibration cue
more strongly than a directional pseudo-force. Confirm the effect through user
testing and compare it with the original sine-wave/quadrant implementation.

## Sensitivity modes

NORMAL uses a sensor threshold of 20 and conservative pressure/motion gates.
LIGHT-GRIP uses a threshold of 8, accepts lower total pressure, and amplifies
smaller center-of-pressure motion without treating every tiny fluctuation as
a water collision. Both modes keep the electrical
maximum at 10000. If LIGHT-GRIP vibrates without contact, send `s` to use
NORMAL and record the no-contact `P` values before further tuning.

