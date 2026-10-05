# Physics — physical sensors

**Owner:** Physics. **Normative.** A physical sensor is a responder that reads state and fields instead of producing a force. Physics owns the sensor models and their kernels. Core owns the plumbing:
- the sensor arena and kind tags;
- output rings and the stateless `poll(sensor, since_index)` (`engine A4`);
- where synthesis sits in the schedule (`engine A1`).

Camera sensors are Rendering's.

## Rules for every sensor

- **Synthesis runs inside the step**, in the sensor phase after Integrate, on that sensor's own rate boundary. It is never shed under load.
- **Noise comes from per-sensor seeded streams** (`rng::Stream`, domain-tagged). Gaussian draws are CPU↔GPU banded at the source (`CORE-3`, Core), so a sensor's GPU band cites that, not a cause of its own.
- **Output is tick-stamped samples into the sensor's ring.** How a consumer stamps or fuses them is the consumer's business.
- **The noise model is part of the row**, and a row change moves that family's digests only. That is why the arena stays split per family (`../superseded/2026-09-consolidation/sensor-arena-dedup.md`).

## IMU

- **Gyro:** the body rate rotated into the mount frame, plus bias plus `σ_g·n`.
- **Accelerometer:** the specific force captured in Integrate, plus the lever-arm term `ω × (ω × r)`, rotated into the mount frame, plus bias plus `σ_a·n`.
- **Bias:** a random walk per axis, one step per sample.
- **Densities:** the noise densities are specified per √Hz and scaled by the sample rate.

All of it is in `sensors/imu.hpp`.

**Known limit:** a body resting on the ground reads about 0 specific force, because contact impulses bypass `force_acc`. In-flight readings are correct. The fix belongs with the contact model's next fidelity step (`02-responders.md`).

## GNSS

- **The fix:** a local-tangent-plane position and velocity in the world frame, at a low rate. There is no geodetic datum; a consumer that needs latitude and longitude owns the origin.
- **Error:** a first-order Gauss-Markov bias with correlation time `bias_tau_s`, plus white noise (`σ_h`, `σ_v`). Velocity is an order of magnitude more accurate than position.

All of it is in `sensors/gnss.hpp`. GNSS runs on both backends. Its GPU draws differ from the CPU's within `CORE-3`'s band, which `GnssDrawsMatchTheCpuWithinTheCore3Band` checks over every fix.

## Next sensors

Barometer, magnetometer and rangefinder fit the same shape: a row, a kind tag, a seeded stream, a ring. None is designed yet.
