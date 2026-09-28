# The second sensor path — what it costs, and why the amendment I wrote is wrong

> **Design only. No code.** Written as the ruling input the Overseer asked for before approving a
> refactor of a dispatch path both sensors share. Same discipline as `interleaving-shuffle.md`.
>
> ⛔ **This note ARGUES AGAINST ITS OWN REALM'S EARLIER AMENDMENT.** `a0f81cde` proposed kind-tagging
> the sim layer's sensor arena so `poll()` dispatches on a tag and GNSS becomes *"the first ROW
> rather than the second SPECIAL CASE."* Measured against the tree, that fix buys a small
> deduplication and pays for it in the two places the arena is not a CPU container.

---

## The debt is real. `7b5a86d5` added the second hand-typed sensor path.

Six Simulation-level function pairs are now structurally identical and differ only in which typed
array they index:

```
add_imu_sensor        / add_gnss_sensor
poll_imu              / poll_gnss
validate_imu_ref      / validate_gnss_ref
imu_sensor            / gnss_sensor
live_imu_sensor_count / live_gnss_sensor_count
free_imu_sensors_of   / free_gnss_sensors_of   (+ clear_imu_ring / clear_gnss_ring)
```

That is exactly the welding `a0f81cde` predicted, and it will re-weld once more the next time a
sensor lands. **The debt is not in dispute. What the amendment gets wrong is where the arena lives.**

---

## THE ARENA IS THREE THINGS, AND THE AMENDMENT WAS WRITTEN ABOUT ONE OF THEM

```
1  A CPU CONTAINER    a span of rows the sim layer allocates, frees and reads   <- the audit saw this
2  A GPU BINDING      RWStructuredBuffer<ImuSensorRow> at binding 5,
                      RWStructuredBuffer<GnssSensorRow> at binding 23           <- TYPED, per family
3  A DIGEST SOURCE    state_digest folds elem_size AND the raw bytes of every
                      registered array, in registration order                   <- BYTE-EXACT
```

Measured:

```
sizeof(ImuSensorRow)  = 128        sizeof(ImuSample) = 48
sizeof(GnssSensorRow) = 112        sizeof(GnssFix)   = 48
IMU holds bindings   5, 6, 11      GNSS holds 23, 24, 25
```

### (2) A single arena forces a UNION STRIDE onto the GPU

One kind-tagged arena is **one row type**, so the GPU binds one `RWStructuredBuffer<SensorRow>`
where `SensorRow` covers both layouts. The stride becomes `max(128, 112) = 128`, every receiver
row carries **16 bytes of padding it does not use**, and `layouts.slang`'s mirror — which today
describes a concrete struct under `@cpp-type`/`@cpp-header` — has to describe a **variant**. Both
kernels then index the same struct and must agree on offsets that mean different things depending
on a tag they each have to read first.

### (3) And it COUPLES TWO SENSORS' DETERMINISM, which is the disqualifying cost

`state_digest` folds each registered array's `elem_size` and its raw bytes.

> ### ⛔⛔ WITH ONE ARENA, A CHANGE TO THE IMU ROW MOVES EVERY GNSS DIGEST, AND A CHANGE TO THE GNSS ROW MOVES EVERY IMU DIGEST. TWO SENSORS THAT SHARE NOTHING PHYSICALLY WOULD SHARE A CORPUS REGENERATION.

This leg already measured what a row-size change costs: `acec7f7f` moved `GnssSensorRow` 96 → 112
and regenerated five corpus YAMLs plus a replica mirror. **Under a union arena that same edit would
also have regenerated every IMU digest in the tree** — and the reverse, forever after. *The
separateness of the arenas is not an accident of how they were built; it is what keeps one sensor's
layout churn out of the other's reproducibility record.*

---

## THE DEDUPLICATION IS AVAILABLE WITHOUT ANY OF THAT, AND THE MACHINERY ALREADY EXISTS

The six pairs differ only in `(RowType, SampleType, ArrayId<RowType>, ArrayId<SampleType>)`. **That
is a template parameter list, not a tag.** And the layer below is *already* generic:
`sensors::ring_poll<T>` and `sensors::PollResult<T>` are templates today, used by both families —
`poll_imu` and `poll_gnss` are thin wrappers over the same instantiated function.

```
THE AMENDMENT       one arena, rows tagged, poll() dispatches on the tag at RUNTIME
THIS NOTE           two arenas, unchanged, and ONE TEMPLATE the six pairs instantiate
```

> ### ⭐ A RUNTIME TAG IS THE RIGHT ANSWER WHEN THE SET OF KINDS IS OPEN AND THE STORAGE IS SHARED. HERE THE STORAGE IS **DELIBERATELY NOT SHARED**, AND THE KINDS ARE A CLOSED, COMPILE-TIME SET — WHICH IS THE SHAPE A TEMPLATE FITS AND A TAG DOES NOT.

**What it costs:** one new internal template per operation, six call sites reduced to six
one-line instantiations, **zero change to any registered array, any binding, any row layout or any
digest.** It is a pure CPU-side refactor with a byte-identical corpus, which means it can be proved
the way this realm proved the re-cut: **the tree hash of every registered array is unchanged, so no
digest can have moved.**

**What it makes possible:** the third sensor adds one instantiation instead of six functions — and
more importantly, **a fix to the shared logic reaches every sensor at once.** The duplication's real
danger is not the line count; it is that `free_gnss_sensors_of` and `free_imu_sensors_of` carry the
same subtle liveness-from-the-map argument **twice**, so a defect found in one is a defect that has
to be *remembered* in the other.

---

## WHAT BREAKS IF IT IS DONE WRONG

- **If the template is written over the ROW but not the RING**, `clear_*_ring` stays duplicated and
  the field-wise zeroing argument — *"naming all six fields zeroes every byte, which is what makes a
  freed sensor's ring read as zeroes in a snapshot"* — is the half most likely to drift. **Both
  halves or neither.**
- **If `GnssSensorRef` and `ImuSensorRef` are unified into one templated `SensorRef<Row>`**, check
  that they remain **mutually unassignable**. Their whole purpose is that a valid IMU ref must not
  compile against `poll_gnss`; a template parameterised on the row type preserves that, a template
  parameterised on nothing does not. ⛔ **This is the one change that could silently REMOVE a
  guarantee while looking like a tidy-up.**
- **If the refactor is allowed to touch a row layout "while we are in here"**, it stops being
  digest-neutral and needs a corpus pre-registration. **Keep it byte-neutral or it is a different
  leg.**
- **If it lands before the kind-tagged-arena question is formally retired**, the next reader finds
  two amendments in this realm's record pointing opposite ways. **Retiring `a0f81cde`'s amendment is
  part of the work, not a footnote to it** — a finding travels with its fix, and so does its
  withdrawal.

---

## Recommendation

**Retire the kind-tagged arena amendment. Do the template.** It captures the whole of the
duplication the audit was right to flag, at no cost to the GPU binding table, the row layouts or
the digests — and it keeps the property the audit did not weigh, which is that **two sensors that
share no hardware should not share a corpus regeneration.**

⚠ **Not started, and not to be started on this note alone.** It touches the poll path for both
sensors, which is why the Overseer asked for the ruling input rather than a one-line description.
