# Adaptive 9D Boid Sound Engine — Technical Specification

## 01. Overview

The Adaptive 9D Boid Sound Engine is a swarm-driven modal synthesis system in which a population of agents (boids) is represented by a 9-dimensional feature vector. The system does not simulate physical forces; instead, it constructs a structured feature space that is continuously mapped into:

- 3D spatial field (for sound placement)
- density field (for excitation and spectral control)
- temporal excitation field (trigger system)
- modal synthesis layer (resonant sound generation)

Sound is generated through independent resonant filters (modal bank) rather than physical coupling systems. Interaction between agents emerges implicitly through shared statistical fields (distance, density, and global normalization).

---

## 02. 9D State Representation

Each boid consists of:

- position[0..8] (9D feature vector)
- velocity[0..8] (9D feature vector)

The 9D space is partitioned into three independent 3D subspaces:

``` text 
| Subspace | Dimensions | Interpretation                      |
| -------- | ---------- | ----------------------------------- |
| S1       | 0–2        | Primary spatial feature vector      |
| S2       | 3–5        | Secondary structural feature vector |
| S3       | 6–8        | Tertiary structural feature vector  |
```
These subspaces are not orthogonalized and are not treated as geometric basis vectors. They are combined linearly.

---

## 03. Spatial Aggregation Model

**3.1 Subspace Fusion**

Each boid’s spatial position is computed as a weighted sum of its three subspaces:

```text
spatialPos[i] =
    w1 * pos[0..2] +
    w2 * pos[3..5] +
    w3 * pos[6..8]
```

Weights are normalized per system configuration.

**3.2 Temporal Smoothing**

Spatial positions are low-pass filtered:

```text
spatialPos(t) = α * spatialPos(t-1) + (1 - α) * fusedPosition
```

This stabilizes the spatial field and reduces jitter in synthesis parameters.

---

 ### 04. Torus Wrapping Model

All spatial computations operate in a periodic domain:

- positions are wrapped into a toroidal space of size 2 units
- differences are wrapped to ensure shortest-path consistency

```text
p -= round(p / 2) * 2;
```

This applies to:

- position
- velocity reconstruction
- inter-boid distances

---

### 05. Density Model

Each boid’s density is computed from its average wrapped distance to all other boids:

```text
meanDist[i] = average distance to all j ≠ i
density[i] = exp(-meanDist[i] * DENSITY_EXP_SCALE)
```

Properties:

- density is local (per boid)
- no explicit global density field exists
- a separate global mean distance is computed for normalization

Interpretation:

- dense clustering → high density values
- sparse distribution → low density values

Density is used as a control modulation factor, not a spatial field.

---

### 06. Velocity Model

Velocity is reconstructed using torus-consistent differences:

```text
velocity[i] = wrap(spatialPos[i] - previousSpatialPos[i])
```

Velocity is used as a secondary excitation descriptor, producing:

- spectral instability
- amplitude variation
- mode-level modulation

Velocity does not define motion trajectories in a physical sense; it modulates spectral texture.

---

### 07. Trigger System (Event Excitation)

Each boid has an independent deterministic accumulator:

```text
acc[i] += triggerRate[i] / sampleRate
if acc[i] ≥ 1:
    excite modal bank
    acc[i] -= 1
```

**Trigger rate computation:**

Trigger rate depends on relative spatial ratios:

```text
adaptive  = meanDist[i] / globalMeanDist
absolute  = meanDist[i] / ABSOLUTE_DISTANCE_SCALE

relative  = mix * adaptive + (1 - mix) * absolute
compression = clamp(1 - relative, 0, 1)

triggerRate = minRate + (maxRate - minRate) * compression^exponent
```

**Key property:**

Triggering is driven by relative structure, not absolute density.

---

### 08 Modal Synthesis System

Each boid controls an independent modal bank:

- N modes per boid (default: 8)
- each mode is a resonant filter

**Mode parameterization:**

For mode index m:

```text
freq[m] = baseFreq * (1 + spread * m + jitter)
bw[m]   = baseBW * (1 + 0.3 * m)
amp[m]  = baseAmp / (1 + decay * m)
```

where:

- baseFreq is derived from spatial position + density
- jitter is a nonlinear term driven by velocity and mode index

```text
jitter = 0.02 * (1 + energy) * sin(m)
```

**Interpretation:**

- higher modes → broader, weaker spectral components
- velocity increases spectral instability
- density increases spectral brightness

### 09. Modal Excitation Model

Each trigger event injects energy into all modes of a boid:

```text
excitation[i][m] += triggerEnergy
```

Energy is reset after each audio frame.

Modal output is summed per boid:

```text
boidOutput[i] = Σ modeOutput[i][m] * weight[m]
```

---

### 10. Spatial Audio Encoding

**Encoding:**

```text
azimuth = atan2(x, z)
distance = clamp(|position|)

gain = 1 / (1 + k * distance)
```

The signal is encoded into a 3rd-order ambisonic field (7 channels):

- W (omnidirectional)
- X/Y/Z (first order)
- higher-order components (2nd + 3rd order)

---

### 11. Ambisonic Decoding

The ambisonic field is decoded into stereo:

```text
stereo = decode(ambiField, speakerAngle)
```

Two virtual speaker positions are used for left/right output.

---

### 12. Output Stage

Final output is saturated using soft clipping:

```text
output = tanh(signal * drive)
```

This prevents modal summation from exceeding stable amplitude bounds.

---

### 13. System Architecture

```text
[OSC Input]
     ↓
[Boid State: 9D vectors]
     ↓
[Subspace Fusion (S1 + S2 + S3)]
     ↓
[Spatial Field (3D smoothed torus space)]
     ↓
 ┌──────────────────────────────┐
 │ Density Computation          │
 │ Velocity Reconstruction      │
 │ Distance Ratios             │
 └──────────────────────────────┘
     ↓
[Trigger System (event-driven excitation)]
     ↓
[Modal Bank (per boid, multi-mode resonators)]
     ↓
[Audio Summation per Boid]
     ↓
[Ambisonic Encoding (3rd order)]
     ↓
[Ambisonic Decoding]
     ↓
[Stereo Output + tanh saturation]
```
 
---

### 14. System Characteristics

**Emergent behavior sources:**

- ratio-based trigger system (relative geometry)
- nonlinear modal jitter term
- density-dependent excitation scaling
- torus-wrapped spatial continuity
- multi-mode spectral decay hierarchy 

**Not present in implementation:**

- no physical force simulation
- no orthogonal 9D basis construction
- no explicit coupling matrix between boids
- no true geometric decomposition into r/t1/t2 vectors

---

### 15. Conceptual Summary

The system is best described as:

A torus-bound, feature-aggregated swarm system where 9D agent states are projected into a smoothed 3D spatial field and used to drive a distributed modal synthesis network. Sound emerges from statistically coupled excitation rather than physical interaction.
