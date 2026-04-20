# Adaptive 9D Boid Sound Engine – Detailed Documentation

The sound synthesis is based on a modal synthesis system driven by a nine-dimensional swarm simulation.
Extending the system to nine dimensions enables a functional decomposition of the state space and allows controlled mapping of spectral, dynamic, spatial, and event-based properties of the sound field.

Instead of classical mass–spring–damper systems, the engine uses resonant, band-limited modal filters.
Each mode consists of multiple parameterized resonators whose frequency, bandwidth, and amplitude are continuously updated.

No physical coupling matrices are computed. Interaction emerges indirectly through shared excitation and parameter-driven modulation, which avoids numerical instability while producing a stable and complex resonant field.

Each mode is assigned to a boid and derived from its nine-dimensional state vector.
The nine dimensions are grouped into three functional subspaces.

---

## 1. Subspaces & Dimensional Structure

``` text 
| Subspace / Dimensions | Interpretation in System          | Sonic Role / Control Function                                                                                |
| --------------------- | --------------------------------- | ------------------------------------------------------------------------------------------------------------ |
| Subspace 1 (dims 0–2) | Primary spatial projection vector | Defines spatial direction used for frequency mapping (via r.y), azimuth, and distance-based spatialization   |
| Subspace 2 (dims 3–5) | Secondary latent vector           | Contributes to spatial position mixing via orthogonal projection; affects aggregated spatial field structure |
| Subspace 3 (dims 6–8) | Tertiary latent vector            | Contributes to additional spatial variation through weighted subspace blending                               |
```

**Local Neighborhood / Density**  
- Density is computed from average distances in aggregated spatial space
- Implemented as an exponential decay of mean distance
- Acts as a global modulation factor for: frequency scaling, trigger rate, spectral density

**Trigger / Event System**
- Event-based excitation derived from distance relationships
- Deterministic accumulator-based triggering mechanism
- Trigger rate depends on local vs global spatial distribution

Key behavior:
- dense regions → higher excitation rate
- sparse regions → lower excitation rate

**Velocity Influence:** 
Velocity is stored but only weakly contributes to the system.
The primary control structure is spatial (position-based), not dynamic (velocity-based).

**Subspace Freezing:** 
Subspaces can be conceptually stabilized (in analysis or parameter control) to reduce complexity in the resulting sound field.

---

## 2. Spatial Interpretation Model

Each boid is interpreted as a structured spatial feature rather than a physical trajectory.

### 2.1 Radial Vector r (dims 0–2)

```text
glm::vec3 r(b.dims[0], b.dims[1], b.dims[2]);

if (glm::length(r) > 1e-6)
{
    r = glm::normalize(r);
}
```
- Only direction is relevant
- r.y is used for frequency mapping
- Defines primary spatial orientation for the sound field

 ### 2.2 Tangential Vector t1 (dims 3–5)

```text
glm::vec3 t1(b.dims[3], b.dims[4], b.dims[5]);

t1 -= glm::dot(t1, r) * r;

double flow = glm::length(t1);

if (flow > 1e-6)
{
    t1 = glm::normalize(t1);
}
else
{
    t1 = glm::vec3(0.0f);
}
```
- Represents secondary spatial variation
- flow = magnitude of orthogonal component
- Influences spectral width and amplitude modulation

### 2.3 Normal Vector t2 (dims 6–8)

```text
glm::vec3 t2 = glm::cross(r, t1);

double curvature = glm::length(t2);

if (curvature > 1e-6)
{
    t2 = glm::normalize(t2);
}
```
- Derived orthogonal vector between r and t1
- curvature is a measure of spatial deviation
- influences detuning and spectral dispersion

### 2.4 Density Field

Density is computed over the full aggregated spatial system.
It acts as a global emergent parameter controlling:
- trigger probability
- frequency lift
- amplitude scaling
- spectral bandwidth expansion

---

## 3. Spectral Parameter Generation (Per Boid)

### Frequency Mapping

```text
radialHeight → radialCurve → densityLift → targetFreq → smoothedFreq
```

### Amplitude Mapping

```text
density + flow → energetic mix → ampBase
```
### Bandwidth Mapping

```text
density * flow → tonal/noisy interpolation → baseBandwidth
```
### Detuning Mapping

```text
density + curvature → instability factor → detuneAmt
```
### Modal Parameter Assignment

```text
for (m)
{
    frq = baseFreq * (1 + detuneAmt * m);
    bw  = baseBW   * (1 + 0.3 * m);
    amp = baseAmp  / (1 + 0.4 * m);
}
```
- higher modes → more diffuse spectral structure
- detuning increases with mode index
- amplitude decreases per mode

---

## 4. Trigger & Envelope System

- Event-based excitation derived from spatial density and distance structure
- Deterministic accumulator-based trigger mechanism
- Excitation drives modal resonance bank directly
- Envelope behavior depends on density-driven energy distribution 

---

## 5. Spatial Encoding & Stereo Output

- Boid position mapped to azimuth and distance
- Distance controls attenuation via exponential decay: `exp(-3 * dist)`  
- Each boid is encoded into a 7-channel circular harmonic field
- Fields are summed into a global spatial buffer
- Decoded into stereo output (L/R)
- Final output uses soft saturation: `tanh()`  

---

## 6. Signalfluss-Diagramm (ASCII)

```text
[Swarm System]
      │
      │ (9D Boid State)
      ▼
[Boid Aggregator]
      │
      ├── Subspace 1 → radial r → frequency + spatial direction
      ├── Subspace 2 → t1 → flow → amplitude + bandwidth
      └── Subspace 3 → t2 → curvature → detuning
      ▼
[Density Field]
      ▼
[ModalBank2D]
      ├── Multi-mode resonators per boid
      ├── frequency / bandwidth / amplitude control
      └── event-based excitation
      ▼
[AmbiEncode2D]
      ▼
[AmbiDecode2D]
      ▼
[Stereo Output]
```
### 7. Legende / Mapping

```text
| Element           | Function                                     |
| ----------------- | -------------------------------------------- |
| r                 | Frequency base, spatial orientation, azimuth |
| t1                | Spectral width, amplitude modulation         |
| t2                | Detuning and spectral instability            |
| density           | Global excitation and spectral energy        |
| modal bank        | Resonant synthesis structure                 |
| encoding/decoding | Circular harmonic spatial projection         |
```
