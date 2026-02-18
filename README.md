# Adaptive 9D Boid Sound Engine – Ausführliche Dokumentation

Die Klangsynthese basiert auf einem Modal-Synthese-Verfahren, gesteuert durch eine neundimensionale Schwarmsimulation.  
Die Erweiterung auf neun Dimensionen ermöglicht eine funktionale Differenzierung des Zustandsraums und erlaubt gezielte Kontrolle über spektrale, dynamische, räumliche und ereignisbasierte Eigenschaften des Klangfeldes.

Anstelle klassischer Masse-Feder-Dämpfer-Modelle werden resonante, bandbegrenzte Modes eingesetzt.  
Jeder Mode besteht aus mehreren parametrisierten Resonatoren, deren Frequenz, Bandbreite und Amplitude kontinuierlich angepasst werden.  
Physikalische Kopplungsmatrizen werden nicht berechnet; Interaktion erfolgt indirekt über gemeinsame Anregung und parameterbasierte Transformation, wodurch numerische Instabilitäten vermieden werden und ein kontrollierbares, komplexes Resonanzverhalten entsteht.

Jeder Mode ist einem Boid des Schwarms zugeordnet und wird aus dessen neundimensionalem Zustandsvektor abgeleitet.  
Die neun Dimensionen werden in drei funktionale Subräume gegliedert.

---

## 1. Subräume & Dimensionen

| Subraum / Dimensionen | Physikalische Bedeutung | Klangliche Wirkung / Steuerparameter |
|----------------------|-----------------------|-------------------------------------|
| Subspace 1 (dims 0–2) | Radialvektor r, normiert + kinetische Energie | Grundfrequenz über r.y, Detuning höherer Modes, Azimut & Distanz für Ambisonics; Geschwindigkeit → zusätzliche Modulation |
| Subspace 2 (dims 3–5) | Tangential-/Seitwärtsvektor t1, orthogonal zu r | Moduliert Amplitude (ampChaotic), Bandwidth, diffuse/chaotische Energie; beeinflusst spektrale Ausdehnung |
| Subspace 3 (dims 6–8) | Normalenvektor t2 = r × t1, Länge = Curvature | Detuning (chaotische Harmonien), spektrale Breite; moduliert harmonische Struktur |

**Lokale Nachbarschaft / Dichte:**  
- Abstand zu Nachbarn innerhalb definierter Subräume  
- Steuert Triggerwahrscheinlichkeit, Amplitude-Lift, Bandwidth-Lift, spektrale Verdichtung; skaliert mit Flow & Curvature  

**Trigger / Envelope:**  
- Ereignisbasierte Anregung abhängig von Dichte & Geschwindigkeit  
- LinearEnvelope moduliert Rauschsignal → regt Modalbank an  
- Attack/Decay abhängig von Dichte, Peak von Geschwindigkeit  

**Velocity-Modulation:** Nur der radiale Subraum moduliert energetische Parameter direkt (Amplitude, Detuning).  

**Subräume einfrieren:** Einzelne Subräume können temporär stabilisiert werden → gezielte Reduktion klanglicher Komplexität.

---

## 2. Frenet-Frame-Logik pro Boid

Jeder Boid wird mathematisch ähnlich einem Frenet-Frame interpretiert.

### 2.1 Radialvektor r (dims 0–2)

```text
glm::vec3 r(b.dims[0], b.dims[1], b.dims[2]);

// Nur die Richtung zählt
if (glm::length(r) > 1e-6)
{
    r = glm::normalize(r);
}
```
- Klanglich: r.y → Grundfrequenz, Geschwindigkeit → zusätzliche Tonhöhenmodulation

 ### 2.2 Tangentialvektor t1 (dims 3–5)

```text
glm::vec3 t1(b.dims[3], b.dims[4], b.dims[5]);

// Orthogonal zu r projizieren
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
- Maß tangentialer Energie = flow
- Klanglich: steigert Amplitude & Bandwidth, diffuses Timbre

### 2.3 Normalenvektor t2 (dims 6–8)

```text
glm::vec3 t2 = glm::cross(r, t1);

double curvature = glm::length(t2);

if (curvature > 1e-6)
{
    t2 = glm::normalize(t2);
}
```
- Maß für Rotations-/Planarenergie = curvature
- Klanglich: moduliert Detuning, chaotische / breitere Klanganteile

### 2.4 Lokale Dichte (density)

- Berechnet über Abstände zu Nachbarn im vollständigen 9D-Zustandsraum (alle drei funktionalen Subräume).
- Emergenter kollektiver Parameter, der folgende Klangparameter moduliert:  
  - Triggerwahrscheinlichkeit  
  - Frequenzlift  
  - Amplitude  
  - Harmonische Stabilität / Bandwidth Lift
- Density wirkt als globaler Skalierungsfaktor auf spektrale, dynamische und ereignisbasierte Prozesse.

---

## 3. Frequenz-, Amplituden-, Bandwidth- & Detuning-Berechnung

```text
double radialHeight = 0.5 * (r.y + 1.0); // [-1,1] -> [0,1]
double density = densities[j];

double speed = glm::length(glm::vec3(
  b.velocity[0],
  b.velocity[1],
  b.velocity[2]
));

double radialCurve = pow(radialHeight, 0.6);
double densityLift = 1.0 + density * 1.5;

double targetFreq = preset.minFreq +
  (preset.maxFreq - preset.minFreq) * radialCurve * densityLift;

targetFreq = std::clamp(targetFreq, preset.minFreq, preset.maxFreq);

smoothedFreq[j] = 0.995 * smoothedFreq[j] + 0.005 * targetFreq;
```
- Radialer Subraum → Grundfrequenz  
- Dichte & Flow → Amplitude, Bandwidth  
- Curvature → Detuning, chaotische Anteile  
- Höhere Modi → diffuser, leiser, leicht detuned → komplexe Klangtextur  

### Amplitude

```text
double ampCinematic = 0.4 * (1.0 - density);
double ampChaotic = 0.8 * density * flow;

double ampBase = preset.minAmp +
  (preset.maxAmp - preset.minAmp) * (ampCinematic + ampChaotic);

ampBase = std::clamp(ampBase, preset.minAmp, preset.maxAmp);
```
### Bandwidth

```text
double bwTonal = 60.0;
double bwNoise = 500.0;
double bwMix = density * flow;

double baseBandwidth = bwTonal + (bwNoise - bwTonal) * bwMix;
baseBandwidth *= preset.bandwidthScale;
```
### Detuning

```text
double detuneClean = 0.001;
double detuneChaos = 0.05;

double detuneAmt = (detuneClean + (detuneChaos - detuneClean) *
  (0.7 * density + 0.3 * curvature)) * preset.detuneScale;

Parametrisierung Modalbank pro Boid & Mode

for(size_t m = 0; m < numModes; m++)
{
  double frq = smoothedFreq[j] * (1.0 + detuneAmt * m);
  double bw = baseBandwidth * (1.0 + 0.3 * m);
  double amp = ampBase / (1.0 + 0.4 * m);

  modalBank2D.setParams(j, m, frq, bw, amp);
}
```
- Höhere Modi → diffuser, leiser, leicht detuned → komplexe Klangtextur
  
---

## 4. Trigger & Envelope

- Ereignisbasierte Anregung abhängig von Dichte und radialem Flow  
- LinearEnvelope moduliert ein Rauschsignal zur Anregung der Resonatoren  
- Hüllkurve skalierbar in Intensität und Dauer → impulsartig oder flächig  

---

## 5. Ambisonic Mapping & Stereo

- Boid → Azimut & Distanz (radialer Subraum)  
- Ambisonics-Codierung → 7-Kanal 2D Ambisonics  
- Distanzabhängiger Gain: `exp(-3 * dist)`  
- Summation aller Boids → Gesamt-Ambisonic-Rahmen  
- Ambisonics-Decodierung → Stereo (L/R)  
- Soft-Clipping: `tanh()`  

---

## 6. Signalfluss-Diagramm (ASCII)
```text
[SwarmOSCReceiver]
      │
      │ (Boid.position[0..8], Boid.velocity[0..8])
      ▼
[BoidAggregator] ──> spatialPos[vec3], densities[float]
      │
      ├───> Subspace 1 → radial r → Grundfrequenz, Azimut & Distanz
      ├───> Subspace 2 → tangential t1 → Flow → Amplitude & Bandwidth
      └───> Subspace 3 → normal t2 → Krümmung → Detuning, chaotische Anteile
      ▼
[BoidToAmbi2D] ──> azimuthRad, distance
      ▼
[AmbiEncode2D] ──> 7-channel 2D ambisonics
      ▼
[ModalBank2D] (N Boids × M Modes)
      ├──> ModalFilter: Frq, Q, Amp per Mode
      ├──> Ereignisbasierte Excitation (Envelope)
      └──> Output summiert pro Boid
      ▼
[AmbiDecode2D] ──> Stereo (L/R)
      ▼
[Audio Output]
```
### 7. Legende / Mapping

| Element               | Funktion                                                       |
|----------------------|----------------------------------------------------------------|
| r                     | Grundfrequenz, räumliche Position (Azimut/Distanz), Detuning  |
| t1                    | Flow → Amplitude & Bandwidth                                   |
| t2                    | Detuning, chaotische Harmonische                               |
| Density               | Triggerwahrscheinlichkeit, harmonische Stabilität, Bandwidth Lift |
| Envelope              | Ereignisbasierte Energie → Anregung der Resonatoren           |
| Modalbank             | Modes erhalten Frq, Amp, Bandwidth, Detuning                  |
| AmbiEncode/Decode 2D  | Ambisonics → Stereo-Raumabbildung                              |
