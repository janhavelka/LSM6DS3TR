# Noise Analysis and Identification in MEMS Sensors
**Source:** noise_analysis.pdf | **Doc #:** DT0064 Rev 1 | **Pages:** 6

## Key Takeaways
- Allan variance (AVAR) is the standard tool for MEMS noise characterization; it decomposes noise by type using slope analysis on log-log σ(τ) plots
- Overlapping variants (OAVAR, OHVAR) improve confidence and should be used up to τ = 10% of data run length; Total variants extend to 30–50%, Theo1 to 75%
- The processing chain is: Moving Average → Differencer → (optional second average or differencer) → square-and-average → √ for deviation
- Angular Random Walk (ARW) for a gyroscope equals the noise density in deg/s/√Hz and is read from the Allan deviation plot at the τ⁻¹/² intercept at τ = 1 s
- Flicker (1/f) noise is distinguished from white noise by its flat (τ⁰) Allan deviation slope; Modified Allan or Time variance is needed to separate White PM from Flicker PM

## Summary

This design tip explains how to characterize MEMS inertial sensor noise using Allan variance and related statistical tools. The fundamental assumption is that the signal of interest is constant during measurement, and the sensor output equals signal plus noise. Standard variance is not well-behaved for increasing data length; Allan variance resolves this by computing the average squared difference between consecutive averaged samples over integration time τ = m·Ts.

The document covers non-overlapping and overlapping variants of Allan (AVAR/OAVAR), Modified Allan (MAVAR), Hadamard (HVAR/OHVAR), Time variance (TVAR), Total variants, and Theo1. Each is expressed as a chain of digital filters (moving average M, first-difference D1, downsampler) applied to the raw data. Reference MATLAB implementations are provided in both straightforward filter form and optimized running-sum form.

Noise types — White PM, Flicker PM, White FM, Flicker FM, Random Walk, Flicker Walk, Random Run — are identified by their characteristic slopes on σ-τ and σ²-τ plots. A 1/f noise generator algorithm using octave-band white generators is also provided.

## Technical Details

### Allan Variance Definition

$$\sigma^2_{AVAR}(\tau) = \frac{1}{2} \left\langle (y_{k+1} - y_k)^2 \right\rangle$$

where $y_k$ is the average of $m$ consecutive samples, $\tau = m \cdot T_s$.

### Processing Chain (Digital Filter Representation)

| Variance | Filter Chain |
|----------|-------------|
| AVAR | M(m) → D1(m) → m:1 ↓ → square & avg |
| OAVAR | M(m) → D1(m) → square & avg (no downsampling) |
| MAVAR | M(m) → D1(m) → M(m) → square & avg |
| HVAR | M(m) → D1(m) → D1(m) → m:1 ↓ → square & avg |
| OHVAR | M(m) → D1(m) → D1(m) → square & avg |

Where M(m) = moving average over m samples; D1(m) = difference between sample n and sample n+m+1.

### Optimized OAVAR (Running Sum)

```
acc[1] = sum(data[1:n])
for i = 1 to N-n:
    acc[i+1] = acc[i] - data[i] + data[i+n]
oavar = 0.5 * sum((acc[1:N-2n+1] - acc[1+n:N-n+1])²) / (N-2n+1) / n²
```

### Noise Type Identification (Slope Table)

| Noise Type | Spectrum | ADEV σ(τ) | AVAR σ²(τ) | MADEV | MAVAR | TDEV | TVAR |
|------------|----------|-----------|------------|-------|-------|------|------|
| White PM | $f^{+2}$ | $\tau^{-1}$ | $\tau^{-2}$ | $\tau^{-3/2}$ | $\tau^{-3}$ | $\tau^{-1/2}$ | $\tau^{-1}$ |
| Flicker PM | $f^{+1}$ | $\tau^{-1}$ | $\tau^{-2}$ | $\tau^{-1}$ | $\tau^{-2}$ | $\tau^{0}$ | $\tau^{0}$ |
| White FM | $f^{0}$ | $\tau^{-1/2}$ | $\tau^{-1}$ | $\tau^{-1/2}$ | $\tau^{-1}$ | $\tau^{+1/2}$ | $\tau^{+1}$ |
| Flicker FM | $f^{-1}$ | $\tau^{0}$ | $\tau^{0}$ | $\tau^{0}$ | $\tau^{0}$ | $\tau^{+1}$ | $\tau^{+2}$ |
| Random Walk | $f^{-2}$ | $\tau^{+1/2}$ | $\tau^{+1}$ | $\tau^{+1/2}$ | $\tau^{+1}$ | $\tau^{+3/2}$ | $\tau^{+3}$ |
| Flicker Walk | $f^{-3}$ | $\tau^{+1}$ | $\tau^{+2}$ | $\tau^{+1}$ | $\tau^{+2}$ | — | — |
| Random Run | $f^{-4}$ | $\tau^{+3/2}$ | $\tau^{+3}$ | $\tau^{+3/2}$ | $\tau^{+3}$ | — | — |

### Confidence and Usable Range

| Method | Usable up to τ = % of data run |
|--------|-------------------------------|
| OAVAR / OHVAR | 10% |
| Total variants (Allan, Modified, Hadamard) | 30–50% |
| Theo1 / Theo1BR / Theo1H | 75% |

Total variance extends data by reflection: 2m samples for Allan Total, 3m for Modified Allan Total and Hadamard Total.

### Gyroscope Angular Random Walk (ARW)

$$\text{ARW} = \text{noise density in } \deg/s/\sqrt{Hz}$$

$$\text{Final angle error RMS} = \text{ARW} \times \sqrt{t}$$

Example: ARW = 1 deg/√s × √1000 s = 31.6° RMS.

ARW is read from the Allan deviation plot as the intercept of the $\tau^{-1/2}$ slope segment at $\tau = 1$ s.

### 1/f (Flicker) Noise Generator

```
n = zeros(1, N)
imax = floor(log2(N))
ngen = randn(1, imax+1) * sd
ngensum = sum(ngen)
for i = 1:N
    tlz = trailing_zeros(i)
    ngensum -= ngen[tlz+1]
    ngen[tlz+1] = randn(1) * sd
    ngensum += ngen[tlz+1]
    n[i] = ngensum + rand(1)*sd
```

## Relevance to LSM6DS3TR Implementation

The LSM6DS3TR's accelerometer and gyroscope noise can be characterized using Allan variance to determine the optimal averaging time, identify dominant noise types, and extract ARW for the gyroscope. This directly informs the choice of ODR, filter bandwidth, and integration time for dead-reckoning (DT0106) and tilt computation (DT0058/DT0060). The datasheet noise density values can be verified experimentally: collect a long static data run, compute OAVAR, and read ARW at the τ⁻¹/² intercept. The minimum of the Allan deviation curve indicates the optimal averaging time beyond which longer averaging no longer reduces noise (flicker floor).
