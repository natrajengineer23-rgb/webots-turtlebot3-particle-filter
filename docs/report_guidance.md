# Two-page report evidence checklist

The report is limited to two pages, so use compact evidence rather than code listings.

## Recommended layout

### Page 1 — Mapping (20 marks)

1. **Method:** State that teleoperated mapping uses command-seeded, scan-to-submap point-to-point ICP and never reads wheel encoders. Mention the rolling local reference, correspondence/correction rejection, SVD rigid alignment, log-odds updates and Bresenham ray tracing.
2. **Justification:** LiDAR scan matching directly uses environmental geometry and avoids wheel-slip/encoder drift, while occupancy grids are compact and compatible with the particle filter.
3. **Evidence:** Insert `maps/map.pgm`, label free/occupied/unknown cells and report the mean accepted ICP RMSE and rejection percentage from `scan_matching_log.csv`.

### Page 2 — Particle filter (80 marks)

1. **Implementation:** Summarise global free-space initialisation, noisy commanded-velocity prediction, tempered LiDAR likelihood-field weighting, normalisation, effective sample size, systematic resampling, roughening and small random-particle injection.
2. **Results:** Include the estimated trajectory and a small table comparing particle counts. Suggested columns: particles, mean update time, convergence time, final position error and final heading error.
3. **What works:** Use observations from your actual runs, such as convergence in geometrically distinctive areas.
4. **Limitations:** Discuss scan-matching failure under large motion, symmetry/multimodality, particle depletion, grid resolution and the mismatch between commanded and achieved velocity.

## Metrics

- Translation error: $e_t=\sqrt{(x-\hat{x})^2+(y-\hat{y})^2}$
- Heading error: $e_\theta=\left|\operatorname{wrap}(\theta-\hat{\theta})\right|$
- Effective sample size: $N_{eff}=1/\sum_i w_i^2$
- ICP rejection rate: rejected scan matches / attempted scan matches

Use ground truth only when calculating evaluation metrics, never as an algorithm input.

## Recorded final-run results

The supplied logs contain 836 mapping updates over 160.384 s and 607 localisation updates over 116.416 s. All scan matches passed the configured quality gates. ICP RMSE was 0.00533 m on average, 0.00645 m at the median and 0.01108 m at the 95th percentile; the 0.07940 m maximum should be reported as a transient outlier.

With 800 particles, median effective sample size was 450.9. The effective sample size fell below the 280-particle resampling threshold on 46 updates (7.6%), after which systematic resampling, roughening and random injection restored diversity. Estimated start-to-finish separation was 0.195 m using the first global estimate and 0.063 m when measured after the first 10 s of initialisation.

These are internal consistency and repeatability results. Because simulator ground truth was deliberately not read or logged, do not describe start-to-finish separation as absolute localisation error. State this absence of ground-truth comparison as an evaluation limitation.
