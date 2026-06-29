# Threshold-Sensitivity Analysis
The analysis measures how the choice of safety threshold, denoted by (\theta_{\mathcal{S}}), affects whether the perception stack reaches a sufficient confidence level before the stopping-distance boundary.

A scenario is counted as **successful** if the calibrated ensemble mean reaches the threshold (\theta_{\mathcal{S}}) before the stopping-distance boundary (sd). Success rates are reported as percentages with 95% bootstrap confidence intervals. The distance-to-threshold value (d_{\mathcal{S}}) is reported as median [IQR] in meters for scenarios where the threshold is reached. A dash (`--`) indicates that no scenario reached the corresponding threshold.

## Threshold Sweep Results

| (\theta_{\mathcal{S}}) | Experiment 1, N=972: Success before (sd) (%) | Experiment 1: (d_{\mathcal{S}}) (m) | Experiment 2, N=1054: Success before (sd) (%) | Experiment 2: (d_{\mathcal{S}}) (m) |
| ---------------------: | -------------------------------------------: | ----------------------------------: | --------------------------------------------: | ----------------------------------: |
|                   0.60 |                            30.9 [28.1, 33.6] |                22.55 [16.55, 25.55] |                             31.4 [28.6, 34.3] |                19.55 [16.55, 25.55] |
|                   0.65 |                            24.3 [21.6, 26.9] |                19.55 [16.55, 22.55] |                             26.8 [24.1, 29.4] |                19.55 [16.55, 25.55] |
|                   0.70 |                            21.5 [18.9, 24.1] |                19.55 [13.55, 22.55] |                             22.2 [19.6, 24.8] |                19.55 [13.55, 22.55] |
|                   0.75 |                            19.2 [16.9, 21.6] |                19.55 [13.55, 22.55] |                             18.5 [16.2, 20.8] |                16.55 [13.55, 22.55] |
|                   0.80 |                            15.7 [13.6, 18.0] |                16.55 [13.55, 22.55] |                             12.9 [10.8, 15.1] |                16.55 [10.55, 19.55] |
|                   0.85 |                               0.9 [0.4, 1.5] |                16.55 [10.55, 19.55] |                                3.8 [2.7, 4.9] |                13.55 [10.55, 19.55] |
|                   0.90 |                               0.0 [0.0, 0.0] |                 10.55 [7.55, 13.55] |                                0.0 [0.0, 0.0] |                 10.55 [7.55, 13.55] |
|                   0.95 |                               0.0 [0.0, 0.0] |                   4.55 [4.55, 4.55] |                                0.0 [0.0, 0.0] |                   4.55 [4.55, 4.55] |
|                   1.00 |                               0.0 [0.0, 0.0] |                                  -- |                                0.0 [0.0, 0.0] |                                  -- |


## Key Takeaways

* Lower safety thresholds produce higher success rates but may accept weaker perception confidence.
* Higher thresholds impose stricter safety requirements but sharply reduce the number of scenarios that succeed before the stopping-distance boundary.
* The transition from (\theta_{\mathcal{S}} = 0.80) to (\theta_{\mathcal{S}} = 0.85) marks a major drop in success rate.
* Thresholds of 0.90 and above are too strict for the evaluated perception stack under these simulation campaigns.
* The default threshold (\theta_{\mathcal{S}} = 0.75) provides a meaningful operating point for the main analysis, while still showing measurable sensitivity to environmental and adversarial factors.