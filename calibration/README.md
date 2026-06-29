# Confidence Calibration
Each model is evaluated on stop-sign detections from COCO-val. We report the number of evaluated predictions/samples (N), the number of correct stop-sign detections under the repository's matching protocol, and calibration quality before and after calibration.

Lower values of ECE and NLL indicate better calibrated confidence estimates.

## Calibration Results

| Model   |  (N) | Correct | Raw ECE | Calibrated ECE | Raw NLL | Calibrated NLL |
| ------- | ---: | ------: | ------: | -------------: | ------: | -------------: |
| YOLOv8  | 1000 |      72 |  0.0133 |         0.0124 |  0.0947 |         0.0926 |
| YOLOv9  |  737 |      71 |  0.0197 |         0.0132 |  0.1164 |         0.1130 |
| RT-DETR | 5756 |      68 |  0.0328 |         0.0030 |  0.0445 |         0.0163 |
| DETR50  |  776 |      65 |  0.0411 |         0.0158 |  0.1314 |         0.0833 |
| DETR101 |  901 |      64 |  0.0366 |         0.0121 |  0.1157 |         0.0783 |

## Interpretation

The calibration results show that post-hoc confidence calibration improves the reliability of stop-sign confidence estimates for all evaluated detectors. The improvement is visible in both calibration metrics: Expected Calibration Error (ECE) and Negative Log-Likelihood (NLL).

The largest improvement is observed for RT-DETR, where ECE decreases from 0.0328 to 0.0030 and NLL decreases from 0.0445 to 0.0163. The DETR-based models also show substantial improvements after calibration, especially in NLL. YOLOv8 is already relatively well calibrated before calibration, so the improvement is smaller but still positive.

These results support the use of model-specific confidence calibration before combining detector outputs in the ensemble. Without calibration, confidence scores from different detector families may not be directly comparable, even when they refer to the same semantic class. Calibration reduces this mismatch and makes the downstream ensemble confidence more meaningful for safety-threshold analysis.

## Metrics

### Expected Calibration Error

Expected Calibration Error, or ECE, measures the gap between predicted confidence and empirical correctness. A lower ECE means that confidence scores better reflect the observed probability of correctness.

For example, if a detector assigns confidence scores around 0.80, then approximately 80% of those detections should be correct for the model to be well calibrated.

### Negative Log-Likelihood

Negative Log-Likelihood, or NLL, measures the probabilistic quality of the predicted confidence values. Lower NLL indicates that the model assigns higher probability to correct detections and lower probability to incorrect detections.

## Key Takeaways

* All evaluated detectors benefit from confidence calibration.
* RT-DETR shows the strongest calibration improvement.
* YOLOv8 starts with the lowest raw ECE and therefore shows a smaller calibration gain.
* DETR50 and DETR101 show clear reductions in both ECE and NLL after calibration.
* Calibration is important before ensemble fusion because different detector families may produce confidence scores with different reliability characteristics.

## Notes

* (N) denotes the number of evaluated stop-sign predictions/samples for each model.
* `Correct` denotes the number of detections counted as correct under the repository's matching protocol.
* `Raw ECE` and `Raw NLL` are computed using the original model confidence scores.
* `Calibrated ECE` and `Calibrated NLL` are computed after applying the calibration procedure.
* Lower ECE and lower NLL indicate better calibration.
