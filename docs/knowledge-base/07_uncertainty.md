# Uncertainty

## Sources

Uncertainty can arise from:

- sparse LiDAR returns
- occlusion
- sensor noise
- low detection confidence
- tracking instability
- prediction uncertainty
- distant objects
- weather and lighting conditions

## Adaptive-Perception Rule

High uncertainty increases perception priority because uncertain regions may require additional information. Uncertainty must be treated as a measurable input rather than a visual label only.

## Requirements

Document the representation and scale of uncertainty, how it is propagated from detection through tracking and prediction, how it affects risk, and how it affects resolution selection. Do not confuse uncertainty with risk: an object can be low-risk but uncertain, or high-risk and well observed.
