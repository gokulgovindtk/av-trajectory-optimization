# AV Trajectory Optimization under Friction Constraints

## Problem
Emergency lane change at 90 km/h must respect 
tire friction limits to prevent vehicle skid.

## Approach
Quintic polynomial path planning enforcing 
smooth position, heading and curvature.

## Key Result
Minimum safe maneuver distance: **56.99m at 25 m/s**

## Tech Stack
- Python, NumPy, Matplotlib
- Linear algebra solver (numpy.linalg)

## Files
- `Autonomous.pdf` - Full mathematical derivation + code
