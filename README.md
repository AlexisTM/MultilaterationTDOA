# MultilaterationTDOA
This library will be used for TDoA localization from a [Bitcraze loco positioning system](https://www.bitcraze.io/loco-pos-system/).

This library fits your use-case if:

* **Setup**:
    * Multiple fixd anchors doing TWR between them
    * Tags receiving data from the anchors and computing the Time Difference on Arrival of messages between the messages of Anchors
* **Measurements**:
    * Position anchor 1 (m)
    * Position anchor 2 (m)
    * TDoA (m, you can take the time divided by the speed of light for the conversion)
    * With P = receiver position
    * With A = Anchor A position
    * With B = Anchor B position
    * TDoA = |PB| - |PA|

## Minimal example

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division, print_function
from multilateration_tdoa import TDoAEngine, TDoAMeasurement, Anchor, Point
import numpy as np

NOISE = 0.25
MEAN_NOISE = 0.0

def noise():
    return np.random.normal(MEAN_NOISE, NOISE)

engine = TDoAEngine(n_measurements=6, max_dist_hess=100) # Avoid value rejection.

A = Anchor((3,3,0))
B = Anchor((-2,2,0))
C = Anchor((2,-4,0))
D = Anchor((-3,-2,0))
P = Point(0,0,0)

# These distances are computed from geogebra
engine.add(TDoAMeasurement(A, C, 0.23 + noise()))
engine.add(TDoAMeasurement(A, B, -1.41 + noise()))
engine.add(TDoAMeasurement(A, D, -0.64 + noise()))
engine.add(TDoAMeasurement(B, C, 1.64 + noise()))
engine.add(TDoAMeasurement(B, D, 0.78 + noise()))
engine.add(TDoAMeasurement(C, D, -0.87 + noise()))


print("\n\nSolve in 3D from anchors on the ground and tag on the ground")
result, hess_inv = engine.solve()
print("result", result)
print("expected", P)
print("real error:", P.dist(result))
```

## Basic testing

To test the system, you can configure and run `python test.py` to locate from simulated data.

## Anchor placement testing

You can configure then run `python anchor_position_test.py` to generate a heatmap of localization quality.

![anchor_test_example.png](img/anchor_test_example.png)

## Author

- Alexis PAQUES (@AlexisTM)
