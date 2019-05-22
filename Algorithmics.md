Additional algorithmics:
==========================

## Jacobian for TDOA measurement (For kalman filtering)

Implmented at Bitcraze for Kalman filtering of each TDoA measurement: [kalman_core.c](https://github.com/bitcraze/crazyflie-firmware/blob/ebf447ee94f524ddf28cccdd7032ff4126f079cc/src/modules/src/kalman_core.c#L386-L388)

```
Jacobian for each measurement = [ 
    (x-Bx)/|PB| - (x-Ax)/|PA|,
    (y-By)/|PB| - (y-Ay)/|PA|,
    (z-Bz)/|PB| - (z-Az)/|PA|
]
```

You can find it using wxmaxima with the following:

With:
* P(x,y,z): the tag position
* A: Anchor A
* B: Anchor B
* dA : |PA| // Distance from position to A
* dB : |PB| // Distance from position to B
* tAB : TDoA measurement
* TAB : tAB = |PB| - |PA| // TDOA measurement equation

```maxima
dA(x,y,z) := sqrt((x-A[1])²+(y-A[2])²+(z-A[3])²);
dB(x,y,z) := sqrt((x-B[1])²+(y-B[2])²+(z-B[3])²);
TAB(x,y,z) := dB(x,y,z)-dA(x,y,z);

J : jacobian([TAB(x,y,z)], [x,y,z]);

/* Terms of the jacobian */
X = submatrix(J, 2,3);
Y = submatrix(J, 1,3);
Z = submatrix(J, 2,1);
```

## Jacobian of our cost function (LSE) for optimization problem

As the derivative of a sum is the sum of the derivative, we will sum the jacobians of the cost function for each measurement.

```
CostFunction for N measurements from i = 0 to i = N-1:

=> sum((|PB_i| - |PA_i| - tAB_i)²)

# Element-wise sum for each measurement of the following equations:
Jacobian = [
    2*((x-Bx)/|PB| - (x-Ax)/|PA|)*(-tAB + |PB|-|PA|),
    2*((y-By)/|PB| - (y-Ay)/|PA|)*(-tAB + |PB|-|PA|),
    2*((y-By)/|PB| - (y-Ay)/|PA|)*(-tAB + |PB|-|PA|)
]
```

You can find it using wxmaxima with the following:

With:
* P(x,y,z): the tag position
* A: Anchor A
* B: Anchor B
* dA : |PA| // Distance from position to A
* dB : |PB| // Distance from position to B
* tAB : TDoA measurement
* TAB : tAB = |PB| - |PA| // TDOA measurement equation

```maxima
dA(x,y,z) := sqrt((x-A[1])²+(y-A[2])²+(z-A[3])²);
dB(x,y,z) := sqrt((x-B[1])²+(y-B[2])²+(z-B[3])²);
TAB(x,y,z) :=  dB(x,y,z)-dA(x,y,z) - tAB;

J : jacobian([TAB(x,y,z)²], [x,y,z]);

X = submatrix(J, 2,3);
Y = submatrix(J, 1,3);
Z = submatrix(J, 2,1);
```

