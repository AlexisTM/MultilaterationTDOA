Additional algorithmics:
==========================

## Jacobian for each TDOA measurement (For direct kalman filtering)

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

## Jacobian for the 3D localization (X-Y-Z): LSE

The Z axis (or another constraint) can be fixed as in the real implementation.
Fixing the Z axis makes it much more precise than just 3D.

As the derivative of a sum is the sum of the derivative, we will sum the jacobians of the cost function for each measurement.

```
CostFunction for N measurements from i = 0 to i = N-1:

=> sum((|PB_i| - |PA_i| - tAB_i)²)

# Element-wise sum for each measurement of the following equations:
Jacobian = [
    2*((x-Bx)/|PB| - (x-Ax)/|PA|)*(-tAB + |PB|-|PA|),
    2*((y-By)/|PB| - (y-Ay)/|PA|)*(-tAB + |PB|-|PA|),
    2*((z-Bz)/|PB| - (z-Az)/|PA|)*(-tAB + |PB|-|PA|)
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

## Jacobian for 4D localization (X-Y-Z-Theta) with 2 receivers: LSE

This is identical to the 3D localization on the difference that each receiver has a position which depends on theta.

The only difference being the position **Kb(Kb_x,Kb_y,Kb_z)** of the anchor on the base_link frame at the position **P(x,y,z,theta)** written **K(K_x,K_y,K_z)**.

**Kr(Kr_x, Kr_y, Kr_z)** is Kb rotated by theta around Z axis. It is kept as an intermediate term because used later on to express the jacobian.

```maxima
[ Kr_x ]   [ Kb_x ]   [ cos(theta)   -sin(theta)   0 ]
[ Kr_y ] = [ Kb_y ] * [ sin(theta)    cos(theta)   0 ]
[ Kr_z ]   [ Kb_z ]   [ 0             0            1 ]


[ K_x ]   [ x ]   [ Kr_x ]
[ K_y ] = [ y ] + [ Kr_y ]
[ K_z ]   [ z ]   [ Kr_z ]
```

The cost function stays the same, using the position **K(K_x,K_y,K_z)** instead of **P(x,y,z)** for the error computation.

With:
* P(x,y,z): the tag position
* A: Anchor A
* B: Anchor B
* dA : |PA| // Distance from position to A
* dB : |PB| // Distance from position to B
* tAB : TDoA measurement
* TAB : tAB = |PB| - |PA| // TDOA measurement equation

The jacobian becomes:

```maxima
# 2*(-tAB + |PB|-|PA|) comes from the square of the LSE.
Jacobian = [
    ((K_x-Bx)/|PB| - (x-Ax)/|PA|)*2*(-tAB + |PB|-|PA|),
    ((K_y-By)/|PB| - (y-Ay)/|PA|)*2*(-tAB + |PB|-|PA|),
    ((K_z-Bz)/|PB| - (z-Az)/|PA|)*2*(-tAB + |PB|-|PA|),
    ((-Kr_y*(K_x-Bx) + Kr_x*(K_y-By))/|PB| - 
     (-Kr_y*(K_x-Ax) + Kr_x*(K_y-By))/|PA|)*2*(-tAB + |PB|-|PA|)
]
```

You can find it using the following xmaxima script:

```maxima
/* P is the position of the rover */

P:matrix([x],[y],[z]);  

/* K1 and K2 are anchor positions in map */ 
/* K1b is the position of the anchor in the base_link of the rover */

K: matrix(
    [x + Kb[x]*cos(theta) - Kb[y]*sin(theta)],
    [y + Kb[x]*sin(theta) + Kb[y]*cos(theta)],
    [z + Kb[z]]
);

A: matrix([A[x]],[A[y]],[A[z]]);
B: matrix([B[x]],[B[y]],[B[z]]);

dNKA(x,y,z,theta) := norm(K-A);
dNKB(x,y,z,theta) := norm(K-B);

dKA(x,y,z,theta) := sqrt((K[1]-A[1])²+(K[2]-A[2])²+(K[3]-A[3])²);
dKB(x,y,z,theta) := sqrt((K[1]-B[1])²+(K[2]-B[2])²+(K[3]-B[3])²);

DKAB(x,y,z,theta) :=  dKB(x,y,z,theta)-dKA(x,y,z,theta) - tKABm;

DNKAB(x,y,z,theta) := dNKB(x,y,z,theta)-dNKA(x,y,z,theta) - tKABm;

diff(DNKAB(x,y,z,theta)², theta, 1);

jacobian(DKAB(x,y,z,theta)², [theta]);
```
