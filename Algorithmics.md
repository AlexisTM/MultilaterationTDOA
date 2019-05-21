Additional algorithmics:
==========================

## Jacobian for TDOA measurement

```
Jacobian = [ 
    (x-Bx)/|PB| - (x-Ax)/|PA|,
    (y-By)/|PB| - (y-Ay)/|PA|
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
TAB(x,y,z) := tAB = dB(x,y,z)-dA(x,y,z);

JALL : jacobian([TAB(x,y,z)], [x,y,z]);

X : submatrix(JALL, 2,3);
Y : submatrix(JALL, 1,3);
Z : submatrix(JALL, 2,1);

jacobian([TAB(x,y,z)], [x,y,z]);
```

## Jacobian of LSE cost function for TDOA

```
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
TAB(x,y,z) :=  0 = dB(x,y,z)-dA(x,y,z) - tAB;

JLSE : jacobian([TAB(x,y,z)²], [x,y,z]);

X : submatrix(JLSE, 2,3);
Y : submatrix(JLSE, 1,3);
Z : submatrix(JLSE, 2,1);
```

results in:

```
jacobian= [
    0=2*((x-B[1])/|PB|-(x-A[1])/|PA|)*(-tAB+|PB|-|PA|)
    0=2*((y-B[2])/|PB|-(y-A[2])/|PA|)*(-tAB+|PB|-|PA|)
    0=2*((z-B[3])/|PB|-(z-A[3])/|PA|)*(-tAB+|PB|-|PA|)]
```

**IMPORTANT QUESTION:** by putting the `tAB` term on the other side of the equation, it is cancelled out. **What should be used?**

```maxima
dA(x,y,z) := sqrt((x-A[1])²+(y-A[2])²+(z-A[3])²);
dB(x,y,z) := sqrt((x-B[1])²+(y-B[2])²+(z-B[3])²);
TAB(x,y,z) :=  tAB = dB(x,y,z)-dA(x,y,z);

JLSE : jacobian([TAB(x,y,z)²], [x,y,z]);

X : submatrix(JLSE, 2,3);
Y : submatrix(JLSE, 1,3);
Z : submatrix(JLSE, 2,1);
```

results in:

```
jacobian = [
    0=2*((x-B[1])/|PB|-(x-A[1])/|PA|)*(|PB|-|PA|),
    0=2*((y-B[2])/|PB|-(y-A[2])/|PA|)*(|PB|-|PA|),
    0=2*((z-B[3])/|PB|-(z-A[3])/|PA|)*(|PB|-|PA|)
]
```