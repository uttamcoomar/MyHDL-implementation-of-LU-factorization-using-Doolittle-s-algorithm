# MyHDL-implementation-of-LU-factorization-using-Doolittle-s-algorithm
 This project presents a MyHDL code that implements LU factorization of a 4X4 matrix using Doolittle's approach. This design assumes that the elements of A in A=LU are sufficiently large to avoid requiring row swaps.
# MyHDL LU Factorization (Doolittle's Method)

A hardware implementation of LU matrix factorization, for a 4X4 matrix, using Doolittle's algorithm, written in MyHDL.

## Overview

This project implements LU decomposition in hardware description language using MyHDL. LU factorization decomposes a matrix A into lower triangular (L) and upper triangular (U) matrices such that A = LU.

## Features

- Implements Doolittle's algorithm for LU factorization
- Synthesizable MyHDL code
- Supports 4x4 matrices
- fixed-point arithmetic using Q8.24 signed numbers

## Requirements

- Python 3.12.3 or later
- MyHDL 0.11
- GHDL
- GTKWave

## Files in this repository

- LUDoolittle_q8_24.py (The MyHDL file)
- pck_myhdl_011.vhd (Dependency for top.vhd. Include it in the same folder as top.vhd)
- top.vhd (VHDL generated using MyHDL)

## Doolittle's Algorithm
Doolittle's method is an approach to LU decomposition where the diagonal elements of the lower triangular matrix L are set to 1.

### Mathematical Formulation

Given an n×n matrix A, we decompose it into:
```
A = LU
```

Where:
- **L** is a lower triangular matrix with 1s on the diagonal
- **U** is an upper triangular matrix

### Algorithm Steps

The decomposition is computed using the following formulas:

**For the Upper Triangular Matrix U:**

For i = 1 to n, and j = i to n:
```
U[i][j] = A[i][j] - Σ(k=1 to i-1) L[i][k] * U[k][j]
```

**For the Lower Triangular Matrix L:**

For i = 2 to n, and j = 1 to i-1:
```
L[i][j] = (A[i][j] - Σ(k=1 to j-1) L[i][k] * U[k][j]) / U[j][j]
```

With L[i][i] = 1 for all i.

### Example

For a 3×3 matrix:
```
     [a11  a12  a13]       [1    0    0]   [u11  u12  u13]
A =  [a21  a22  a23]  =  L [l21  1    0] × [0    u22  u23]
     [a31  a32  a33]       [l31  l32  1]   [0    0    u33]
```

### Computational Order

1. Compute first row of U: u11, u12, u13, ...
2. Compute first column of L: l21, l31, ...
3. Compute second row of U: u22, u23, ...
4. Compute second column of L: l32, l42, ...
5. Continue alternating until complete


## License



## Author
[Uttam Kumar/uttamcoomar]
