#!/usr/bin/env python3
# vim: set ai et ts=4 sw=4:

from math import floor, ceil
import sys

def si5351_calc(Fclk):
    if Fclk < 8_000 or Fclk > 160_000_000:
        return None

    rdiv = 0
    if Fclk < 1_000_000:
        Fclk *= 64
        rdiv = 6 # log2(64)

    Fxtal = 25_000_000
    Nmin, Nmax = 24, 36 # PLL should run between 600 Meg and 900 Meg
    Mmin, Mmax = 8, 1800 # OR: [4, 6]

    if Fclk < 81_000_000:
        # Valid for Fclk in 0.5..112.5Meg range
        # However an error is > 6 Hz above 81 Megs, 13 Hz in worse case
        A = 36 # PLL runs @ 900 Meg
        B = 0
        C = 1
        Fpll = 900_000_000
        X = floor(Fpll/Fclk)
        T = (Fclk >> 20) + 1
        Y = floor((Fpll % Fclk) / T)
        Z = floor(Fclk/T)
    else:
        # Valid for Fclk in 75..160 Meg range
        if Fclk >= 150_000_000:
            X = 4
        elif Fclk >= 100_000_000:
            X = 6
        else:
            X = 8
        Y = 0
        Z = 1
        Numerator = X*Fclk
        A = floor(Numerator/Fxtal)
        T = (Fxtal >> 20) + 1
        B = floor((Numerator % Fxtal) / T)
        C = floor(Fxtal / T)

    if A < Nmin or A > Nmax or (X != 4 and X != 6 and not (X >= Mmin or X <= Mmax)):
        print("Constraint violation: A = {}, X = {}".format(A, X))
        return None

    if B > 0xFFFFF or C == 0 or C > 0xFFFFF or Y > 0xFFFFF or Z == 0 or Z > 0xFFFFF:
        print("Constraint violation: B = {}, C = {}, Y = {}, Z = {}".format(B, C, Y, Z))
        return None

    N = A+B/C
    M = X+Y/Z
    Fres = floor(Fxtal*N/(M * (1 << rdiv)))
    return { 'pll': {'a': A, 'b': B, 'c': C}, 'ms': {'a': X, 'b': Y, 'c': Z}, 'rdiv': rdiv, 'freq': Fres}

if __name__ == '__main__':
    result = si5351_calc(145_500_000)
    print(result)
    # sys.exit(0)
    
    step = 1
    max_err = 0
    for Fclk in range(8_000, 160_000_000+1, step):
        if Fclk % 1_000_000 == 0:
            print("{}...".format(Fclk))
        result = si5351_calc(Fclk)
        if result is None:
            print("{} - no solution".format(Fclk))
            break
        err = abs(result['freq'] - Fclk)
        max_err = max(max_err, err)
        if err > 6:
            print("Fclk = {}, result = {} - wrong frequency, err: {}".format(Fclk, result, abs(result['freq'] - Fclk)))
            break
    print("All done! max_err = {}".format(max_err))
