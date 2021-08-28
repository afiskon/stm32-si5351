#!/usr/bin/env python3
# vim: set ai et ts=4 sw=4:

from math import floor, ceil
import sys

def si5351_iqmode(Fclk):
    # There are no solution above 100 MHz without using integer mode,
    # which is not supported when phase shift is enabled.
    if Fclk < 55_000 or Fclk > 100_000_000:
        return None

    rdiv = 0
    if Fclk < 750_000:
        Fclk *= 128
        rdiv = 7
    elif Fclk < 1_500_000:
        Fclk *= 64
        rdiv = 6
    elif Fclk < 3_000_000:
        Fclk *= 32
        rdiv = 5
    elif Fclk < 6_000_000:
        Fclk *= 16
        rdiv = 4
    elif Fclk < 9_000_000: # actual threshold is ~7_050_000
        Fclk *= 8
        rdiv = 3

    Fxtal = 25_000_000
    Nmin, Nmax = 24, 36 # PLL should run between 600 Meg and 900 Meg
    Mmin, Mmax = 8, 1800

    PhOff = floor(900_000_000 / Fclk)
    if PhOff > 127:
      print("Constraint violation: PhOff = {}".format(PhOff))
      return None

    Fpll = Fclk*PhOff

    A = floor(Fpll / Fxtal)
    B = floor((Fpll % Fxtal) / 24)
    C = floor(Fxtal / 24)

    X = PhOff
    Y = 0
    Z = 1
 
    if A < Nmin or A > Nmax or X < Mmin or X > Mmax or (X == Mmin and Y == 0):
      print("Constraint violation: A = {}, X = {}, Y = {}".format(A, X, Y))
      return None

    N = A+B/C
    M = X+Y/Z
    Fres = floor(Fxtal*N/(M * (1 << rdiv)))
    return { 'pll': {'a': A, 'b': B, 'c': C}, 'ms': {'a': X, 'b': Y, 'c': Z}, 'rdiv': rdiv, 'freq': Fres, 'phoff' : PhOff}

if __name__ == '__main__':
    print(si5351_iqmode(3_560_000))
    print(si5351_iqmode(7_030_000))
    print(si5351_iqmode(14_060_000))
    print(si5351_iqmode(28_060_000))

    step = 1
    max_err = 0
    min_phoff =  999
    max_phoff = -999
    for Fclk in range(55_000, 100_000_000+1, step):
        if Fclk % 1_000_000 == 0:
            print("{}...".format(Fclk))
        result = si5351_iqmode(Fclk)
        if result is None:
            print("{} - no solution".format(Fclk))
            break
        err = abs(result['freq'] - Fclk)
        max_err = max(max_err, err)
        min_phoff = min(min_phoff, result['phoff'])
        max_phoff = max(max_phoff, result['phoff'])
        if err > 3:
            print("Fclk = {}, result = {} - wrong frequency, err: {}".format(Fclk, result, abs(result['freq'] - Fclk)))
            break
    print("All done! max_err = {}, min_phoff = {}, max_phoff = {}".format(max_err, min_phoff, max_phoff))
