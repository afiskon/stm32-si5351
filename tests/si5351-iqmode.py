#!/usr/bin/env python3
# vim: set ai et ts=4 sw=4:

from math import floor, ceil
import sys

def si5351_iqmode(Fclk):
    # There are no solution above 100 MHz without using integer mode,
    # which is not supported when phrequency shift is enabled.
    if Fclk < 8_000 or Fclk > 100_000_000:
        return None

    rdiv = 0
    if Fclk < 500_000:
        Fclk *= 64
        rdiv = 6 # log2(64)

    Fxtal = 25_000_000
    Nmin, Nmax = 24, 36 # PLL should run between 600 Meg and 900 Meg
    Mmin, Mmax = 8, 1800

    PhOff = floor(900_000_000 / Fclk)
    Fpll = Fclk*PhOff

    A = floor(Fpll / Fxtal)
    B = floor((Fpll % Fxtal) / 24)
    C = floor(Fxtal / 24)

    X = floor(Fpll/Fclk)
    T = Fclk >> 20
    if Fclk & 0xFFFFF:
        T += 1
    Y = floor((Fpll % Fclk) / T)
    Z = floor(Fclk/T)

    if A < Nmin or A > Nmax or X < Mmin or X > Mmax or (X == Mmin and Y == 0):
      print("Constraint violation: A = {}, X = {}, Y = {}".format(A, X, Y))
      return None

    N = A+B/C
    M = X+Y/Z
    Fres = floor(Fxtal*N/(M * (1 << rdiv)))
    return { 'pll': {'a': A, 'b': B, 'c': C}, 'ms': {'a': X, 'b': Y, 'c': Z}, 'rdiv': rdiv, 'freq': Fres, 'phoff' : PhOff}

if __name__ == '__main__':
    step = 1
    max_err = 0
    for Fclk in range(8_000, 100_000_000+1, step):
        if Fclk % 1_000_000 == 0:
            print("{}...".format(Fclk))
        result = si5351_iqmode(Fclk)
        if result is None:
            print("{} - no solution".format(Fclk))
            break
        err = abs(result['freq'] - Fclk)
        max_err = max(max_err, err)
        if err > 3:
            print("Fclk = {}, result = {} - wrong frequency, err: {}".format(Fclk, result, abs(result['freq'] - Fclk)))
            break
    print("All done! max_err = {}".format(max_err))
