#!/usr/bin/env python3
# vim: set ai et ts=4 sw=4:

from math import floor, ceil
import sys

def si5351_iqmode(Fclk):
    # There are no solution above 100 MHz without using integer mode,
    # which is not supported when phase shift is enabled.
    if Fclk < 1_400_000 or Fclk > 100_000_000:
        return None

    rdiv = 0

    Fxtal = 25_000_000
    Nmin, Nmax = 24, 36 # PLL should run between 600 Meg and 900 Meg
    Mmin, Mmax = 8, 1800

    dirty_hack = False
    if Fclk < 4_900_000:
        # dirty hack, run PLL below 600 MHz to cover 1.4 MHz .. 4.725 MHz range
        dirty_hack = True
        PhOff = 127
    elif Fclk < 8_000_000: # will find a solution for Fclk in 4.9..8 MHz range
        PhOff = floor(625_000_000 / Fclk)
    else:
        PhOff = floor(900_000_000 / Fclk)

    if PhOff > 127:
      print("Constraint violation: PhOff = {}".format(PhOff))
      return None

    Fpll = Fclk*PhOff
    if not dirty_hack and (Fpll < 600_000_000 or Fpll > 900_000_000):
        print("Constraint violation: Fpll = {}".format(Fpll))
        return None

    A = floor(Fpll / Fxtal)
    B = floor((Fpll % Fxtal) / 24)
    C = floor(Fxtal / 24)

    X = PhOff
    Y = 0
    Z = 1
 
    if (A < Nmin and not dirty_hack) or A > Nmax or X < Mmin or X > Mmax or (X == Mmin and Y == 0):
      print("Constraint violation: A = {}, X = {}, Y = {}".format(A, X, Y))
      return None

    if B > 0xFFFFF or C == 0 or C > 0xFFFFF or Y > 0xFFFFF or Z == 0 or Z > 0xFFFFF:
        print("Constraint violation: B = {}, C = {}, Y = {}, Z = {}".format(B, C, Y, Z))
        return None
        
    N = A+B/C
    M = X+Y/Z
    Fres = floor(Fxtal*N/(M * (1 << rdiv)))
    return { 'pll': {'a': A, 'b': B, 'c': C}, 'ms': {'a': X, 'b': Y, 'c': Z}, 'rdiv': rdiv, 'freq': Fres, 'phoff' : PhOff}

if __name__ == '__main__':
    step = 1
    max_err = 0
    min_phoff =  999
    max_phoff = -999
    for Fclk in range(1_400_000, 100_000_000+1, step):
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
