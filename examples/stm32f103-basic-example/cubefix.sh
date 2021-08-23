#!/bin/sh

set -e

dos2unix Makefile
dos2unix Src/main.c
dos2unix Src/stm32f1xx_it.c
