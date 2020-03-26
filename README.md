### OVERVIEW

This is a prototype implementation of the Bezier Guarding algorithm.

It furthermore includes various algorithmic steps that attempt to
verify the validity of the provided input domain curves before the
actual Bezier Guarding algorithm is started. This includes a step
that, in the case of intersecting input (which violates the input
assumptions) attempts to resolve these intersections in the input
by splitting the intersecting curves at the points of intersection.
Note that these prechecking and preprocessing steps are not
guaranteed to succeed in all cases, but they give valuable hints
in case some input violates the assumptions.

The code includes the EXACT as well as th FLOAT variant of the algo-
rithm. It can be configured as either by setting the ALL_EXACT flag
to True or False in CMake.

The input is expected in a simple .json format (see the included
example data). In the end the algorithm reports success or failure
and optionally (see the command line parameter description below)
outputs the generated higher-order mesh in .msh format and/or a vis-
ualization in .eps format. The code does not perform optimization
of the generated mesh, it is output in its initial regular state.
Note that these output formats require rounding to standard limited
precision, which may cause numerical degeneracies.


### HOW TO USE

## Requirements

-# CMake (v >= 3.1)
-# GMP library
-# GMPXX library
-# CGAL (only for EXACT version)

## Building

In project root directory:
-# mkdir build
-# cd build
-# cmake .. -DALL_EXACT=True (to build the EXACT version)
-# OR: cmake .. -DALL_EXACT=False (to build the FLOAT version)
-# make

## Running

Need to specify 1 (optionally 2) command line parameters
  -# path to the input file in .json format
  -# string with char-flags:
     -# 'b' to add bounding box
     -# 'e' to enable eps output
     -# 'g' to enable gmsh output
In directory containing binary "bezmeshCLI", e.g.:
./bezmeshCLI "path/to/your/curve_file.json" "be"
