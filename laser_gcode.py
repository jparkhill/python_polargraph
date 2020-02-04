from math import sqrt, pow, cos, sin, pi
import copy, pickle, random, os

import numpy as np
from scipy import interpolate
from matplotlib import pyplot as plt

from PIL import Image
import imageio
import svgwrite
from svg_tools import path_bounds

def paths_bounds(paths):
    """
    returns xmin, ymin, xmax, ymax
    """
    if (not type(paths)==list):
        return [0.,0.,200.,200.]
    L = [path_bounds(X) for X in paths if len(X)>=2]
    if (len(L)==0):
        return [0.,0.,200.,200.]
    A = np.array(L)
    return A[:,:2].min(0).tolist()+A[:,2:].max(0).tolist()

def cymk_bounds(cymk):
        A=np.array([paths_bounds(cymk[0]),
        paths_bounds(cymk[1]),
        paths_bounds(cymk[2]),
        paths_bounds(cymk[3])])
        return A[:,:2].min(0).tolist()+A[:,2:].max(0).tolist()

preamble = """; Totem // SongShine // USongShine // MakeBlock Laserbox
; MKS DLC 2.0 Based Laser control test program.
; Written by hand with love, by john.parkhill@gmail.com
;
; setup
G90         ; Absolute positioning
G21         ; Set units to mm

$0=10      ; step pulse (micro s)
$1=25
$2=0        ; Don't invert X
$3=0
$4=0
$5=0
$6=0
$10=1
$11=0.010
$12=0.002
$13=0
$20=0
$21=0
$22=0 ; disable homing.
$23=0
$24=25.000
$25=200.000
$26=200.
$27=1.000
$30=1000 ; max spindle speed RPM
$31=0 ; max spindle speed RPM
$100=80.000 ; xsteps /mm
$101=80.000 ; ysteps /mm
$102=250.000; zsteps /mm
$110=300.000; x Max Rate mm/min
$111=300.000; y Max Rate mm/min
$112=200.000; z Max Rate mm/min
$120=3000.000; x accel (mm/sec^2)
$121=3000.000; y accel (mm/sec^2)
$122=10.000; z accel (mm/sec^2)
$130=200.000; x max travel (mm)
$131=200.000; y max travel (mm)
$132=200.000; z max travel (mm)

$32=1      ; enter GRBL 1.1 laser mode
M3           ; const mode
M4           ; dynamic mode
"""

def lines_to_gcode(paths, target_width = 180., rate=250.0,
                   min_travel = 0.25,
                   power=400., outfile = 'plot.gcode'):
    """
    Writes monochrome lines to a gcode with desired scaling.
    Flips X-axis because of the geometry of my plotter.
    """
    bnds = paths_bounds(paths)
    width = abs(bnds[2]-bnds[0])
    scale_factor = target_width/width
    # Reflect the x-coordinate first
    def map_x(x):
        return -1.*x*scale_factor
    def map_y(y):
        return (y-bnds[1])*scale_factor
    # Write the boundary size to screen.
    print("Plot Size (cm): {:f}x{:f}".format(abs(map_x(bnds[2])-map_x(bnds[0])), abs(map_y(bnds[3])-map_y(bnds[1]))))
    with open("bounds_"+outfile,'w') as f:
        for line in preamble:
            f.write(line)
        f.write("G1 X{:0.3f} Y{:0.3f} S0 F{:0.3f}\n".format(map_x(bnds[0]),
                                                            map_y(bnds[1]),rate))
        f.write("G1 X{:0.3f} Y{:0.3f} S{:0.3f} F{:0.3f}\n".format(map_x(bnds[2]),
                                                                  map_y(bnds[1]),
                                                                  power, rate))
        f.write("G1 X{:0.3f} Y{:0.3f} S{:0.3f} F{:0.3f}\n".format(map_x(bnds[2]),
                                                                  map_y(bnds[3]),
                                                                  power, rate))
        f.write("G1 X{:0.3f} Y{:0.3f} S{:0.3f} F{:0.3f}\n".format(map_x(bnds[0]),
                                                                  map_y(bnds[3]),
                                                                  power, rate))
        f.write("G1 X{:0.3f} Y{:0.3f} S{:0.3f} F{:0.3f}\n".format(map_x(bnds[0]),
                                                                  map_y(bnds[1]),
                                                                  power, rate))
        f.write("G1 X{:0.3f} Y{:0.3f} S0 F{:0.3f}\n".format(0,0,rate))
    with open(outfile,'w') as f:
        for line in preamble:
            f.write(line)
        for path in paths:
            if len(path) < 2:
                continue
            X,Y = path[0]
            xm = map_x(X)
            ym = map_y(Y)
            # print("Plot Starts At ",xm,ym)
            f.write("G1 X{:0.3f} Y{:0.3f} S0 F{:0.3f}\n".format(map_x(X), map_y(Y), rate))
            for i,p in enumerate(path[1:]):
                xm = map_x(X)
                ym = map_y(Y)
                p0m = map_x(p[0])
                p1m = map_y(p[1])
                if np.sqrt((xm-p0m)*(xm-p0m) + (ym-p1m)*(ym-p1m))<min_travel:
                    continue
                X,Y = p
                f.write("G1 X{:0.3f} Y{:0.3f} S{:0.3f} F{:0.3f}\n".format(map_x(X),
                                                            map_y(Y), power, rate))
        f.write("G1 X{:d} Y{:d} S0 F{:0.3f}\n".format(0,0,rate))
    return
