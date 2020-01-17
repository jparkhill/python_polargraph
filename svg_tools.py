"""
svg_to_paths()
Can parse basic SVG's and generate coordinate paths
to fill them appropriately. It generates simple
[[[[c_coord_1],[c_coord_2]], [c_path_2,... ] ... ] , ... ]
cymk plotter paths.

Those paths can be made for example in using https://picsvg.com
"""

import os, re
from math import sqrt, pow, cos, sin, pi
import copy, pickle, random
from xml.dom import minidom
import numpy as np
from scipy import interpolate
import imageio
from PIL import Image
import svgwrite
from lineifiers import *

def scale_xform(X,scl = 1.0):
    return [scl*X[0], scl*X[1]]

def ident_xform(X):
    return [X[0],X[1]]

def parse_transform(t):
    if 'scale' in t:
        scl = float(t.replace('scale(','').replace(')',''))
        tore = lambda X: scale_xform(X, scl = scl)
    elif 'matrix' in t:
        meles = t.replace('matrix(','').replace(')','').replace(',',' ').split(' ')
        a,b,c,d,e,f = list(map(float,meles))
        return lambda X: [a*X[0]+c*X[1]+e, b*X[0]+d*X[1]+f]
    else:
        tore = ident_xform
    return tore

def parse_fill(S):
    """
    Returns a CYMK (0,1) color intensity
    """
    fi = S.index('#')
    hexes = S[fi+1:fi+1+6]
    R = eval('0x'+hexes[:2])
    G = eval('0x'+hexes[2:4])
    B = eval('0x'+hexes[4:])
    c = 1. - R/255
    m = 1. - G/255
    y = 1. - B/255
    min_cmy = min(c,m,y)
    if (R==0 and G==0 and B==0):
        return 0., 0., 0., 1.
    else:
        k = min_cmy
    c = (c - min_cmy) / (1. - min_cmy+1e-9)
    m = (m - min_cmy) / (1. - min_cmy+1e-9)
    y = (y - min_cmy) / (1. - min_cmy+1e-9)
    return c,m,y,k

def parse_group_into_lines(g, lines, x_form_ = ident_xform,
                        fill_style ='outline', fill_color=[0,0,0,0]):
    if ('transform' in g.attributes):
        XF = parse_transform(g.attributes['transform'].value)
        x_form = lambda X: XF(x_form_(X))
    else:
        x_form = x_form_
    if ('style' in g.attributes and not fill_style is 'outline'):
        fill_color = parse_fill(g.attributes['style'].value)
        fill_style = 'hatch'
    elif ('fill' in g.attributes and not fill_style is 'outline'):
        fill_color = parse_fill(g.attributes['fill'].value)
        fill_style = 'hatch'
    for child in g.childNodes:
        if (child.nodeName=='g'):
            parse_group_into_lines(child, lines, x_form,
                                    fill_style=fill_style,
                                    fill_color=fill_color)
        elif (child.nodeName=='path'):
            parse_path_into_lines(child, lines, x_form,
                                    fill_style=fill_style,
                                    fill_color=fill_color)

def path_bounds(path):
    A = np.array(path)
    if (len(A.shape) != 2):
        print(A.shape)
        raise Exception("Bad Path")
    if (A.shape[1]!=2):
        print(A.shape)
        raise Exception("Bad Path")
    return A.min(0).tolist()+A.max(0).tolist()

def paths_bounds(paths):
    L = [path_bounds(X) for X in paths if len(X)>=2]
    A = np.array(L)
    return A[:,:2].min(0).tolist()+A[:,2:].max(0).tolist()

def interior_hatches(a_path, ys):
    tore = []
    for y in ys:
        xs = []
        for I,s in enumerate(a_path[:-1]):
            s2 = a_path[I+1]
            YS = sorted([s[1],s2[1]])
            if y >= YS[0] and y <= YS[1]:
                rise = (s2[1]-s[1])
                run = (s2[0]-s[0])
                if (rise == 0):
                    continue
                if (run == 0):
                    xs.append(s[0])
                    continue
                slope = rise/run
                int  = s[1] - slope*s[0]
                x = (y-int)/slope
                xs.append(x)
        # Now sort the x's
        SX = sorted(xs)
        if (len(SX)%2==1):
            print("WARNING ODD INT")
        NS = len(SX)//2
        for I in range(NS):
            tore.append([[SX[2*I],y],[SX[2*I+1],y]])
    return tore

def interior_hatches_paths(paths, ys):
    tore = []
    for y in ys:
        xs = []
        for a_path in paths:
            for I,s in enumerate(a_path[:-1]):
                s2 = a_path[I+1]
                YS = sorted([s[1],s2[1]])
                if y >= YS[0] and y <= YS[1]:
                    rise = (s2[1]-s[1])
                    run = (s2[0]-s[0])
                    if (rise == 0):
                        continue
                    if (run == 0):
                        xs.append(s[0])
                        continue
                    slope = rise/run
                    int  = s[1] - slope*s[0]
                    x = (y-int)/slope
                    xs.append(x)
        # Now sort the x's
        SX = sorted(xs)
        if (len(SX)%2==1):
            print("WARNING ODD INT")
        NS = len(SX)//2
        for I in range(NS):
            tore.append([[SX[2*I],y],[SX[2*I+1],y]])
    return tore

def hatch_paths_within_path(a_path, cymk, linewidth=2., slope = 0.):
    """
    Creates horiz hatches in a curve.

    Args:
        a_path: a list of coordinate tuples
        cymk: sequence-like of c,m,y,k 0-1 weights.
    Returns:
        [c_paths, ... ]
        which you could append to cymk to get the desired hatching.
    """
    xmin, ymin, xmax, ymax = path_bounds(a_path)
    ydist = ymax - ymin
    nc = ydist*cymk[0]/linewidth
    ny = ydist*cymk[1]/linewidth
    nm = ydist*cymk[2]/linewidth
    nk = ydist*cymk[3]/linewidth
    cys = np.linspace(ymin,ymax,nc)
    chs = interior_hatches(a_path, cys)
    yys = np.linspace(ymin,ymax,ny)
    yhs = interior_hatches(a_path, yys)
    mys = np.linspace(ymin,ymax,nm)
    mhs = interior_hatches(a_path, mys)
    kys = np.linspace(ymin,ymax,nk)
    khs = interior_hatches(a_path, kys)
    return [chs,yhs,mhs,khs]

def hatch_paths_within_paths(paths, cymk, linewidth=4., slope = 0.):
    """
    Creates horiz hatches in a curve.

    Args:
        a_path: a list of coordinate tuples
        cymk: sequence-like of c,m,y,k 0-1 weights.
    Returns:
        [c_paths, ... ]
        which you could append to cymk to get the desired hatching.
    """
    if len(paths)<1:
        return [[],[],[],[]]
    xmin, ymin, xmax, ymax = paths_bounds(paths)
    ydist = ymax - ymin
    nc = ydist*cymk[0]/linewidth
    ny = ydist*cymk[1]/linewidth
    nm = ydist*cymk[2]/linewidth
    nk = ydist*cymk[3]/linewidth
    cys = np.linspace(ymin,ymax,nc)
    chs = interior_hatches_paths(paths, cys)
    yys = np.linspace(ymin,ymax,ny)
    yhs = interior_hatches_paths(paths, yys)
    mys = np.linspace(ymin,ymax,nm)
    mhs = interior_hatches_paths(paths, mys)
    kys = np.linspace(ymin,ymax,nk)
    khs = interior_hatches_paths(paths, kys)
    return [chs,yhs,mhs,khs]

def parse_path_into_lines(a_path_, lines, x_form_ = ident_xform,
                                   fill_style ='outline', fill_color=[0,0,0,0],
                                   X=0 , Y=0, bezier_steps = 20 ):
    """
    Parses most of the commands found in an SVG path.
    """
    # Check for any xform of the path
    if(not 'd' in a_path_.attributes):
        return
    else:
        a_path = a_path_.attributes['d'].value
    # Get any required transformations.
    if ('style' in a_path_.attributes):
        fill_color = parse_fill(a_path_.attributes['style'].value)
        fill_style = 'hatch'
    else:
        fill_color=[0.,0.,0.,1.]
    if ('transform' in a_path_.attributes):
        XF = parse_transform(a_path_.attributes['transform'].value)
        x_form = lambda X: XF(x_form_(X))
    else:
        x_form = x_form_
    X0, Y0 = X,Y
    # TODO deal with contiguous tokens.
    # insert spaces after any char if needed.
    p = re.compile("([mcMClLzcChHvV ,])")
    tokens=[X for X in p.split(a_path) if len(X)>0 and not X in [' ',',']]

    out_paths = []
    current_path = []
    token = None
    while (len(tokens)>0):
        token = tokens.pop(0)

        #TODO parse E

        if (token == 'm'):
            if (len(current_path)>0):
                out_paths.append(copy.copy(current_path))
                current_path = []
            while (len(tokens)>0):
                if tokens[0].isalpha():
                    break
                X += float(tokens.pop(0))
                Y += float(tokens.pop(0))
            X0,Y0 = X,Y
            continue
        elif (token == 'M'):
            if (len(current_path)>0):
                out_paths.append(copy.copy(current_path))
                current_path = []
            X = float(tokens.pop(0))
            Y = float(tokens.pop(0))
            X0,Y0 = X,Y
            continue
        elif (token == 'l'):
            while (len(tokens)>0):
                if tokens[0].isalpha():
                    break
                # Simple line
                current_path.append(x_form([X,Y]))
                X += float(tokens.pop(0))
                Y += float(tokens.pop(0))
                current_path.append(x_form([X,Y]))
        elif (token == 'L'):
            while (len(tokens)>0):
                if tokens[0].isalpha():
                    break
                # Simple line
                current_path.append(x_form([X,Y]))
                X = float(tokens.pop(0))
                Y = float(tokens.pop(0))
                current_path.append(x_form([X,Y]))
        elif (token == 'z'):
            # close the contour
            current_path.append(x_form([X,Y]))
            current_path.append(x_form([X0,Y0]))
            X,Y = X0, Y0
            if (len(current_path)>0):
                out_paths.append(copy.copy(current_path))
                current_path = []
        elif (token == 'h'):
            # close the contour
            current_path.append(x_form([X,Y]))
            X += float(tokens.pop(0))
            current_path.append(x_form([X,Y]))
        elif (token == 'v'):
            # close the contour
            current_path.append(x_form([X,Y]))
            Y += float(tokens.pop(0))
            current_path.append(x_form([X,Y]))
        elif (token == 'C'):
            try:
                while (len(tokens)>0):
                    if tokens[0].isalpha():
                        break
                    # Simple line
                    current_path.append(x_form([X,Y]))
                    p0x = float(tokens.pop(0))
                    p0y = float(tokens.pop(0))
                    C1x = float(tokens.pop(0))
                    C1y = float(tokens.pop(0))
                    C2x = X = float(tokens.pop(0))
                    C2y = Y = float(tokens.pop(0))
                    for t in [1.*X/bezier_steps for X in range(1,bezier_steps+1)]:
                        omt = 1.-t
                        Xp = pow(omt,3)*p0x + 3*pow(omt,2.)*t*C1x + 3*pow(t,2.)*omt*C2x+pow(t,3)*X
                        Yp = pow(omt,3)*p0y + 3*pow(omt,2.)*t*C1y + 3*pow(t,2.)*omt*C2y+pow(t,3)*Y
                        current_path.append(x_form([Xp,Yp]))
            except Exception as Ex:
                print(token)
                print(tokens)
                raise Ex
        elif (token == 'c'):
            try:
                while (len(tokens)>0):
                    if tokens[0].isalpha():
                        break
                    # Simple line
                    current_path.append(x_form([X,Y]))
                    p0x = X
                    p0y = Y
                    C1x = X + float(tokens.pop(0))
                    C1y = Y + float(tokens.pop(0))
                    C2x = X + float(tokens.pop(0))
                    C2y = Y + float(tokens.pop(0))
                    X += float(tokens.pop(0))
                    Y += float(tokens.pop(0))
                    for t in [1.*X/bezier_steps for X in range(1,bezier_steps+1)]:
                        omt = 1.-t
                        Xp = pow(omt,3)*p0x + 3*pow(omt,2.)*t*C1x + 3*pow(t,2.)*omt*C2x+pow(t,3)*X
                        Yp = pow(omt,3)*p0y + 3*pow(omt,2.)*t*C1y + 3*pow(t,2.)*omt*C2y+pow(t,3)*Y
                        current_path.append(x_form([Xp,Yp]))
            except Exception as Ex:
                print(token)
                print(tokens)
                raise Ex
        else:
            print('BAD TOKEN', token)
            raise Exception('Bad Token')
    if (len(current_path)>0):
        out_paths.append(copy.copy(current_path))
    if (fill_style=='hatch'):
        c_hs, y_hs, m_hs, k_hs = hatch_paths_within_paths(out_paths, fill_color)
        lines[0].extend(c_hs)
        lines[1].extend(y_hs)
        lines[2].extend(m_hs)
        lines[3].extend(k_hs)
        if (fill_color[-1] > .99):
            lines[-1].extend(out_paths)
    else:
        return lines[-1].extend(out_paths)

def svg_to_paths(filename = 'drawing.svg', fill_style = 'outline'):
    """
    Args:
        filename: an svg filename
        fill_style: 'outline', None
    Returns:
        [cpaths, ypaths... ]
        where cpaths is a list of lists of coordinate pairs [[[x,y]]]
    """
    tore = [[],[],[],[]] # Cymk lines.
    xmldoc = minidom.parse(filename)
    itemlist = xmldoc.getElementsByTagName('path')
    print("Converting", len(itemlist),"paths")
    lines = [[],[],[],[]]
    for child in xmldoc.childNodes[0].childNodes:
        if (child.nodeName=='g'):
            parse_group_into_lines(child, lines, x_form_ = ident_xform,
                                    fill_style =fill_style, fill_color=[0,0,0,0])
        elif (child.nodeName=='path'):
            parse_path_into_lines(child, lines, x_form_ = ident_xform,
                                    fill_style =fill_style, fill_color=[0,0,0,0])
    return lines
