from math import sqrt, pow, cos, sin, pi
import copy, pickle, random, os

import numpy as np
from scipy import interpolate
from matplotlib import pyplot as plt

from PIL import Image
import imageio
import svgwrite

#
# Idea: projection of 3d paths.
# First make a wiggle, then z-project it.
# tilt then 2d project.
#
def lerp(p0,p1,npts = 600):
    a = np.linspace(0,1,npts)
    L=(1.-a[:,np.newaxis])*np.array(p0)[np.newaxis,:]+a[:,np.newaxis]*np.array(p1)[np.newaxis,:]
    return L.tolist()

def wiggle_fill(x_dim, y_dim, nwiggle = 80, npts=400):
    paths = []
    Xs = np.linspace(x_dim[0],x_dim[1],nwiggle)
    Ys = np.linspace(y_dim[0],y_dim[1],nwiggle)
    last_vertex = (Xs[0],Ys[0])
    d='r'
    N = 1
    while N < nwiggle-1:
        if (d == 'ld'):
            paths.extend(lerp(last_vertex, (Xs[0], Ys[N]),npts = npts))
            last_vertex = (Xs[0], Ys[N])
            d ='d'
        elif (d == 'd'):
            N+=1
            paths.extend(lerp(last_vertex, (Xs[0], Ys[N]),npts = npts))
            last_vertex = (Xs[0], Ys[N])
            d = 'ur'
        elif (d == 'ur'):
            paths.extend(lerp(last_vertex, (Xs[N], Ys[0]),npts = npts))
            last_vertex = (Xs[N], Ys[0])
            d = 'r'
        elif (d=='r'):
            N+=1
            paths.extend(lerp(last_vertex, (Xs[N], Ys[0]),npts = npts))
            last_vertex = (Xs[N], Ys[0])
            d = 'ld'
    N = 0
    d='ur'
    while N < nwiggle-1:
        if (d == 'ld'):
            paths.extend(lerp(last_vertex, (Xs[N], Ys[-1]),npts = npts))
            last_vertex = (Xs[N], Ys[-1])
            d ='r'
        elif (d == 'd'):
            N+=1
            paths.extend(lerp(last_vertex, (Xs[-1], Ys[N]),npts = npts))
            last_vertex = (Xs[-1], Ys[N])
            d = 'ld'
        elif (d == 'ur'):
            paths.extend(lerp(last_vertex, (Xs[-1], Ys[N]),npts = npts))
            last_vertex = (Xs[-1], Ys[N])
            d = 'd'
        elif (d=='r'):
            N+=1
            paths.extend(lerp(last_vertex, (Xs[N], Ys[-1]),npts = npts))
            last_vertex = (Xs[N], Ys[-1])
            d = 'ur'
    return paths

def smooth_path0(a_path, iters = 400):
    path = a_path.copy()
    for iter in range(iters):
        new_path = [path[0]]
        for I in range(1,len(path)-2):
            to_app = (new_path[-1]+path[I+1])/2.
            new_path.append(to_app)
        new_path.append(path[-1])
        path = np.stack(new_path,0)
    return path

def smooth_path(a_path, window = 400):
    to_app=[a_path[0]]
    for K in range(1,a_path.shape[0]):
        aslice = a_path[K-window:K]
        if len(aslice)==0:
            to_app.append(a_path[K])
        else:
            to_app.append(aslice.mean(0))
    return np.stack(to_app,0)

def path_channel_distort(path, F, magn=1.):
    """
    Moves points in a path per a channel...
    """
    new_path = []
    for vertex in path:
        z = F(vertex[1],vertex[0])
        new_path.append([vertex[0]+4*magn*z,vertex[1]+3*magn*z])
    return new_path

def floyd_steinberg(X, mx=255., alg = 'stucki'):
    """
    output is 1 or zero.
    """
    rank = len(X.shape)
    if (rank==2):
        out = np.zeros((X.shape[0]+4, X.shape[1]+4))
    elif (rank==3):
        out = np.zeros((X.shape[0]+4, X.shape[1]+4, X.shape[2]))
    errors = np.zeros(out.shape)
    xdim = X.shape[0]
    ydim = X.shape[1]
    if (alg == 'stucki'):
        error_array = np.array([[0.,0.,0,8,4],[2.,4.,8,4,2],[1.,2.,4,2,1]])/42.
    else:
        error_array = np.array([[0.,0.,0,1,1],[0.,1.,1,1,0],[0.,0.,1,0,0]])/8.
    for y in range(2,ydim+2):
        for x in range(2,xdim+2):
            div = X[x-2,y-2].astype(np.float64)/mx + errors[x,y]
            output = np.round(div)
            error = div - output
            out[x,y] = output
            if (rank == 2):
                errors[x-2:x+3,y:y+3] += error * error_array.T
            else:
                errors[x-2:x+3,y:y+3] += error * error_array[:,:,np.newaxis].transpose(1,0,2)
    return out[2:-2,2:-2]

def rgb_to_cmyk(X, RGB_SCALE = 255):
    """
    Args:
        X: an X,Y, RGB rasterized image. 
    """
    CMYK_SCALE = 1.
    r = X[:,:,0].astype(np.float64)
    g = X[:,:,1].astype(np.float64)
    b = X[:,:,2].astype(np.float64)
    # rgb [0,255] -> cmy [0,1]
    c = 1. - r / RGB_SCALE
    m = 1. - g / RGB_SCALE
    y = 1. - b / RGB_SCALE
    # extract out k [0, 1]
    #     min_cmy = min(c, m, y)
    min_cmy = np.minimum(c, m)
    min_cmy = np.minimum(min_cmy, y)
    k = np.where(r+g+b == 0, 1., min_cmy)
    c = (c - min_cmy) / (1. - min_cmy+1e-9)
    m = (m - min_cmy) / (1. - min_cmy+1e-9)
    y = (y - min_cmy) / (1. - min_cmy+1e-9)
    # rescale to the range [0,CMYK_SCALE]
    return np.stack([c * CMYK_SCALE, y * CMYK_SCALE, m * CMYK_SCALE,  k * CMYK_SCALE], -1)

## Render a dithered image as polygons
def ngon(X=0, Y=0, r=1, n=6, phase = 0, closed = True):
    pts = []
    step = 2*pi/n
    for K in range(n):
        pts.append([X+r*cos(K*step+phase),
                    Y+r*sin(K*step+phase)])
    if closed:
        pts.append([X+r*cos(phase),
                    Y+r*sin(phase)])
    return pts

def random_ngon_linify(cymk_img, n_min=3, n_max=6, r_min = .1, r_max = 1.):
    """
    Draws ngons over a previously dithered image.
    """
    c_lines, y_lines, m_lines, k_lines = [],[],[],[]
    for X in range(cymk_img.shape[0]):
        for Y in range(cymk_img.shape[1]):
            if (cymk_img[X,Y,0]>0.5):
                R = random.random()*(r_max-r_min) + r_min
                NN = random.randint(n_min,n_max)
                phase = random.random()*2*pi
                c_lines.append(ngon(X,Y,r=R,n=NN,phase=phase))
            if (cymk_img[X,Y,1]>0.5):
                R = random.random()*(r_max-r_min) + r_min
                NN = random.randint(n_min,n_max)
                phase = random.random()*2*pi
                y_lines.append(ngon(X,Y,r=R,n=NN,phase=phase))
            if (cymk_img[X,Y,2]>0.5):
                R = random.random()*(r_max-r_min) + r_min
                NN = random.randint(n_min,n_max)
                phase = random.random()*2*pi
                m_lines.append(ngon(X,Y,r=R,n=NN,phase=phase))
            if (cymk_img[X,Y,3]>0.5):
                R = random.random()*(r_max-r_min) + r_min
                NN = random.randint(n_min,n_max)
                phase = random.random()*2*pi
                k_lines.append(ngon(X,Y,r=R,n=NN,phase=phase))
    return c_lines, y_lines, m_lines, k_lines

def write_svg(lines, outname="dump", scale = 1.0):
    dwg = svgwrite.Drawing(outname+'.svg')
    for line in lines:
        if (len(line)<2):
            continue
        for I in range(len(line)-1):
            dwg.add(dwg.line(line[I][::-1], line[I+1][::-1], stroke=svgwrite.rgb(0, 0, 0, '%')))
    dwg.save()

def cymk_to_svg(lines, outname="dump", scale = 1.0):
    dwg = svgwrite.Drawing(outname+'.svg')
    for line in lines[0]:
        if (len(line)<2):
            continue
        for I in range(len(line)-1):
            dwg.add(dwg.line((scale*line[I][1],scale*line[I][0]),
                             (scale*line[I+1][1],scale*line[I+1][0]),
                             stroke=svgwrite.rgb(0, 100, 100, '%')))
    for line in lines[1]:
        if (len(line)<2):
            continue
        for I in range(len(line)-1):
            dwg.add(dwg.line((scale*line[I][1],scale*line[I][0]),
                            (scale*line[I+1][1],scale*line[I+1][0]),
                            stroke=svgwrite.rgb(100, 100, 0, '%')))
    for line in lines[2]:
        if (len(line)<2):
            continue
        for I in range(len(line)-1):
            dwg.add(dwg.line((scale*line[I][1],scale*line[I][0]),
                            (scale*line[I+1][1],scale*line[I+1][0]),
                             stroke=svgwrite.rgb(100, 0, 100, '%')))
    for line in lines[3]:
        if (len(line)<2):
            continue
        for I in range(len(line)-1):
            dwg.add(dwg.line((scale*line[I][1],scale*line[I][0]),
                            (scale*line[I+1][1],scale*line[I+1][0]),
                             stroke=svgwrite.rgb(0, 0, 0, '%')))
    dwg.save()

def grid_lineify(f, x_lim=(0.,256) ,y_lim=(0.,256), ntraj = 600,
            max_step = 3000, gamma = 0.02, dt = 9., e0 = 0.1,
            T = 0.1,
            e_thresh = 0.001, h = 2e-1, m = 3, bounce = False
           ):
    """
    Units here are going to be pixel/sec.
    """
    lines = []
    nx = int(np.sqrt(ntraj))
    x_starts, y_starts = np.meshgrid(np.linspace(x_lim[0],x_lim[1],nx),
                                     np.linspace(y_lim[0],y_lim[1],nx))
    x_starts = x_starts.flatten()
    y_starts = y_starts.flatten()
    for traj in range(len(x_starts)):
        x,y = x_starts[traj].item(), y_starts[traj].item()
        PE = f(x, y)
        v0 = np.sqrt(e0/m)
        vx,vy = np.random.normal(0,v0), np.random.normal(0,v0)
        line = []
        step = 0
        while step < max_step and np.sqrt(vx*vx+vy*vy) > e_thresh:
            PE = f(x, y)
            if (np.exp(-PE/.01) > np.random.random()):
                break
            # cdiff grad
            gx = ((f(x+h,y)-f(x-h,y))/(2*h)).item()
            gy = ((f(x,y+h)-f(x,y-h))/(2*h)).item()
            vx += 0.5*dt*(gx - gamma*vx + np.random.normal(0,np.sqrt(gamma*e0))  )/m
            vy += 0.5*dt*(gy - gamma*vy + np.random.normal(0,np.sqrt(gamma*e0))  )/m
            x += vx*dt
            y += vy*dt
            # Bounce off edges.
            if (bounce):
                if (x > x_lim[1]):
                    x -= 2.0*np.abs(x-x_lim[1])
                    vx *= -1
                if (x < x_lim[0]):
                    x += 2.0*np.abs(x-x_lim[0])
                    vx *= -1
                if (y > y_lim[1]):
                    y -= 2.0*np.abs(y-y_lim[1])
                    vy *= -1
                if (y < y_lim[0]):
                    y += 2.0*np.abs(y-y_lim[0])
                    vy *= -1
            else: # absorb
                if (x > x_lim[1]):
                    break
                elif (x < x_lim[0]):
                    break
                elif (y > y_lim[1]):
                    break
                elif (y < y_lim[0]):
                    break
            line.append([x,y])
            gx = ((f(x+h,y)-f(x-h,y))/(2*h)).item()
            gy = ((f(x,y+h)-f(x,y-h))/(2*h)).item()
            vx += 0.5*dt*(gx - gamma*vx + np.random.normal(0,np.sqrt(gamma*e0))  )/m
            vy += 0.5*dt*(gy - gamma*vy + np.random.normal(0,np.sqrt(gamma*e0))  )/m
            step += 1
        lines.append(line)
    return lines
