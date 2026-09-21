"""Explicit unit conversion and rigid registration of pipeline CSVs; no ROS."""
import argparse
import csv
import math
from pathlib import Path


def multiply(a, b):
    x,y,z,w=a; X,Y,Z,W=b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input'); parser.add_argument('output')
    parser.add_argument('--units', required=True, choices=['mm','m'])
    parser.add_argument('--xyz', nargs=3, type=float, required=True,
                        help='Measured object-to-world translation in meters')
    parser.add_argument('--rpy-deg', nargs=3, type=float, required=True,
                        help='Measured object-to-world fixed-axis roll pitch yaw')
    a=parser.parse_args()
    if Path(a.input).resolve()==Path(a.output).resolve():
        parser.error('Use a separate output path')
    if not all(math.isfinite(v) for v in a.xyz+a.rpy_deg):
        parser.error('Nonfinite transform')
    r,p,y=[math.radians(v)/2 for v in a.rpy_deg]
    q=multiply(multiply((0,0,math.sin(y),math.cos(y)),
                       (0,math.sin(p),0,math.cos(p))), (math.sin(r),0,0,math.cos(r)))
    inv=(-q[0],-q[1],-q[2],q[3]); scale=.001 if a.units=='mm' else 1.
    output=[]
    with open(a.input) as f:
        for row in csv.DictReader(f):
            v=[float(row[k])*scale for k in ('x','y','z')]
            orientation=[float(row[k]) for k in ('qx','qy','qz','qw')]
            if not all(math.isfinite(x) for x in v+orientation):
                raise ValueError('Nonfinite CSV pose')
            if abs(sum(x*x for x in orientation)-1)>0.02:
                raise ValueError('Nonunit quaternion')
            v=multiply(multiply(q, (*v,0)),inv)[:3]
            output.append([int(row.get('line_id',0)),
                           *[v[i]+a.xyz[i] for i in range(3)], *multiply(q,orientation)])
    if not output: raise ValueError('Empty input')
    with open(a.output,'w',newline='') as f:
        w=csv.writer(f); w.writerow(['line_id','x','y','z','qx','qy','qz','qw']);w.writerows(output)
    print(f'{len(output)} poses in meters written to {a.output}; review the registration in RViz')
