"""FX450 플레이트 STEP을 Rx(+90)로 회전: (x,y,z) -> (x,-z,y)
CARTESIAN_POINT와 DIRECTION의 좌표만 변환 - 엔티티 순서/개수 불변(면 번호 보존).
"""
import re
import sys

PAT = re.compile(
    r"(CARTESIAN_POINT|DIRECTION)(\s*\(\s*'[^']*'\s*,\s*\(\s*)"
    r"([-0-9.Ee+]+)\s*,\s*([-0-9.Ee+]+)\s*,\s*([-0-9.Ee+]+)(\s*\)\s*\))"
)

def fmt(v):
    s = f"{v:.12G}"
    if '.' not in s and 'E' not in s and 'e' not in s:
        s += '.'
    return s

def rot(m):
    x, y, z = float(m.group(3)), float(m.group(4)), float(m.group(5))
    # Rx(+90): (x, y, z) -> (x, -z, y)
    nx, ny, nz = x, -z, y
    return f"{m.group(1)}{m.group(2)}{fmt(nx)},{fmt(ny)},{fmt(nz)}{m.group(6)}"

def process(path):
    with open(path, 'r', errors='ignore') as f:
        txt = f.read()
    new, n = PAT.subn(rot, txt)
    with open(path, 'w') as f:
        f.write(new)
    print(f"{path}: {n}개 엔티티 회전 완료")

cad = r"c:\Users\psjqk\OneDrive\Desktop\디지\프로젝트별\UGRP\workload\control_seoungjin\controller\Quadcopter-Drone-Model-Simscape\CAD\Geometry"
process(cad + r"\quadcopter_drone_plate_top.stp")
process(cad + r"\quadcopter_drone_plate_bottom.stp")
