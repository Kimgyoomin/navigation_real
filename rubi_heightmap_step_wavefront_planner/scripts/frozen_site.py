"""Read a versioned site and RGHM v1 file without modifying or re-dating it."""
import hashlib
import math
from pathlib import Path
import shlex
import struct
import yaml


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_height(filename):
    path = Path(filename).expanduser().resolve()
    payload = path.read_bytes()
    lines = payload.decode('utf-8').splitlines()
    if len(lines) < 2 or lines[0] != 'RUBI_GLOBAL_HEIGHTMAP 1':
        raise ValueError('Expected RUBI_GLOBAL_HEIGHTMAP 1')
    header = shlex.split(lines[1])
    if len(header) != 9:
        raise ValueError('Malformed RGHM header')
    frame = header[0]
    resolution, ox, oy = map(float, header[1:4])
    nx, ny = map(int, header[4:6])
    cutoff, stamp, count = float(header[6]), int(header[7]), int(header[8])
    if (not frame or not all(map(math.isfinite, (resolution, ox, oy, cutoff)))
            or resolution <= 0 or cutoff < 0 or nx < 1 or ny < 1 or nx*ny > 25_000_000
            or not 0 <= stamp < 2**63 or not 0 <= count <= nx*ny or len(lines) != count+2):
        raise ValueError('Invalid RGHM dimensions, time or count')
    points = bytearray()
    seen = set()
    for line in lines[2:]:
        fields = line.split()
        if len(fields) != 5:
            raise ValueError('Malformed RGHM cell')
        index, z, variance, t, usable = (int(fields[0]), float(fields[1]),
            float(fields[2]), int(fields[3]), int(fields[4]))
        if (index in seen or not 0 <= index < nx*ny or not math.isfinite(z)
                or not math.isfinite(variance) or not 0 <= variance <= cutoff
                or not 0 < t <= stamp or usable not in (0, 1)):
            raise ValueError('Invalid or duplicated RGHM cell')
        seen.add(index)
        if usable:
            x = ox+(index % nx+.5)*resolution
            y = oy+(index // nx+.5)*resolution
            points.extend(struct.pack('<fff', x, y, z))
    if not points:
        raise ValueError('No usable height cells')
    metadata = dict(path=str(path), sha256=hashlib.sha256(payload).hexdigest(),
                    frame=frame, resolution_m=resolution, acquisition_stamp_ns=stamp,
                    usable_cells=len(points)//12, dimensions=[nx, ny])
    return metadata, bytes(points)


def load_site(filename):
    path = Path(filename).expanduser().resolve()
    site = yaml.safe_load(path.read_text())
    if site.get('format') != 1 or site.get('frame') != 'map':
        raise ValueError('Expected site format 1, frame map')
    if site.get('alignment', {}).get('verified') is not True:
        raise ValueError('Site has no recorded alignment review')
    assets = {}
    for key in ('pcd', 'height', 'occupancy', 'image'):
        item = site['assets'][key]
        file = (path.parent/item['path']).resolve()
        sha = digest(file)
        if sha != item['sha256']:
            raise ValueError(f'{key} checksum mismatch: {file}')
        assets[key] = dict(path=str(file), sha256=sha)
    occ_path = Path(assets['occupancy']['path'])
    occ = yaml.safe_load(occ_path.read_text())
    if (occ_path.parent/occ['image']).resolve() != Path(assets['image']['path']):
        raise ValueError('Occupancy YAML does not reference the verified image')
    height, points = read_height(assets['height']['path'])
    if height['frame'] != site['frame'] or abs(height['resolution_m']-.05) > 1e-9:
        raise ValueError('This experiment requires map-frame height cells at 0.05 m')
    if abs(height['resolution_m']-site['height_resolution']) > 1e-9:
        raise ValueError('Site resolution differs from RGHM resolution')
    manifest = dict(site_file=str(path), site_sha256=digest(path), site_id=site['id'],
                    assets=assets, height=height, alignment=site['alignment'],
                    note='Frozen data; recorded alignment review is not a new accuracy certification.')
    return manifest, points
