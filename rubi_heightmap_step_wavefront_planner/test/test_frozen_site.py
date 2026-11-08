"""File integrity and immutable cell geometry, independent of a developer's maps."""
import hashlib
from pathlib import Path
import struct
import sys
import pytest
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from frozen_site import load_site, read_height


def fixture_site(tmp_path):
    (tmp_path/'height.rghm').write_text('RUBI_GLOBAL_HEIGHTMAP 1\n"map" 0.05 -1 -2 2 2 0.1 2000000000 2\n0 0.05 0.01 1000000000 1\n3 0.12 0.01 1000000000 0\n')
    (tmp_path/'occupancy.yaml').write_text('image: floor.pgm\nresolution: 0.05\n')
    (tmp_path/'floor.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    (tmp_path/'localization.pcd').write_text('fixture')
    assets = {}
    for key, name in [('height','height.rghm'), ('occupancy','occupancy.yaml'),
                      ('image','floor.pgm'), ('pcd','localization.pcd')]:
        assets[key] = dict(path=name, sha256=hashlib.sha256((tmp_path/name).read_bytes()).hexdigest())
    site = dict(format=1, id='fixture', frame='map', height_resolution=.05,
                alignment={'verified': True}, assets=assets)
    (tmp_path/'site.yaml').write_text(yaml.safe_dump(site))
    return tmp_path/'site.yaml'


def test_cells_acquisition_and_unknown_mask(tmp_path):
    site = fixture_site(tmp_path)
    manifest, xyz = load_site(site)
    assert manifest['height']['acquisition_stamp_ns'] == 2000000000
    assert manifest['height']['usable_cells'] == 1
    assert struct.unpack('<fff', xyz) == pytest.approx((-.975, -1.975, .05))


def test_tampered_asset_rejected(tmp_path):
    site = fixture_site(tmp_path)
    (tmp_path/'height.rghm').write_text('replacement')
    with pytest.raises(ValueError, match='checksum mismatch'):
        load_site(site)


def test_duplicate_cells_rejected(tmp_path):
    fixture_site(tmp_path)
    file = tmp_path/'height.rghm'
    file.write_text(file.read_text().replace('3 0.12', '0 0.12'))
    with pytest.raises(ValueError, match='duplicated'):
        read_height(file)
