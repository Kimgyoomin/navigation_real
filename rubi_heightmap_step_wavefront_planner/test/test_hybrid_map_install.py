from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


PACKAGE = 'rubi_heightmap_step_wavefront_planner'
SOURCE_MAPS = Path(__file__).resolve().parents[1] / 'maps'


def _assert_map_contract(map_dir):
    yaml_path = map_dir / 'RUBI_occupancy_map.yaml'
    image_path = map_dir / 'RUBI_occupancy_map.pgm'
    assert yaml_path.is_file()
    assert image_path.is_file()
    document = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    assert document['image'] == 'RUBI_occupancy_map.pgm'
    assert document['resolution'] == 0.05
    assert document['origin'] == [-2.65, -12.90, 0.0]
    assert (yaml_path.parent / document['image']).resolve() == image_path.resolve()


def test_source_map_contract():
    _assert_map_contract(SOURCE_MAPS)


def test_installed_map_contract():
    share = Path(get_package_share_directory(PACKAGE))
    _assert_map_contract(share / 'maps')
