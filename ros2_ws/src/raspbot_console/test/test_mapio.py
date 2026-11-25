"""mapio 纯逻辑单元测试（离线，不依赖 rclpy）。"""

import os
import struct
import sys
import tempfile
import unittest
import zlib

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
import support  # noqa: E402

from raspbot_console import mapio  # noqa: E402


def _grid(width=3, height=2, data=None):
    if data is None:
        # 行 0 = 最小 y，行 1 = 最大 y
        data = [0, -1, 100, 50, 100, 0]
    return mapio.make_map_grid(width, height, 0.05, (-2.4, -2.0, 0.0), data,
                               frame_id='map', stamp=1764012345.2)


class GrayMappingTests(unittest.TestCase):

    def test_known_values(self):
        self.assertEqual(mapio.occupancy_gray(-1), 205)
        self.assertEqual(mapio.occupancy_gray(0), 254)
        self.assertEqual(mapio.occupancy_gray(100), 0)

    def test_linear_middle_and_clamp(self):
        self.assertEqual(mapio.occupancy_gray(50), 127)
        self.assertEqual(mapio.occupancy_gray(65), 89)
        self.assertEqual(mapio.occupancy_gray(200), 0)
        self.assertEqual(mapio.occupancy_gray(-5), 205)


class PngTests(unittest.TestCase):

    def test_encode_decode_roundtrip(self):
        rows = [bytes([0, 127, 255]), bytes([10, 20, 30])]
        png = mapio.encode_png_gray8(3, 2, rows)
        self.assertTrue(png.startswith(b'\x89PNG\r\n\x1a\n'))
        width, height, decoded = mapio.decode_png_gray8(png)
        self.assertEqual((width, height), (3, 2))
        self.assertEqual(decoded, rows)

    def test_png_structure_filter_zero(self):
        row = bytes([1, 2, 3])
        png = mapio.encode_png_gray8(3, 1, [row])
        # 解析 IDAT：每行应为 1 字节 filter(0) + 3 字节像素
        pos = 8
        idat = b''
        while pos < len(png):
            length = struct.unpack('>I', png[pos:pos + 4])[0]
            tag = png[pos + 4:pos + 8]
            chunk = png[pos + 8:pos + 8 + length]
            pos += 12 + length
            if tag == b'IDAT':
                idat += chunk
        raw = zlib.decompress(idat)
        self.assertEqual(raw, b'\x00' + row)

    def test_decode_rejects_bad_signature(self):
        with self.assertRaises(mapio.MapIOError):
            mapio.decode_png_gray8(b'not a png')


class PgmTests(unittest.TestCase):

    def test_grid_pgm_row_order(self):
        grid = _grid()
        pgm = grid.to_pgm()
        self.assertTrue(pgm.startswith(b'P5\n3 2\n255\n'))
        img = mapio.parse_pgm(pgm)
        self.assertEqual((img.width, img.height, img.maxval), (3, 2, 255))
        # 行 0 = 世界最大 y（原数据第 1 行 [50, 100, 0]）
        self.assertEqual(img.rows[0], bytes([127, 0, 254]))
        self.assertEqual(img.rows[1], bytes([254, 205, 0]))

    def test_parse_pgm_with_comment(self):
        pgm = b'P5\n# comment\n2 2\n255\n' + bytes([1, 2, 3, 4])
        img = mapio.parse_pgm(pgm)
        self.assertEqual(img.width, 2)
        self.assertEqual(img.rows[0], bytes([1, 2]))

    def test_pgm_to_png_roundtrip(self):
        grid = _grid()
        png = mapio.pgm_to_png(grid.to_pgm())
        width, height, rows = mapio.decode_png_gray8(png)
        self.assertEqual((width, height), (3, 2))
        self.assertEqual(rows[0], bytes([127, 0, 254]))

    def test_parse_pgm_invalid(self):
        for raw in (b'P2\n2 2\n255\n1 2 3 4', b'P5\n2 2\n255\n1 2',
                    b'P5\n0 2\n255\n', b''):
            with self.assertRaises(mapio.MapIOError):
                mapio.parse_pgm(raw)


class MapGridTests(unittest.TestCase):

    def test_meta(self):
        grid = _grid()
        meta = grid.meta(12)
        self.assertEqual(meta['available'], True)
        self.assertEqual(meta['version'], 12)
        self.assertEqual(meta['width'], 3)
        self.assertEqual(meta['height'], 2)
        self.assertAlmostEqual(meta['resolution'], 0.05)
        self.assertEqual(meta['origin'], {'x': -2.4, 'y': -2.0, 'yaw': 0.0})
        self.assertEqual(meta['frame_id'], 'map')
        self.assertAlmostEqual(meta['stamp'], 1764012345.2)

    def test_content_hash_ignores_stamp_but_not_data(self):
        base = _grid()
        same = mapio.make_map_grid(3, 2, 0.05, (-2.4, -2.0, 0.0),
                                   [0, -1, 100, 50, 100, 0],
                                   frame_id='map', stamp=1.0)
        moved = mapio.make_map_grid(3, 2, 0.05, (-2.4, -2.0, 0.0),
                                    [0, -1, 100, 50, 100, 1],
                                    frame_id='map', stamp=1.0)
        self.assertEqual(base.content_hash(), same.content_hash())
        self.assertNotEqual(base.content_hash(), moved.content_hash())

    def test_invalid_grid(self):
        with self.assertRaises(mapio.MapIOError):
            mapio.make_map_grid(0, 2, 0.05, (0, 0, 0), [])
        with self.assertRaises(mapio.MapIOError):
            mapio.make_map_grid(2, 2, 0.05, (0, 0, 0), [0, 1])


class YamlTests(unittest.TestCase):

    def test_parse_minimal_yaml(self):
        text = (
            '# 注释\n'
            'image: room.pgm\n'
            'mode: "trinary"  # 行尾注释\n'
            'resolution: 0.05\n'
            'origin: [-2.4, -2.0, 0.0]\n'
            'negate: 0\n'
        )
        parsed = mapio.parse_minimal_yaml(text)
        self.assertEqual(parsed['image'], 'room.pgm')
        self.assertEqual(parsed['mode'], 'trinary')
        self.assertEqual(parsed['resolution'], 0.05)
        self.assertEqual(parsed['origin'], [-2.4, -2.0, 0.0])
        self.assertEqual(parsed['negate'], 0)

    def test_format_map_yaml(self):
        text = mapio.format_map_yaml('room.pgm', 0.05, (-2.4, -2.0, 0.0))
        lines = text.strip().splitlines()
        self.assertEqual(lines[0], 'image: room.pgm')
        self.assertEqual(lines[1], 'mode: trinary')
        self.assertEqual(lines[2], 'resolution: 0.05')
        self.assertEqual(lines[3], 'origin: [-2.4, -2.0, 0.0]')
        self.assertEqual(lines[4], 'negate: 0')
        self.assertEqual(lines[5], 'occupied_thresh: 0.65')
        self.assertEqual(lines[6], 'free_thresh: 0.196')

    def test_load_map_yaml_valid_and_invalid(self):
        tmp = support.temp_dir()
        valid = os.path.join(tmp, 'room.yaml')
        with open(valid, 'w', encoding='utf-8') as fh:
            fh.write(mapio.format_map_yaml('room.pgm', 0.05, (-2.4, -2.0, 0.0)))
        info = mapio.load_map_yaml(valid)
        self.assertTrue(info.valid)
        self.assertEqual(info.name, 'room')
        self.assertEqual(info.pgm_path, os.path.join(tmp, 'room.pgm'))

        bad = os.path.join(tmp, 'bad.yaml')
        with open(bad, 'w', encoding='utf-8') as fh:
            fh.write('image: [\nresolution: x\n')
        bad_info = mapio.load_map_yaml(bad)
        self.assertFalse(bad_info.valid)
        self.assertTrue(bad_info.error)

        missing = mapio.load_map_yaml(os.path.join(tmp, 'nope.yaml'))
        self.assertFalse(missing.valid)


class MapFileTests(unittest.TestCase):

    def setUp(self):
        self.tmp = support.temp_dir()
        self.maps_dir = os.path.join(self.tmp, 'maps')
        self.fallback = os.path.join(self.tmp, 'fallback')
        os.makedirs(self.maps_dir)
        os.makedirs(self.fallback)

    def _write_map(self, directory, name, grid=None):
        return mapio.save_map(directory, name, grid or _grid())

    def test_save_and_list(self):
        result = self._write_map(self.maps_dir, 'room')
        self.assertTrue(os.path.isfile(result['yaml']))
        self.assertTrue(os.path.isfile(result['pgm']))
        self.assertEqual(result['width'], 3)
        self.assertEqual(result['height'], 2)
        self.assertAlmostEqual(result['resolution'], 0.05)
        with open(result['yaml'], 'r', encoding='utf-8') as fh:
            self.assertIn('image: room.pgm', fh.read())
        maps = mapio.list_maps([self.maps_dir])
        self.assertEqual(len(maps), 1)
        entry = maps[0]
        self.assertEqual(entry['name'], 'room')
        self.assertTrue(entry['valid'])
        self.assertEqual(entry['preview_url'], '/api/maps/room/preview.png')
        self.assertEqual(entry['width'], 3)
        self.assertEqual(entry['height'], 2)

    def test_save_conflict_and_overwrite(self):
        self._write_map(self.maps_dir, 'room')
        with self.assertRaises(mapio.MapExistsError):
            self._write_map(self.maps_dir, 'room')
        result = mapio.save_map(self.maps_dir, 'room', _grid(), overwrite=True)
        self.assertTrue(result['ok'])

    def test_save_invalid_name(self):
        with self.assertRaises(mapio.MapIOError):
            mapio.save_map(self.maps_dir, 'a/b', _grid())

    def test_list_priority_and_invalid(self):
        self._write_map(self.maps_dir, 'room')
        self._write_map(self.fallback, 'room')
        self._write_map(self.fallback, 'other')
        bad = os.path.join(self.fallback, 'broken.yaml')
        with open(bad, 'w', encoding='utf-8') as fh:
            fh.write('image: broken.pgm\nresolution: bad\n')
        maps = mapio.list_maps([self.maps_dir, self.fallback])
        names = [entry['name'] for entry in maps]
        # 目录优先级：主目录 room 先出现；随后 fallback 内按文件名排序
        self.assertEqual(names, ['room', 'broken', 'other'])
        self.assertIn(os.path.join(self.maps_dir, 'room.yaml'),
                      [entry['path'] for entry in maps])
        broken = [entry for entry in maps if entry['name'] == 'broken'][0]
        self.assertFalse(broken['valid'])

    def test_choose_save_dir(self):
        self.assertEqual(mapio.choose_save_dir(self.maps_dir, self.fallback),
                         os.path.abspath(self.maps_dir))
        missing = os.path.join(self.tmp, 'missing')
        chosen = mapio.choose_save_dir(missing, self.fallback)
        self.assertEqual(chosen, os.path.abspath(self.fallback))

    def test_preview_png(self):
        result = self._write_map(self.maps_dir, 'room')
        png = mapio.pgm_file_to_png(result['pgm'])
        width, height, rows = mapio.decode_png_gray8(png)
        self.assertEqual((width, height), (3, 2))
        self.assertEqual(rows[0], bytes([127, 0, 254]))

    def test_existing_map_dirs(self):
        dirs = mapio.existing_map_dirs([self.maps_dir, '/no/such/dir'])
        self.assertEqual(dirs, [os.path.abspath(self.maps_dir)])

    def test_name_valid(self):
        self.assertTrue(mapio.map_name_valid('room_1'))
        self.assertFalse(mapio.map_name_valid('a b'))
        self.assertFalse(mapio.map_name_valid(''))


if __name__ == '__main__':
    unittest.main()
