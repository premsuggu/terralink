"""Unit tests for nav.ply_export - the shared PLY point-cloud writer and
color-mapping helpers used by scripts/export_elevation_map.py and
voxel_map_node.py's own export-on-request handler. Pure NumPy/file-I/O,
no ROS/Gazebo needed - same discipline as every other algorithm module in
this project.
"""
import numpy as np

from nav.ply_export import elevation_to_rgb, traversability_to_rgb, write_ply_points
from emap.traversability import DIFFICULT, EASY, LETHAL


class TestWritePlyPoints:
    def test_writes_correct_header_and_vertex_count(self, tmp_path):
        points = np.array([[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]])
        out = tmp_path / "cloud.ply"
        write_ply_points(str(out), points)

        lines = out.read_text().splitlines()
        assert lines[0] == "ply"
        assert lines[1] == "format ascii 1.0"
        assert "element vertex 2" in lines
        assert "end_header" in lines

    def test_default_color_is_written_when_none_given(self, tmp_path):
        points = np.array([[0.0, 0.0, 0.0]])
        out = tmp_path / "cloud.ply"
        write_ply_points(str(out), points)

        last_line = out.read_text().splitlines()[-1]
        assert last_line == "0.000000 0.000000 0.000000 200 200 200"

    def test_custom_colors_are_written_per_point(self, tmp_path):
        points = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]])
        colors = np.array([[255, 0, 0], [0, 255, 0]])
        out = tmp_path / "cloud.ply"
        write_ply_points(str(out), points, colors_rgb=colors)

        data_lines = out.read_text().splitlines()[-2:]
        assert data_lines[0].endswith("255 0 0")
        assert data_lines[1].endswith("0 255 0")

    def test_mismatched_color_length_raises(self, tmp_path):
        points = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]])
        colors = np.array([[255, 0, 0]])  # only 1 color for 2 points
        out = tmp_path / "cloud.ply"
        try:
            write_ply_points(str(out), points, colors_rgb=colors)
            assert False, "expected ValueError"
        except ValueError:
            pass

    def test_empty_point_cloud_still_writes_a_valid_header(self, tmp_path):
        points = np.empty((0, 3))
        out = tmp_path / "cloud.ply"
        write_ply_points(str(out), points)
        assert "element vertex 0" in out.read_text()


class TestElevationToRgb:
    def test_lowest_point_is_blue_highest_is_red(self):
        elevation = np.array([0.0, 1.0])
        colors = elevation_to_rgb(elevation)
        # lowest -> blue-dominant, highest -> red-dominant
        assert colors[0][2] > colors[0][0]
        assert colors[1][0] > colors[1][2]

    def test_flat_map_does_not_crash_on_zero_span(self):
        elevation = np.array([5.0, 5.0, 5.0])
        colors = elevation_to_rgb(elevation)
        assert colors.shape == (3, 3)


class TestTraversabilityToRgb:
    def test_lethal_cell_is_red(self):
        colors = traversability_to_rgb(np.array([LETHAL]), np.array([True]))
        assert tuple(colors[0]) == (220, 40, 40)

    def test_difficult_cell_is_yellow(self):
        colors = traversability_to_rgb(np.array([DIFFICULT]), np.array([True]))
        assert tuple(colors[0]) == (230, 200, 40)

    def test_easy_cell_is_green(self):
        colors = traversability_to_rgb(np.array([EASY]), np.array([True]))
        assert tuple(colors[0]) == (60, 200, 60)

    def test_unobserved_cell_is_gray_regardless_of_score(self):
        # is_valid=False must win even where the score looks EASY - "haven't
        # looked yet" must never be silently drawn as "confirmed safe".
        colors = traversability_to_rgb(np.array([EASY]), np.array([False]))
        assert tuple(colors[0]) == (120, 120, 120)
