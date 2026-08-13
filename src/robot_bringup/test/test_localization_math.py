"""Unit tests for localization mathematics."""

import math
import sys
from pathlib import Path


NODES_DIR = Path(__file__).resolve().parents[1] / "nodes"
sys.path.insert(0, str(NODES_DIR))

from localization_math import geodetic_to_enu, quaternion_to_yaw


REF_LAT = 42.488712
REF_LON = -83.551455
REF_ALT = 260.696


def test_reference_point_maps_to_enu_origin():
    east, north, up = geodetic_to_enu(
        REF_LAT,
        REF_LON,
        REF_ALT,
        REF_LAT,
        REF_LON,
        REF_ALT,
    )

    assert math.isclose(east, 0.0, abs_tol=1e-9)
    assert math.isclose(north, 0.0, abs_tol=1e-9)
    assert math.isclose(up, 0.0, abs_tol=1e-9)


def test_altitude_change_maps_to_up_axis():
    east, north, up = geodetic_to_enu(
        REF_LAT,
        REF_LON,
        REF_ALT + 10.0,
        REF_LAT,
        REF_LON,
        REF_ALT,
    )

    assert math.isclose(east, 0.0, abs_tol=1e-9)
    assert math.isclose(north, 0.0, abs_tol=1e-9)
    assert math.isclose(up, 10.0, abs_tol=1e-9)


def test_identity_quaternion_has_zero_yaw():
    yaw = quaternion_to_yaw(
        x=0.0,
        y=0.0,
        z=0.0,
        w=1.0,
    )

    assert math.isclose(yaw, 0.0, abs_tol=1e-9)


def test_positive_ninety_degree_yaw():
    half_angle = math.pi / 4.0

    yaw = quaternion_to_yaw(
        x=0.0,
        y=0.0,
        z=math.sin(half_angle),
        w=math.cos(half_angle),
    )

    assert math.isclose(yaw, math.pi / 2.0, abs_tol=1e-9)


def test_negative_ninety_degree_yaw():
    half_angle = -math.pi / 4.0

    yaw = quaternion_to_yaw(
        x=0.0,
        y=0.0,
        z=math.sin(half_angle),
        w=math.cos(half_angle),
    )

    assert math.isclose(yaw, -math.pi / 2.0, abs_tol=1e-9)


def test_one_hundred_eighty_degree_yaw():
    yaw = quaternion_to_yaw(
        x=0.0,
        y=0.0,
        z=1.0,
        w=0.0,
    )

    assert math.isclose(abs(yaw), math.pi, abs_tol=1e-9)