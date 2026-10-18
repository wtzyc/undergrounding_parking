"""God-camera ground-plane BEV projection utilities."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import cv2
import numpy as np


class BEVError(ValueError):
    """Raised when BEV calibration or projection input is invalid."""


def _finite(value: float, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise BEVError(f'{name} must be finite')
    return number


def _validate_rotation(values: tuple[float, ...]) -> np.ndarray:
    if len(values) != 9:
        raise BEVError('camera rotation must contain nine values')
    matrix = np.asarray([_finite(value, 'camera rotation') for value in values])
    matrix = matrix.reshape((3, 3))
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-4):
        raise BEVError('camera rotation must be an orthonormal matrix')
    if np.linalg.det(matrix) <= 0.0:
        raise BEVError('camera rotation must have a positive determinant')
    return matrix


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    roll = _finite(roll, 'roll')
    pitch = _finite(pitch, 'pitch')
    yaw = _finite(yaw, 'yaw')
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole camera parameters in pixel coordinates."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        if int(self.width) != self.width or int(self.height) != self.height:
            raise BEVError('image width and height must be integers')
        if self.width <= 0 or self.height <= 0:
            raise BEVError('image width and height must be positive')
        for value, name in (
            (self.fx, 'fx'),
            (self.fy, 'fy'),
            (self.cx, 'cx'),
            (self.cy, 'cy'),
        ):
            _finite(value, name)
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise BEVError('fx and fy must be positive')
        distortion = tuple(_finite(value, 'distortion') for value in self.distortion)
        object.__setattr__(self, 'distortion', distortion)

    @classmethod
    def from_camera_info(cls, camera_info: Any) -> 'CameraIntrinsics':
        k = list(camera_info.k)
        if len(k) < 9:
            raise BEVError('CameraInfo.k must contain nine values')
        return cls(
            width=int(camera_info.width),
            height=int(camera_info.height),
            fx=float(k[0]),
            fy=float(k[4]),
            cx=float(k[2]),
            cy=float(k[5]),
            distortion=tuple(float(value) for value in camera_info.d),
        )

    @property
    def matrix(self) -> np.ndarray:
        return np.asarray([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0],
        ], dtype=float)


@dataclass(frozen=True)
class CameraExtrinsics:
    """Camera optical-frame pose expressed in the base frame.

    The rotation maps a vector in the camera optical frame (x right, y down,
    z forward) to the vehicle base frame (x forward, y left, z up).
    """

    x: float
    y: float
    z: float
    rotation_camera_to_base: tuple[float, ...]
    frame_id: str = 'camera_god'

    def __post_init__(self) -> None:
        for value, name in ((self.x, 'camera x'), (self.y, 'camera y'), (self.z, 'camera z')):
            _finite(value, name)
        if not self.frame_id:
            raise BEVError('camera frame_id must not be empty')
        _validate_rotation(tuple(self.rotation_camera_to_base))

    @classmethod
    def from_base_rpy(
        cls,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        frame_id: str = 'camera_god',
    ) -> 'CameraExtrinsics':
        # Optical axes for a level camera aligned with the vehicle base:
        # camera z -> base x, camera x -> base -y, camera y -> base -z.
        optical_to_level_base = np.asarray([
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ])
        rotation = _rpy_matrix(roll, pitch, yaw) @ optical_to_level_base
        return cls(
            x=float(x),
            y=float(y),
            z=float(z),
            rotation_camera_to_base=tuple(float(value) for value in rotation.reshape(-1)),
            frame_id=frame_id,
        )

    @property
    def rotation(self) -> np.ndarray:
        return _validate_rotation(tuple(self.rotation_camera_to_base))

    @property
    def translation(self) -> np.ndarray:
        return np.asarray([self.x, self.y, self.z], dtype=float)


@dataclass(frozen=True)
class BEVConfig:
    """Rectangular ground region and its pixel resolution."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    resolution_m_per_pixel: float
    frame_id: str = 'base_footprint'
    ground_z: float = 0.0

    def __post_init__(self) -> None:
        for value, name in (
            (self.x_min, 'x_min'),
            (self.x_max, 'x_max'),
            (self.y_min, 'y_min'),
            (self.y_max, 'y_max'),
            (self.resolution_m_per_pixel, 'resolution_m_per_pixel'),
            (self.ground_z, 'ground_z'),
        ):
            _finite(value, name)
        if self.x_max <= self.x_min or self.y_max <= self.y_min:
            raise BEVError('BEV bounds must have positive area')
        if self.resolution_m_per_pixel <= 0.0:
            raise BEVError('BEV resolution must be positive')
        if not self.frame_id:
            raise BEVError('BEV frame_id must not be empty')

    @property
    def width(self) -> int:
        return int(math.ceil((self.y_max - self.y_min) / self.resolution_m_per_pixel))

    @property
    def height(self) -> int:
        return int(math.ceil((self.x_max - self.x_min) / self.resolution_m_per_pixel))


@dataclass(frozen=True)
class WarpedBEV:
    """Warped image plus metadata needed by downstream geometry consumers."""

    image: np.ndarray
    valid_mask: np.ndarray
    stamp: float | None
    source_frame: str
    frame_id: str
    metadata: dict[str, Any]


class BEVProjector:
    """Project a calibrated pinhole camera onto a horizontal ground plane."""

    def __init__(
        self,
        intrinsics: CameraIntrinsics,
        extrinsics: CameraExtrinsics,
        config: BEVConfig,
    ) -> None:
        self.intrinsics = intrinsics
        self.extrinsics = extrinsics
        self.config = config
        self._camera_matrix = intrinsics.matrix
        self._camera_matrix_inverse = np.linalg.inv(self._camera_matrix)
        self._rotation = extrinsics.rotation
        self._base_to_camera = np.linalg.inv(self._rotation)
        self._distortion = np.asarray(intrinsics.distortion, dtype=float)
        self._has_distortion = bool(np.any(np.abs(self._distortion) > 1e-12))
        self._source_u, self._source_v, self._valid_mask, self._valid_pixel_ratio = (
            self._build_remap_tables()
        )

    def _build_remap_tables(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        """Precompute the fixed camera-to-ground projection used by every frame."""
        rows, columns = np.indices((self.config.height, self.config.width), dtype=np.float64)
        ground_x = self.config.x_min + rows * self.config.resolution_m_per_pixel
        ground_y = self.config.y_min + columns * self.config.resolution_m_per_pixel
        relative = np.stack((
            ground_x - self.extrinsics.x,
            ground_y - self.extrinsics.y,
            np.full_like(ground_x, self.config.ground_z - self.extrinsics.z),
        ), axis=-1)
        points_camera = relative @ self._base_to_camera.T
        depth = points_camera[:, :, 2]
        valid = depth > 1e-9
        source_u = np.full(depth.shape, -1.0, dtype=np.float32)
        source_v = np.full(depth.shape, -1.0, dtype=np.float32)

        if np.any(valid):
            valid_points = points_camera[valid]
            if self._has_distortion:
                projected, _ = cv2.projectPoints(
                    valid_points.reshape(-1, 1, 3),
                    np.zeros(3),
                    np.zeros(3),
                    self._camera_matrix,
                    self._distortion,
                )
                projected = projected.reshape(-1, 2)
                projected_u = projected[:, 0]
                projected_v = projected[:, 1]
            else:
                projected_u = (
                    self.intrinsics.fx * valid_points[:, 0] / valid_points[:, 2]
                    + self.intrinsics.cx
                )
                projected_v = (
                    self.intrinsics.fy * valid_points[:, 1] / valid_points[:, 2]
                    + self.intrinsics.cy
                )
            valid_rows, valid_columns = np.where(valid)
            source_u[valid_rows, valid_columns] = projected_u.astype(np.float32)
            source_v[valid_rows, valid_columns] = projected_v.astype(np.float32)
            valid &= (
                np.isfinite(source_u)
                & np.isfinite(source_v)
                & (source_u >= 0.0)
                & (source_u < self.intrinsics.width)
                & (source_v >= 0.0)
                & (source_v < self.intrinsics.height)
            )
            source_u[~valid] = -1.0
            source_v[~valid] = -1.0

        valid_ratio = float(np.count_nonzero(valid)) / float(valid.size)
        valid_mask = cv2.rotate(valid.astype(np.uint8) * 255, cv2.ROTATE_180)
        return source_u, source_v, valid_mask, valid_ratio

    def _undistorted_ray(self, u: float, v: float) -> np.ndarray:
        if self._has_distortion:
            point = np.asarray([[[float(u), float(v)]]], dtype=float)
            normalized = cv2.undistortPoints(
                point,
                self._camera_matrix,
                self._distortion,
            )[0, 0]
            return np.asarray([normalized[0], normalized[1], 1.0], dtype=float)
        return self._camera_matrix_inverse @ np.asarray([u, v, 1.0], dtype=float)

    def pixel_to_ground(
        self,
        u: float,
        v: float,
        *,
        require_image_bounds: bool = True,
    ) -> tuple[float, float] | None:
        u = _finite(u, 'pixel u')
        v = _finite(v, 'pixel v')
        if require_image_bounds and not (
            0.0 <= u < self.intrinsics.width
            and 0.0 <= v < self.intrinsics.height
        ):
            return None
        ray_camera = self._undistorted_ray(u, v)
        ray_base = self._rotation @ ray_camera
        denominator = float(ray_base[2])
        if abs(denominator) < 1e-9:
            return None
        scale = (self.config.ground_z - self.extrinsics.z) / denominator
        if not math.isfinite(scale) or scale <= 1e-9:
            return None
        point = self.extrinsics.translation + scale * ray_base
        if not np.all(np.isfinite(point)):
            return None
        return float(point[0]), float(point[1])

    def ground_to_pixel(
        self,
        x: float,
        y: float,
        *,
        require_image_bounds: bool = False,
    ) -> tuple[float, float] | None:
        x = _finite(x, 'ground x')
        y = _finite(y, 'ground y')
        point_base = np.asarray([
            x - self.extrinsics.x,
            y - self.extrinsics.y,
            self.config.ground_z - self.extrinsics.z,
        ], dtype=float)
        point_camera = self._base_to_camera @ point_base
        depth = float(point_camera[2])
        if depth <= 1e-9:
            return None
        if self._has_distortion:
            projected, _ = cv2.projectPoints(
                point_camera.reshape(1, 1, 3),
                np.zeros(3),
                np.zeros(3),
                self._camera_matrix,
                self._distortion,
            )
            u, v = projected[0, 0]
        else:
            u = self.intrinsics.fx * point_camera[0] / depth + self.intrinsics.cx
            v = self.intrinsics.fy * point_camera[1] / depth + self.intrinsics.cy
        u, v = _finite(u, 'projected u'), _finite(v, 'projected v')
        if require_image_bounds and not (
            0.0 <= u < self.intrinsics.width
            and 0.0 <= v < self.intrinsics.height
        ):
            return None
        return u, v

    def ground_to_bev(
        self,
        x: float,
        y: float,
        *,
        require_bounds: bool = True,
    ) -> tuple[float, float] | None:
        x = _finite(x, 'ground x')
        y = _finite(y, 'ground y')
        column = (y - self.config.y_min) / self.config.resolution_m_per_pixel
        row = (x - self.config.x_min) / self.config.resolution_m_per_pixel
        if require_bounds and not (
            0.0 <= column < self.config.width
            and 0.0 <= row < self.config.height
        ):
            return None
        return self.config.width - 1 - column, self.config.height - 1 - row

    def bev_to_ground(self, column: float, row: float) -> tuple[float, float] | None:
        column = _finite(column, 'BEV column')
        row = _finite(row, 'BEV row')
        if not (
            0.0 <= column < self.config.width
            and 0.0 <= row < self.config.height
        ):
            return None
        raw_column = self.config.width - 1 - column
        raw_row = self.config.height - 1 - row
        return (
            self.config.x_min + raw_row * self.config.resolution_m_per_pixel,
            self.config.y_min + raw_column * self.config.resolution_m_per_pixel,
        )

    def warp(
        self,
        image: np.ndarray,
        *,
        stamp: float | None = None,
        source_frame: str | None = None,
    ) -> WarpedBEV:
        if not isinstance(image, np.ndarray) or image.ndim not in (2, 3):
            raise BEVError('image must be a 2D or 3D numpy array')
        if image.shape[0] != self.intrinsics.height or image.shape[1] != self.intrinsics.width:
            raise BEVError('image dimensions do not match CameraInfo')
        warped = cv2.remap(
            image,
            self._source_u,
            self._source_v,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        warped = cv2.rotate(warped, cv2.ROTATE_180)
        metadata = {
            'status': 'ok',
            'stamp': stamp,
            'source_frame': source_frame or self.extrinsics.frame_id,
            'frame_id': self.config.frame_id,
            'ground_plane_z': self.config.ground_z,
            'resolution_m_per_pixel': self.config.resolution_m_per_pixel,
            'width_px': self.config.width,
            'height_px': self.config.height,
            'x_range_m': [self.config.x_min, self.config.x_max],
            'y_range_m': [self.config.y_min, self.config.y_max],
            'image_rotation_deg': 180,
            'pixel_origin_base': {
                'x': self.config.x_max - self.config.resolution_m_per_pixel,
                'y': self.config.y_max - self.config.resolution_m_per_pixel,
            },
            'pixel_axes': {'row': '-x', 'column': '-y'},
            'valid_pixel_ratio': self._valid_pixel_ratio,
        }
        return WarpedBEV(
            image=warped,
            valid_mask=self._valid_mask.copy(),
            stamp=stamp,
            source_frame=source_frame or self.extrinsics.frame_id,
            frame_id=self.config.frame_id,
            metadata=metadata,
        )

    def debug_overlay(self, image: np.ndarray) -> np.ndarray:
        if not isinstance(image, np.ndarray) or image.ndim not in (2, 3):
            raise BEVError('image must be a 2D or 3D numpy array')
        if image.ndim == 2:
            overlay = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            overlay = image.copy()
        corners = [
            (self.config.x_min, self.config.y_min),
            (self.config.x_max, self.config.y_min),
            (self.config.x_max, self.config.y_max),
            (self.config.x_min, self.config.y_max),
        ]
        projected = [
            self.ground_to_pixel(x, y, require_image_bounds=False)
            for x, y in corners
        ]
        visible = [
            (int(round(u)), int(round(v)))
            for point in projected
            if point is not None
            for u, v in (point,)
        ]
        if len(visible) >= 2:
            cv2.polylines(
                overlay,
                [np.asarray(visible, dtype=np.int32)],
                isClosed=len(visible) == 4,
                color=(0, 255, 255),
                thickness=2,
            )
        for x in np.arange(self.config.x_min, self.config.x_max, 1.0):
            points = [
                self.ground_to_pixel(x, self.config.y_min, require_image_bounds=False),
                self.ground_to_pixel(x, self.config.y_max, require_image_bounds=False),
            ]
            points = [
                (int(round(u)), int(round(v)))
                for point in points
                if point is not None
                for u, v in (point,)
            ]
            if len(points) == 2:
                cv2.line(overlay, points[0], points[1], (255, 180, 0), 1)
        return overlay

    def status(
        self,
        status: str,
        *,
        stamp: float | None = None,
        source_frame: str = '',
        reason: str = '',
    ) -> dict[str, Any]:
        if not status:
            raise BEVError('status must not be empty')
        return {
            'status': status,
            'reason': reason,
            'stamp': stamp,
            'source_frame': source_frame,
            'frame_id': self.config.frame_id,
            'ground_plane_z': self.config.ground_z,
            'resolution_m_per_pixel': self.config.resolution_m_per_pixel,
            'width_px': self.config.width,
            'height_px': self.config.height,
            'image_rotation_deg': 180,
            'pixel_origin_base': {
                'x': self.config.x_max - self.config.resolution_m_per_pixel,
                'y': self.config.y_max - self.config.resolution_m_per_pixel,
            },
            'pixel_axes': {'row': '-x', 'column': '-y'},
        }
