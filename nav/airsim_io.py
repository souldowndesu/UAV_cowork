# -*- coding: utf-8 -*-
"""AirSim 连接、状态读取、LiDAR 点云读取（线程安全，单一客户端）。

坐标全程 NED 世界系（+X 北 / +Y 东 / +Z 向下）。单架 SimpleFlight 载具出生点即世界
原点，``simGetVehiclePose`` / ``getMultirotorState`` 的 position 直接作为世界系真值。

线程安全：msgpackrpc/tornado 的客户端连接**不是线程安全的**，多线程并发调用会触发
``BufferError: Existing exports of data``。因此**所有 RPC 调用都在同一把 ``RLock`` 下
串行执行**（含载具移动指令），这是本项目唯一访问客户端的出口。
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass

import numpy as np


@dataclass
class VehicleState:
    position: np.ndarray       # (3,) 世界系 NED
    velocity: np.ndarray       # (3,) 世界系线速度 m/s
    acceleration: np.ndarray   # (3,) 世界系线加速度 m/s²（由 body 旋转而来）
    yaw: float                 # 弧度
    collision: bool = False
    timestamp_ns: int = 0


def quat_to_rot_body2world(q) -> np.ndarray:
    """AirSim 四元数(w,x,y,z) → body→world 旋转矩阵 R_wb。

    AirSim 的 ``linear_acceleration`` 在 body frame，转到世界系用 ``R_wb @ a_body``。
    """
    w, x, y, z = float(q.w_val), float(q.x_val), float(q.y_val), float(q.z_val)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype="float64")


class AirSimIO:
    def __init__(self, cfg):
        self.cfg = cfg
        self._lock = threading.RLock()
        self._client = None
        self._airsim = None

    # ------------------------------------------------------------------
    # 内部（须在持锁状态下调用）
    # ------------------------------------------------------------------
    def _connect_locked(self):
        import airsim
        self._airsim = airsim
        client = airsim.MultirotorClient(ip=self.cfg.ip, port=self.cfg.port)
        client.confirmConnection()
        return client

    def _client_locked(self):
        if self._client is None:
            self._client = self._connect_locked()
        return self._client

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------
    def arm(self):
        with self._lock:
            c = self._client_locked()
            c.enableApiControl(True, vehicle_name=self.cfg.vehicle_name)
            c.armDisarm(True, vehicle_name=self.cfg.vehicle_name)

    def takeoff(self):
        with self._lock:
            c = self._client_locked()
            c.takeoffAsync(vehicle_name=self.cfg.vehicle_name).join()

    def land(self):
        with self._lock:
            c = self._client_locked()
            # fire-and-forget：不 join。landAsync().join() 在无人机悬停/看门狗接管时会
            # 无限期挂起，阻塞整个进程；改为下发后立即返回。
            c.landAsync(vehicle_name=self.cfg.vehicle_name)

    def teleport(self, pos, yaw_deg: float = 0.0):
        with self._lock:
            c = self._client_locked()
            pose = self._airsim.Pose(
                self._airsim.Vector3r(float(pos[0]), float(pos[1]), float(pos[2])),
                self._airsim.to_quaternion(0.0, 0.0, math.radians(float(yaw_deg))),
            )
            c.simSetVehiclePose(pose, True, vehicle_name=self.cfg.vehicle_name)
            c.enableApiControl(True, vehicle_name=self.cfg.vehicle_name)
            c.armDisarm(True, vehicle_name=self.cfg.vehicle_name)

    def move_to_z(self, z: float, velocity: float = 2.0):
        with self._lock:
            c = self._client_locked()
            c.moveToZAsync(float(z), float(velocity),
                           vehicle_name=self.cfg.vehicle_name).join()

    def move_by_velocity(self, vx: float, vy: float, vz: float, duration: float):
        """唯一写者下发的速度指令（在锁内完成 RPC 发送，fire-and-forget）。"""
        with self._lock:
            c = self._client_locked()
            c.moveByVelocityAsync(float(vx), float(vy), float(vz), float(duration),
                                  vehicle_name=self.cfg.vehicle_name)

    def move_by_velocity_z(self, vx: float, vy: float, z: float, duration: float):
        """水平速度（世界系 NED）+ 显式锁定高度 z（AltZ 高度控制器）。

        用 ``MaxDegreeOfFreedom``（四旋翼可任意方向蟹行、机头不受运动方向约束）而非
        ``ForwardOnly``：协调器下发的是**世界系**速度 ``u``，``ForwardOnly`` 会把 vx/vy
        当机体系并强迫机头转向运动方向，既造成方向偏差又拖慢实际速度；``yaw_mode`` 保持
        当前偏航不旋转（is_rate=True、速率 0）。
        """
        with self._lock:
            c = self._client_locked()
            c.moveByVelocityZAsync(
                float(vx), float(vy), float(z), float(duration),
                drivetrain=self._airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=self._airsim.YawMode(True, 0.0),
                vehicle_name=self.cfg.vehicle_name)

    # ------------------------------------------------------------------
    # 状态 / 感知
    # ------------------------------------------------------------------
    def get_state(self) -> VehicleState:
        with self._lock:
            c = self._client_locked()
            s = c.getMultirotorState(vehicle_name=self.cfg.vehicle_name)
            k = s.kinematics_estimated
            pitch, roll, yaw = self._airsim.to_eularian_angles(k.orientation)
            # linear_acceleration 在 body frame，转到世界系（NED）
            a_body = np.array([k.linear_acceleration.x_val,
                               k.linear_acceleration.y_val,
                               k.linear_acceleration.z_val], dtype="float64")
            R_wb = quat_to_rot_body2world(k.orientation)
            a_world = R_wb @ a_body
            return VehicleState(
                position=np.array([k.position.x_val, k.position.y_val, k.position.z_val],
                                  dtype="float64"),
                velocity=np.array([k.linear_velocity.x_val, k.linear_velocity.y_val,
                                   k.linear_velocity.z_val], dtype="float64"),
                acceleration=a_world,
                yaw=float(yaw),
                collision=bool(getattr(getattr(s, "collision", None), "has_collided", False)),
                timestamp_ns=int(s.timestamp),
            )

    def get_lidar_points(self) -> np.ndarray:
        """返回 LiDAR 点云（世界系 NED，米），形状 (N,3) float32。

        实测：尽管 settings.json 配 ``DataFrame="VehicleInertialFrame"``，本构建
        ``getLidarData`` 返回的 ``point_cloud`` 实际是**全局坐标**（瞬移载具后点云质心
        随世界固定、不随载具相对移动）。因此直接作为世界点返回，**不再叠加载具位置**。
        未配置 LiDAR 时返回空数组。
        """
        with self._lock:
            c = self._client_locked()
            try:
                ld = c.getLidarData(lidar_name=self.cfg.lidar_name,
                                    vehicle_name=self.cfg.vehicle_name)
            except Exception:
                return np.zeros((0, 3), dtype="float32")
            pc = getattr(ld, "point_cloud", None)
            if pc is None or len(pc) == 0:
                return np.zeros((0, 3), dtype="float32")
            pts = np.asarray(pc, dtype="float64").reshape(-1, 3)
            # 点云已是全局 NED 坐标，直接返回
            return pts.astype("float32")

    def get_depth_points(self) -> np.ndarray:
        """回退感知：前中相机 DepthPerspective → 世界系点云（近似，忽略俯仰/横滚）。"""
        with self._lock:
            c = self._client_locked()
            responses = c.simGetImages(
                [self._airsim.ImageRequest(self.cfg.camera_name,
                                           self._airsim.ImageType.DepthPerspective,
                                           True, False)],
                vehicle_name=self.cfg.vehicle_name)
            if not responses:
                return np.zeros((0, 3), dtype="float32")
            resp = responses[0]
            depth = self._airsim.list_to_2d_float_array(resp.image_data_float,
                                                        resp.width, resp.height)
            depth = np.asarray(depth, dtype="float64") / 100.0  # cm → m
            # 同帧读取载具位姿/姿态（同一把锁内）
            s = c.getMultirotorState(vehicle_name=self.cfg.vehicle_name)
            k = s.kinematics_estimated
            pos = np.array([k.position.x_val, k.position.y_val, k.position.z_val])
            pitch, roll, yaw = self._airsim.to_eularian_angles(k.orientation)
        # ---- 释放锁后做纯 numpy 反投影（不涉及 RPC） ----
        H, W = depth.shape
        hfov = math.radians(90.0)
        fx = (W / 2.0) / math.tan(hfov / 2.0)
        fy = fx  # 方形像素近似
        cx, cy = W / 2.0, H / 2.0
        u, v = np.meshgrid(np.arange(W), np.arange(H))
        xn = (u - cx) / fx
        yn = (v - cy) / fy
        norm = np.sqrt(xn * xn + yn * yn + 1.0)
        d = depth
        px = d / norm
        py = xn * d / norm
        pz = yn * d / norm
        valid = np.isfinite(d) & (d > 0.3) & (d < self.cfg.max_range)
        body = np.stack([px[valid], py[valid], pz[valid]], axis=1)
        cyw, syw = math.cos(yaw), math.sin(yaw)
        R = np.array([[cyw, -syw, 0.0], [syw, cyw, 0.0], [0.0, 0.0, 1.0]])
        world = body @ R.T + pos
        return world[::4].astype("float32")
