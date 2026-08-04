"""
摄像头调度管理模块 - CameraManager

职责:
- 独立线程管理前视摄像头（当前单摄像头模式）
- 预留人脸识别摄像头接口（后续扩展）
- 维护最新帧字典，支持非阻塞读取
- 控制补光灯开关

设计原则:
1. 任何读取前必须判空，防止丢帧崩溃
2. 使用线程锁保证线程安全
3. 支持动态启停，资源正确释放
4. 单摄像头模式下，侧视/人脸识别摄像头返回 None
"""

import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

import sys
sys.path.insert(0, str(Path(__file__).parent.parent))
from utils.logger import get_logger


@dataclass
class CameraConfig:
    """摄像头配置数据类"""
    device_id: int
    width: int = 1920
    height: int = 1080
    fps: int = 30
    exposure: int = -1
    auto_focus: bool = False
    name: str = "camera"


class CameraThread(threading.Thread):
    """摄像头采集线程"""

    def __init__(self, config: CameraConfig, logger=None):
        super().__init__(daemon=True)
        self.config = config
        self.logger = logger or get_logger()

        self._cap: Optional[cv2.VideoCapture] = None
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()
        self._running = False
        self._frame_count = 0
        self._fps_counter = 0
        self._last_fps_time = time.time()
        self._actual_fps = 0.0

    def open(self) -> bool:
        """打开摄像头"""
        if cv2 is None:
            self.logger.error(f"[{self.config.name}] OpenCV未安装")
            return False

        try:
            # 尝试V4L2后端 (Linux)
            self._cap = cv2.VideoCapture(self.config.device_id, cv2.CAP_V4L2)
            if not self._cap.isOpened():
                # 回退到默认后端
                self._cap = cv2.VideoCapture(self.config.device_id)

            if not self._cap.isOpened():
                self.logger.error(f"[{self.config.name}] 无法打开摄像头 device_id={self.config.device_id}")
                return False

            # 设置分辨率
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
            self._cap.set(cv2.CAP_PROP_FPS, self.config.fps)

            # 设置曝光
            if self.config.exposure >= 0:
                self._cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 关闭自动曝光
                self._cap.set(cv2.CAP_PROP_EXPOSURE, self.config.exposure)

            # 禁用自动对焦 (提高稳定性)
            if not self.config.auto_focus:
                self._cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

            # 读取一帧测试
            ret, frame = self._cap.read()
            if not ret or frame is None:
                self.logger.error(f"[{self.config.name}] 无法读取帧")
                self._cap.release()
                return False

            actual_width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self._cap.get(cv2.CAP_PROP_FPS)

            self.logger.info(
                f"[{self.config.name}] 摄像头已打开 | "
                f"分辨率: {actual_width}x{actual_height} | "
                f"FPS: {actual_fps}"
            )
            return True

        except Exception as e:
            self.logger.error(f"[{self.config.name}] 打开摄像头异常: {e}")
            return False

    def run(self):
        """线程主循环"""
        if self._cap is None or not self._cap.isOpened():
            self.logger.error(f"[{self.config.name}] 摄像头未正确打开，线程退出")
            return

        self._running = True
        self.logger.info(f"[{self.config.name}] 采集线程启动")

        while self._running:
            try:
                ret, frame = self._cap.read()
                if ret and frame is not None:
                    with self._frame_lock:
                        self._latest_frame = frame.copy()
                    self._frame_count += 1
                    self._fps_counter += 1
                else:
                    self.logger.warning(f"[{self.config.name}] 读取帧失败")
                    time.sleep(0.001)

                # FPS计算
                current_time = time.time()
                if current_time - self._last_fps_time >= 1.0:
                    self._actual_fps = self._fps_counter / (current_time - self._last_fps_time)
                    self._fps_counter = 0
                    self._last_fps_time = current_time

            except Exception as e:
                self.logger.exception(f"[{self.config.name}] 采集异常: {e}")
                time.sleep(0.01)

        self.logger.info(f"[{self.config.name}] 采集线程停止 | 总帧数: {self._frame_count}")

    def get_frame(self) -> Optional[np.ndarray]:
        """获取最新帧 (线程安全)"""
        with self._frame_lock:
            if self._latest_frame is not None:
                return self._latest_frame.copy()
            return None

    def get_fps(self) -> float:
        """获取实际FPS"""
        return self._actual_fps

    def stop(self):
        """停止采集线程"""
        self._running = False
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        self.logger.info(f"[{self.config.name}] 摄像头已释放")


class CameraManager:
    """
    摄像头调度管理器

    【当前配置: 单摄像头模式】
    - front: 前视摄像头 (巡线用，必须)
    - side: 侧视摄像头 (侦察用，当前禁用)
    - face: 人脸识别摄像头 (后续扩展，当前禁用)

    提供统一的帧获取接口，未启用的摄像头返回 None。
    """

    def __init__(self, settings: dict):
        """
        初始化摄像头管理器

        Args:
            settings: 配置字典，从settings.yaml加载
        """
        self.logger = get_logger()
        self.settings = settings

        # 解析配置
        cam_cfg = settings.get('cameras', {})

        # 前视摄像头 (主摄像头，必须)
        self.front_config = CameraConfig(
            device_id=cam_cfg.get('front', {}).get('device_id', 0),
            width=cam_cfg.get('front', {}).get('width', 1920),
            height=cam_cfg.get('front', {}).get('height', 1080),
            fps=cam_cfg.get('front', {}).get('fps', 30),
            exposure=cam_cfg.get('front', {}).get('exposure', -1),
            auto_focus=cam_cfg.get('front', {}).get('auto_focus', False),
            name="front"
        )

        # 侧视摄像头 (当前禁用，预留配置)
        self.side_config = CameraConfig(
            device_id=cam_cfg.get('side', {}).get('device_id', 1),
            width=cam_cfg.get('side', {}).get('width', 1920),
            height=cam_cfg.get('side', {}).get('height', 1080),
            fps=cam_cfg.get('side', {}).get('fps', 30),
            exposure=cam_cfg.get('side', {}).get('exposure', -1),
            auto_focus=cam_cfg.get('side', {}).get('auto_focus', True),
            name="side"
        )

        # 人脸识别摄像头 (后续扩展，当前禁用)
        self.face_config = None
        face_cfg = cam_cfg.get('face')
        if face_cfg:
            self.face_config = CameraConfig(
                device_id=face_cfg.get('device_id', 2),
                width=face_cfg.get('width', 1920),
                height=face_cfg.get('height', 1080),
                fps=face_cfg.get('fps', 30),
                exposure=face_cfg.get('exposure', -1),
                auto_focus=face_cfg.get('auto_focus', True),
                name="face"
            )

        # 摄像头线程
        self.front_cam: Optional[CameraThread] = None
        self.side_cam: Optional[CameraThread] = None
        self.face_cam: Optional[CameraThread] = None

        # 补光灯GPIO引脚 (预留)
        self.led_config = settings.get('led', {})
        self.front_led_pin = self.led_config.get('front_pin', 17)
        self.side_led_pin = self.led_config.get('side_pin', 27)
        self._led_initialized = False

        # 摄像头启用开关
        self._enable_side = cam_cfg.get('side', {}).get('enabled', False)
        self._enable_face = cam_cfg.get('face', {}).get('enabled', False) if face_cfg else False

        self.logger.info(
            f"CameraManager 初始化完成 | "
            f"前视: 启用 | 侧视: {'启用' if self._enable_side else '禁用'} | "
            f"人脸: {'启用' if self._enable_face else '禁用(预留)'}"
        )

    def initialize(self) -> bool:
        """
        初始化所有摄像头和补光灯

        Returns:
            bool: 前视摄像头是否成功打开
        """
        success = False

        # 初始化前视摄像头 (必须)
        self.front_cam = CameraThread(self.front_config, self.logger)
        if self.front_cam.open():
            self.front_cam.start()
            success = True
        else:
            self.logger.error("前视摄像头初始化失败")
            self.front_cam = None

        # 初始化侧视摄像头 (当前禁用)
        if self._enable_side:
            self.side_cam = CameraThread(self.side_config, self.logger)
            if self.side_cam.open():
                self.side_cam.start()
            else:
                self.logger.warning("侧视摄像头初始化失败")
                self.side_cam = None
        else:
            self.logger.info("侧视摄像头已禁用（单摄像头模式）")
            self.side_cam = None

        # 初始化人脸识别摄像头 (后续扩展)
        if self._enable_face and self.face_config:
            self.face_cam = CameraThread(self.face_config, self.logger)
            if self.face_cam.open():
                self.face_cam.start()
            else:
                self.logger.warning("人脸识别摄像头初始化失败")
                self.face_cam = None
        else:
            self.face_cam = None

        # 初始化补光灯GPIO (RDK X5)
        try:
            import gpiod
            self._init_led_gpio()
        except ImportError:
            self.logger.warning("gpiod模块未安装，补光灯控制不可用")
        except Exception as e:
            self.logger.warning(f"补光灯GPIO初始化失败: {e}")

        if success:
            self.logger.info("CameraManager 初始化成功")
        else:
            self.logger.error("CameraManager 初始化失败: 前视摄像头不可用")

        return success

    def _init_led_gpio(self):
        """初始化补光灯GPIO (使用libgpiod)"""
        try:
            import gpiod
            self._chip = gpiod.Chip('gpiochip0')

            # 配置前视补光灯
            self._front_led_line = self._chip.get_line(self.front_led_pin)
            self._front_led_line.request(
                consumer='camera_manager',
                type=gpiod.LINE_REQ_DIR_OUT
            )
            self._front_led_line.set_value(0)

            # 配置侧视补光灯
            self._side_led_line = self._chip.get_line(self.side_led_pin)
            self._side_led_line.request(
                consumer='camera_manager',
                type=gpiod.LINE_REQ_DIR_OUT
            )
            self._side_led_line.set_value(0)

            self._led_initialized = True
            self.logger.info("补光灯GPIO初始化成功")

        except Exception as e:
            self.logger.error(f"GPIO初始化失败: {e}")
            self._led_initialized = False

    def get_frames(self) -> Dict[str, Optional[np.ndarray]]:
        """
        获取所有摄像头的最新帧

        Returns:
            dict: {
                "front": np.ndarray or None,
                "side": np.ndarray or None,
                "face": np.ndarray or None
            }
        """
        frames = {
            "front": None,
            "side": None,
            "face": None,
        }

        if self.front_cam is not None:
            frames["front"] = self.front_cam.get_frame()

        if self.side_cam is not None:
            frames["side"] = self.side_cam.get_frame()

        if self.face_cam is not None:
            frames["face"] = self.face_cam.get_frame()

        return frames

    def get_front_frame(self) -> Optional[np.ndarray]:
        """获取前视摄像头帧"""
        if self.front_cam is not None:
            return self.front_cam.get_frame()
        return None

    def get_side_frame(self) -> Optional[np.ndarray]:
        """获取侧视摄像头帧 (当前返回 None)"""
        if self.side_cam is not None:
            return self.side_cam.get_frame()
        return None

    def get_face_frame(self) -> Optional[np.ndarray]:
        """获取人脸识别摄像头帧 (当前返回 None，后续扩展)"""
        if self.face_cam is not None:
            return self.face_cam.get_frame()
        return None

    def set_led(self, camera: str, brightness: int):
        """
        设置补光灯亮度

        Args:
            camera: "front" 或 "side"
            brightness: 0-255 (目前只支持开关: 0=关, >0=开)
        """
        if not self._led_initialized:
            return

        try:
            value = 1 if brightness > 0 else 0
            if camera == "front" and hasattr(self, '_front_led_line'):
                self._front_led_line.set_value(value)
                self.logger.debug(f"前视补光灯: {'开启' if value else '关闭'}")
            elif camera == "side" and hasattr(self, '_side_led_line'):
                self._side_led_line.set_value(value)
                self.logger.debug(f"侧视补光灯: {'开启' if value else '关闭'}")
        except Exception as e:
            self.logger.error(f"补光灯控制失败: {e}")

    def get_fps(self) -> Dict[str, float]:
        """获取所有摄像头的实际FPS"""
        return {
            "front": self.front_cam.get_fps() if self.front_cam else 0.0,
            "side": self.side_cam.get_fps() if self.side_cam else 0.0,
            "face": self.face_cam.get_fps() if self.face_cam else 0.0,
        }

    def release(self):
        """释放所有资源"""
        self.logger.info("CameraManager 释放资源...")

        if self.front_cam is not None:
            self.front_cam.stop()
            self.front_cam = None

        if self.side_cam is not None:
            self.side_cam.stop()
            self.side_cam = None

        if self.face_cam is not None:
            self.face_cam.stop()
            self.face_cam = None

        # 关闭补光灯
        if self._led_initialized:
            try:
                if hasattr(self, '_front_led_line'):
                    self._front_led_line.set_value(0)
                    self._front_led_line.release()
                if hasattr(self, '_side_led_line'):
                    self._side_led_line.set_value(0)
                    self._side_led_line.release()
                if hasattr(self, '_chip'):
                    self._chip.close()
            except Exception as e:
                self.logger.error(f"GPIO释放失败: {e}")

        self.logger.info("CameraManager 资源已释放")

    # ============================================================
    # 人脸识别摄像头扩展接口 (预留)
    # ============================================================

    def enable_face_camera(self, device_id: int = 2) -> bool:
        """
        动态启用人脸识别摄像头 (运行时扩展)

        Args:
            device_id: 摄像头设备ID

        Returns:
            bool: 是否成功启用
        """
        if self.face_cam is not None:
            self.logger.warning("人脸识别摄像头已在运行")
            return True

        self.face_config = CameraConfig(
            device_id=device_id,
            width=1920,
            height=1080,
            fps=30,
            name="face"
        )
        self.face_cam = CameraThread(self.face_config, self.logger)
        if self.face_cam.open():
            self.face_cam.start()
            self._enable_face = True
            self.logger.info("人脸识别摄像头已启用")
            return True
        else:
            self.logger.error("人脸识别摄像头启用失败")
            self.face_cam = None
            return False

    def disable_face_camera(self):
        """动态禁用人脸识别摄像头"""
        if self.face_cam is not None:
            self.face_cam.stop()
            self.face_cam = None
            self._enable_face = False
            self.logger.info("人脸识别摄像头已禁用")
