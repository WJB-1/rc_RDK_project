#!/usr/bin/env python3
"""
RoboCup Rescue Brain - 程序入口

职责:
- 加载全局配置
- 初始化并拉起摄像头线程
- 初始化通讯线程 (STM32)
- 初始化导航状态机
- 运行核心状态机主循环
- 推送调试数据到 Web 可视化服务器

硬件平台: RDK X5 (上位机) + STM32 (下位机)
"""

import os
import sys
import time
import math
import signal
import threading
from pathlib import Path

# 添加项目路径（兼容直接运行和作为模块导入）
PROJECT_ROOT = Path(__file__).parent
sys.path.insert(0, str(PROJECT_ROOT))
# 如果项目目录名是 robocup_rescue_brain，确保父目录也在路径中
# 这样 import robocup_rescue_brain.xxx 才能找到
if PROJECT_ROOT.name == "robocup_rescue_brain":
    sys.path.insert(0, str(PROJECT_ROOT.parent))

import yaml

from utils.logger import get_logger
from hardware.camera import CameraManager
from perception.lane_tracker import LaneTracker
from perception.perception_adapter import culvert_detection_to_event, obstacle_detection_to_event
from web import WebPushServer
from navigation.state_machine import AgentStateMachine
from navigation.map_topology import get_topology
from communication.robot_bridge import RobotBridge
from vision.algorithms.crossroad_seg import detect_crossroad_from_seg, confirm_crossroad_seg, estimate_distance as seg_estimate_distance


class RescueBrain:
    """
    救援机器人大脑主类

    整合所有模块，运行主控制循环
    """

    def __init__(self, config_path: str = "config/settings.yaml"):
        """
        初始化机器人大脑

        Args:
            config_path: 配置文件路径
        """
        self.logger = get_logger()
        self.config_path = PROJECT_ROOT / config_path
        self.settings: dict = {}

        # 模块实例
        self.camera_manager: CameraManager = None
        self.lane_tracker: LaneTracker = None
        self.web: WebPushServer = None
        self.agent: AgentStateMachine = None
        self.bridge: RobotBridge = None
        self._serial = None  # pyserial 串口对象
        self._vision_tools = None  # VisionToolsImpl 实例，供感知线程使用

        # 运行状态
        self._running = False
        self._main_loop_count = 0
        self._last_fps_time = time.time()
        self._actual_fps = 0.0

        # 当前控制输出
        self.current_offset = 0.0
        self.is_intersection = False

        # 串口节流：避免每帧都写串口造成堵塞
        self._last_sent_offset = 0.0
        self._last_offset_send_time = 0.0
        self._serial_lock = threading.Lock()        # 全局串口发送锁
        self._last_serial_send_time = 0.0           # 上次发送时间
        self._serial_min_interval = 0.020           # 最小发送间隔 20ms（匹配下位机周期）

        self.logger.info("=" * 50)
        self.logger.info("RoboCup Rescue Brain 初始化中...")
        self.logger.info("=" * 50)

    def load_config(self) -> bool:
        """加载配置文件"""
        try:
            if not self.config_path.exists():
                self.logger.error(f"配置文件不存在: {self.config_path}")
                return False

            with open(self.config_path, 'r', encoding='utf-8') as f:
                self.settings = yaml.safe_load(f)

            self.logger.info(f"配置文件加载成功: {self.config_path}")
            return True

        except Exception as e:
            self.logger.exception(f"加载配置文件失败: {e}")
            return False

    def initialize(self) -> bool:
        """
        初始化所有模块

        Returns:
            bool: 初始化是否成功
        """
        try:
            # 1. 加载配置
            if not self.load_config():
                self.logger.error("配置加载失败，使用默认配置")
                self.settings = self._get_default_config()

            # 2. 初始化导航状态机
            # 注入 vision 工具（如果有 YOLO 模型则加载）
            self.logger.info("[1/5] 初始化导航状态机...")
            self.agent = AgentStateMachine()
            self._init_vision_tools()

            # 3. 初始化摄像头管理器
            self.logger.info("[2/5] 初始化摄像头管理器...")
            self.camera_manager = CameraManager(self.settings)
            if not self.camera_manager.initialize():
                self.logger.error("摄像头管理器初始化失败")
                # 继续运行，可能使用模拟数据

            # 4. 初始化巡线追踪器
            self.logger.info("[3/5] 初始化巡线追踪器...")
            self.lane_tracker = LaneTracker(self.settings)

            # 5. 初始化 STM32 通讯桥接
            self.logger.info("[4/5] 初始化 STM32 通讯桥接...")
            self.bridge = RobotBridge(agent=self.agent)

            # 尝试连接实际串口
            serial_cfg = self.settings.get('serial', {})
            port = serial_cfg.get('port', '/dev/ttyUSB0')
            baudrate = serial_cfg.get('baudrate', 115200)
            try:
                import serial
                self._serial = serial.Serial(
                    port=port,
                    baudrate=baudrate,
                    timeout=serial_cfg.get('timeout', 0.01),
                    write_timeout=serial_cfg.get('write_timeout', 0.01),
                )

                def serial_send(data: bytes):
                    """串口发送：加锁 + 节流，匹配下位机 20ms 处理周期"""
                    with self._serial_lock:
                        # 节流：距上次发送不足 20ms 则等待
                        now = time.time()
                        elapsed = now - self._last_serial_send_time
                        if elapsed < self._serial_min_interval:
                            time.sleep(self._serial_min_interval - elapsed)
                        n = self._serial.write(data)
                        self._last_serial_send_time = time.time()
                        return n

                self.bridge.set_serial_send(serial_send)
                # 手动调试模式下不启动 bridge 的 50Hz 自动循环
                # bridge.start() 会导致 agent.tick() 与下位机回传竞争串口
                self.logger.info(f"串口已连接: {port} @ {baudrate}bps（手动模式，串口写间隔≥20ms）")
            except Exception as e:
                self.logger.warning(f"串口连接失败 ({port}): {e}")
                self.logger.warning("将在模拟模式下运行 (无真实硬件)")
                self._serial = None

            # 6. 初始化 Web 调试服务器
            self.logger.info("[5/5] 初始化 Web 调试服务器...")
            web_cfg = self.settings.get('web_debug', {})
            if web_cfg.get('enabled', True):
                self.web = WebPushServer(
                    host=web_cfg.get('host', '0.0.0.0'),
                    port=web_cfg.get('port', 5000),
                    cmd_callback=self._on_debug_cmd,
                )
                topo = get_topology()
                self.web.set_map_topology(
                    nodes={name: node.to_dict() for name, node in topo.nodes.items()},
                    edges=[edge.to_dict() for edge in topo.edges],
                )
                # 预生成 TSP 巡逻路径等静态数据注入前端
                try:
                    from navigation.map_oracle import MapOracle
                    from navigation.map_config import MISSION_NODES, NODE_COORDS, EXPECTED_YAW, LANE_WIDTH_MM
                    oracle = MapOracle(topo)
                    patrol_path = oracle.query_shortest_path("START", list(MISSION_NODES))
                    self.web.set_base_map_data({
                        "patrol_path": patrol_path,
                        "lane_width_mm": LANE_WIDTH_MM,
                        "field_size_mm": [3200, 4400],
                        "expected_yaw": EXPECTED_YAW,
                        "node_coords": {k: dict(v) for k, v in NODE_COORDS.items()},
                    })
                except Exception as e:
                    self.logger.warning(f"base_map_data 注入失败: {e}")
                self.web.start()
                self.logger.info(
                    f"Web 面板: http://{web_cfg.get('host', '0.0.0.0')}:{web_cfg.get('port', 5000)}"
                )
            else:
                self.web = None

            self.logger.info("=" * 50)
            self.logger.info("所有模块初始化完成")
            self.logger.info("=" * 50)
            return True

        except Exception as e:
            self.logger.exception(f"初始化失败: {e}")
            return False

    def _init_vision_tools(self):
        """
        初始化 vision tools (YOLO 检测引擎)

        模型路径读取顺序:
        1. 环境变量 YOLO_MODEL_PATH
        2. 配置文件 yolo.model_path
        3. 默认路径 models/yolov8_detection_x5.bin

        加载失败时优雅降级: 记录警告，_vision_tools 保持为 None。
        """
        yolo_path = os.environ.get("YOLO_MODEL_PATH", "")
        if not yolo_path:
            yolo_path = self.settings.get("yolo", {}).get("model_path", "")
        if not yolo_path:
            yolo_path = "models/yolov8_detection_x5.bin"

        # 转为绝对路径
        yolo_path_full = str(PROJECT_ROOT / yolo_path) if not os.path.isabs(yolo_path) else yolo_path

        try:
            from vision.tools import VisionToolsImpl

            if not Path(yolo_path_full).exists():
                self.logger.warning(f"YOLO 模型文件不存在: {yolo_path_full}，跳过 vision tools 加载")
                return

            vision = VisionToolsImpl(yolo_model_path=yolo_path_full)
            self._vision_tools = vision
            self.agent.set_vision_tools(vision)
            self.logger.info(f"Vision tools 已注入 (模型: {yolo_path_full})")
        except ImportError as e:
            self.logger.warning(f"Vision tools 依赖缺失 (非 RDK X5 环境？): {e}")
        except Exception as e:
            self.logger.warning(f"Vision tools 加载失败: {e}")

    def _get_default_config(self) -> dict:
        """获取默认配置"""
        return {
            'cameras': {
                'front': {'device_id': 0, 'width': 1920, 'height': 1080, 'fps': 30},
                'side': {'device_id': 1, 'width': 1920, 'height': 1080, 'fps': 30}
            },
            'lane_tracking': {
                'roi': {'y_start': 0.6, 'y_end': 0.9, 'x_start': 0.1, 'x_end': 0.9},
                'hsv_lower': [0, 0, 200],
                'hsv_upper': [180, 30, 255],
                'blur_kernel': 5
            },
            'debug': {'show_video': True, 'log_level': 'INFO'}
        }

    def _signal_handler(self, signum, frame):
        """信号处理函数"""
        self.logger.info(f"收到信号 {signum}，准备退出...")
        self.stop()

    def _on_debug_cmd(self, cmd: str, payload: dict):
        """
        处理调试面板发送的手动控制指令。
        支持手动控制 + 自动模式切换。
        """
        self.logger.info(f"[调试面板] 收到指令: {cmd}, 参数: {payload}")

        if cmd == 'set_mode':
            self._manual_mode = (payload.get('mode', 'manual') == 'manual')
            self.logger.info(f"模式: {'手动' if self._manual_mode else '自动'}")
            return

        # --- 自动模式 ---
        if cmd == 'auto_mode_start':
            self._manual_mode = False
            self.logger.info("[自动模式] 启动状态机 + 路径规划")
            if self.agent:
                self.agent.start()
            if self.bridge and self._serial is not None:
                self.bridge.start()
            return

        if cmd == 'auto_mode_stop':
            self._manual_mode = True
            self.logger.info("[手动模式] 停止状态机 tick")
            if self.bridge:
                self.bridge.stop()
            return

        from communication import ActionCode, encode_action

        if self.bridge is None:
            self.logger.warning("STM32 桥接层未初始化，无法发送指令")
            return

        # --- 持续运动 (按下走，松开发 stop) ---
        if cmd == 'forward':
            vx = payload.get('vx', 300)
            self.logger.info(f"[手动控制] 前进 vx={vx}")
            self.bridge.send_velocity(vx, 0)
        elif cmd == 'backward':
            vx = payload.get('vx', 300)
            self.logger.info(f"[手动控制] 后退 vx={vx}")
            self.bridge.send_velocity(-vx, 0)
        elif cmd == 'left':
            # 原地左转: 先急停再发 CMD_TURN_IMU +90
            angle = payload.get('angle', 90)
            self.logger.info(f"[手动控制] 原地左转 angle={angle}")
            self.bridge._serial_send(encode_action(ActionCode.STOP))
            self.bridge.send_turn_imu(angle)
        elif cmd == 'right':
            # 原地右转: 先急停再发 CMD_TURN_IMU -90
            angle = payload.get('angle', 90)
            self.logger.info(f"[手动控制] 原地右转 angle={angle}")
            self.bridge._serial_send(encode_action(ActionCode.STOP))
            self.bridge.send_turn_imu(-angle)

        # --- 离散指令 (点击即执行) ---
        elif cmd == 'stop':
            self.logger.info("[手动控制] 发送急停指令")
            self.bridge._serial_send(encode_action(ActionCode.STOP))
        elif cmd == 'reset_odom':
            self.logger.info("[手动控制] 发送里程清零指令")
            self.bridge._serial_send(encode_action(ActionCode.RESET_ODOM))
        elif cmd == 'turn_imu':
            # 原地 IMU 转向 (CMD_TURN_IMU 0x03)
            angle = payload.get('angle', 90)
            self.logger.info(f"[手动控制] 原地转向 angle={angle}")
            self.bridge._serial_send(encode_action(ActionCode.STOP))
            self.bridge.send_turn_imu(angle)
        elif cmd == 'intersection_turn':
            # 路口边走边转 (CMD_INTERSECTION_TURN 0x05)
            direction = payload.get('direction', 'left')
            self.logger.info(f"[手动控制] 路口边走边转 direction={direction}")
            self.bridge.send_intersection_turn(direction)

    def _push_debug_data(self, seg_frame, offset_mm, is_intersection):
        """推送数据到 Web 面板"""
        if self.web is None:
            return

        self.web.update(
            seg_frame=seg_frame,
            offset_mm=offset_mm,
            is_intersection=is_intersection,
        )

        x, y, yaw = self.agent.get_position()
        visited = list(self.agent.visited_nodes)
        progress = self.agent.topo.get_mission_progress()

        self.web.update_navigation(
            agent_state=self.agent.get_state_name(),
            position=(x, y, yaw),
            current_node=self.agent.current_node,
            target_node=self.agent.target_node,
            planned_path=self.agent.planned_path,
            visited_nodes=visited,
            progress={"visited": progress[0], "total": progress[1]},
        )

        if self.agent.event_log:
            self.web.update_navigation(event=self.agent.event_log[-1])

    def apply_vision_yaw_correction(self):
        """
        视觉偏转角微调 — 可选工具，由调度层按需调用。

        原理：摄像头检测到的车道线偏角 (lane_angle_rad) 反映车头方向偏差。
        用 5% 低通滤波修正 agent.yaw_deg，避免单帧抖动。

        适用场景：IMU 不可用或需要视觉冗余时。
        注意：正常情况下航向修正由下位机 IMU 闭环完成 (move_4.c)。
        """
        lane_state = self.lane_tracker.last_lane_state
        if lane_state is None or lane_state.get("frame_dropped", False):
            return
        angle_rad = lane_state.get("lane_angle_rad", 0.0)
        quality = lane_state.get("quality_score", 0.0)
        if quality > 0.5 and abs(angle_rad) > 0.01:
            self.agent.yaw_deg += math.degrees(angle_rad) * 0.05

    def _validate_wall_detection(self, detection):
        """
        YOLO 检测到"墙壁"(标签3) → 查地图校验
        Returns: "tunnel" (隧道侧墙, 忽略) or "culvert" (新涵洞, 标记边) or "ignore"
        """
        if self.agent is None:
            return "ignore"
        current_edge = self.agent.executor.current_task
        if current_edge is None:
            return "ignore"
        if current_edge.is_tunnel:
            return "tunnel"  # 隧道侧墙，正常
        return "culvert"

    def _perception_loop(self):
        """
        感知层处理循环
        运行在独立线程，持续处理视觉数据

        异常隔离原则: 每个检测模块有独立的 try/except，
        确保一个模块的异常不影响其他模块。
        """
        self.logger.info("感知层线程启动")

        while self._running:
            try:
                frames = self.camera_manager.get_frames()
                front_frame = frames.get("front")

                if front_frame is None:
                    time.sleep(0.001)
                    continue

                # ---- 1. 车道巡线 + 偏移发送 (独立异常块) ----
                try:
                    offset_mm, is_intersection, debug_frame = self.lane_tracker.process(front_frame)
                    self.current_offset = offset_mm
                    self.is_intersection = is_intersection

                    # 发送车道偏移量到下位机（最低优先级，大幅节流）
                    if self.bridge and self._serial is not None:
                        now = time.time()
                        delta_offset = abs(offset_mm - self._last_sent_offset)
                        delta_time = now - self._last_offset_send_time
                        if (delta_offset > 10.0 or delta_time > 0.2) and delta_time > 0.05:
                            self.bridge.send_lane_offset(offset_mm)
                            self._last_sent_offset = offset_mm
                            self._last_offset_send_time = now

                    # 推送调试数据
                    self._push_debug_data(debug_frame, offset_mm, is_intersection)
                except Exception as e:
                    self.logger.exception(f"车道巡线/偏移发送异常: {e}")

                # ---- 2. 分割路口检测 (独立异常块) ----
                try:
                    seg_is_intersection = False
                    seg_distance = 150
                    if self.lane_tracker.last_seg_mask is not None:
                        h, w = self.lane_tracker.last_seg_mask.shape[:2]
                        roi_y1 = h * 2 // 3
                        roi_y2 = h
                        seg_is_cross, seg_dist, seg_duty = detect_crossroad_from_seg(
                            self.lane_tracker.last_seg_mask,
                            roi_y1=roi_y1, roi_y2=roi_y2
                        )
                        if seg_is_cross:
                            confirmed, _counter = confirm_crossroad_seg(seg_is_cross)
                            if confirmed:
                                seg_is_intersection = True
                                if seg_dist > 0:
                                    seg_distance = seg_dist

                    if is_intersection or seg_is_intersection:
                        lane_state = self.lane_tracker.last_lane_state
                        if seg_is_intersection:
                            distance = seg_distance
                        elif lane_state is not None:
                            distance = lane_state.get("distance_to_crossroad_mm", -1.0)
                            if distance <= 0 or distance > 2000:
                                distance = 300
                        else:
                            distance = 300
                        from navigation.contracts import CrossroadEvent
                        self.agent.on_crossroad_detected(
                            CrossroadEvent(distance_mm=distance, duty_cycle=0.9))
                except Exception as e:
                    self.logger.exception(f"路口检测异常: {e}")

                # ---- 3. YOLO 涵洞/墙壁/入口检测 (M2+M3, 独立异常块) ----
                try:
                    if self._vision_tools is not None:
                        culvert_result = self._vision_tools.detect_culvert(
                            front_frame,
                            is_tunnel=self.agent.executor.current_task.is_tunnel
                                      if self.agent and self.agent.executor.current_task else False
                        )
                        if culvert_result.detected:
                            # 墙壁检测（标签3）→ 地图校验
                            if culvert_result.is_wall_detection:
                                wall_type = self._validate_wall_detection(culvert_result)
                                if wall_type == "culvert":
                                    culvert_event = culvert_detection_to_event(
                                        culvert_result, "side")
                                    self.agent.on_culvert_detected(culvert_event)
                                elif wall_type == "tunnel":
                                    self.logger.info("隧道侧墙检测，忽略")
                            else:
                                # M3: 涵洞口/隧道口入口检测
                                entrance_boxes = [b for b in culvert_result.boxes
                                                  if b.class_id in (1, 2)]
                                culvert_boxes = [b for b in entrance_boxes
                                                 if b.class_id == 1]
                                tunnel_boxes = [b for b in entrance_boxes
                                                if b.class_id == 2]
                                if culvert_boxes:
                                    best = max(culvert_boxes, key=lambda b: b.confidence)
                                    culvert_event = culvert_detection_to_event(
                                        culvert_result, "front")
                                    culvert_event.confidence = best.confidence
                                    self.agent.on_culvert_entrance_detected(culvert_event)
                                if tunnel_boxes:
                                    self.logger.info("隧道口检测（标签2），仅记录，不动作")
                except Exception as e:
                    self.logger.exception(f"YOLO 涵洞/墙壁/入口检测异常: {e}")

                # ---- 4. 障碍物检测 (M4, 独立异常块) ----
                try:
                    if self._vision_tools is not None:
                        seg_mask = self.lane_tracker.last_seg_mask
                        obstacle_result = self._vision_tools.detect_obstacle(
                            front_frame, seg_mask=seg_mask
                        )
                        if obstacle_result.detected and obstacle_result.in_lane:
                            obs_event = obstacle_detection_to_event(obstacle_result)
                            self.agent.on_obstacle_detected(obs_event)
                except Exception as e:
                    self.logger.exception(f"障碍物检测异常: {e}")

                # 短暂休眠，避免CPU占用过高
                time.sleep(0.001)

            except Exception as e:
                self.logger.exception(f"感知层主循环异常(获取帧等): {e}")
                time.sleep(0.01)

        self.logger.info("感知层线程停止")

    def _serial_rx_loop(self):
        """
        串口接收线程 — 持续读取 STM32 反馈数据
        """
        self.logger.info("串口接收线程启动")
        while self._running and self._serial is not None:
            try:
                if self._serial.in_waiting > 0:
                    data = self._serial.read(self._serial.in_waiting)
                    if data and self.bridge:
                        self.bridge.on_stm32_data(data)
                time.sleep(0.001)  # 1ms 轮询
            except Exception as e:
                self.logger.exception(f"串口接收异常: {e}")
                time.sleep(0.1)
        self.logger.info("串口接收线程停止")

    def _decision_loop(self):
        """
        决策层主循环
        高频运行 (目标50Hz)，输出控制指令
        【当前阶段: 纯手动模式，状态机和模拟里程计已禁用】
        """
        self.logger.info("决策层主循环启动")
        loop_count = 0
        last_time = time.time()

        while self._running:
            try:
                loop_start = time.time()

                # 自动模式：状态机 tick
                if not getattr(self, '_manual_mode', True):
                    action = self.agent.tick()
                    if action and self.bridge:
                        self.bridge.send_turn_command(action)

                # 模拟里程计（无真实 STM32 时启用）
                if not getattr(self, '_manual_mode', False) and loop_count % 10 == 0:
                    from navigation.contracts import OdomUpdate
                    self.agent.on_odom_update(OdomUpdate(dy_mm=5, dyaw_deg=0))

                loop_count += 1
                self._main_loop_count += 1

                # 计算实际循环频率
                current_time = time.time()
                if current_time - self._last_fps_time >= 1.0:
                    self._actual_fps = self._main_loop_count / (current_time - self._last_fps_time)
                    self._main_loop_count = 0
                    self._last_fps_time = current_time

                    # 打印状态
                    cam_fps = self.camera_manager.get_fps()
                    pos = self.agent.get_position()
                    self.logger.info(
                        f"系统状态 | 主循环: {self._actual_fps:.1f}Hz | "
                        f"前视: {cam_fps['front']:.1f}fps | "
                        f"位置: ({pos[0]:.0f}, {pos[1]:.0f}) | "
                        f"状态: {self.agent.get_state_name()} | "
                        f"偏移: {self.current_offset:+.1f}mm"
                    )

                # 控制循环频率 (20ms = 50Hz)
                elapsed = time.time() - loop_start
                sleep_time = max(0, 0.02 - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)

            except Exception as e:
                self.logger.exception(f"决策层异常: {e}")
                time.sleep(0.01)

        self.logger.info("决策层主循环停止")

    def run(self):
        """启动主程序"""
        # 注册信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        # 初始化
        if not self.initialize():
            self.logger.error("初始化失败，程序退出")
            return

        self._running = True

        # 启动 Agent 状态机
        self.agent.start()

        # 启动感知层线程
        perception_thread = threading.Thread(target=self._perception_loop, daemon=True)
        perception_thread.start()

        # 启动串口接收线程 (读取 STM32 反馈)
        if self._serial is not None:
            serial_rx_thread = threading.Thread(target=self._serial_rx_loop, daemon=True)
            serial_rx_thread.start()

        # 运行决策层主循环 (在主线程)
        self._decision_loop()

        # 等待感知层线程结束
        perception_thread.join(timeout=2.0)

    def stop(self):
        """停止程序"""
        self._running = False
        self.logger.info("正在停止程序...")

        # 停止桥接层
        if self.bridge:
            self.bridge.stop()

        # 关闭串口
        if self._serial is not None:
            try:
                self._serial.close()
                self.logger.info("串口已关闭")
            except Exception as e:
                self.logger.warning(f"关闭串口异常: {e}")

        # 释放资源
        if self.camera_manager is not None:
            self.camera_manager.release()

        # 关闭OpenCV窗口
        try:
            import cv2
            cv2.destroyAllWindows()
        except:
            pass

        self.logger.info("程序已停止")


def main():
    """程序入口"""
    print("=" * 60)
    print("RoboCup Rescue Brain - RDK X5 视觉系统")
    print("公共安全赛项 | 侦查机器人")
    print("=" * 60)

    brain = RescueBrain()

    try:
        brain.run()
    except KeyboardInterrupt:
        print("\n用户中断")
        brain.stop()
    except Exception as e:
        print(f"\n程序异常: {e}")
        import traceback
        traceback.print_exc()
        brain.stop()

    print("\n程序已退出")


if __name__ == "__main__":
    main()
