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
from web import WebPushServer
from navigation.state_machine import AgentStateMachine
from navigation.map_topology import get_topology
from communication.robot_bridge import RobotBridge


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

        # 运行状态
        self._running = False
        self._main_loop_count = 0
        self._last_fps_time = time.time()
        self._actual_fps = 0.0

        # 当前控制输出
        self.current_offset = 0.0
        self.is_intersection = False

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
            try:
                from vision.tools import VisionToolsImpl
                yolo_path = str(PROJECT_ROOT / "models" / "yolov8_detection_x5.bin")
                if Path(yolo_path).exists():
                    self.agent.set_vision_tools(
                        VisionToolsImpl(yolo_model_path=yolo_path)
                    )
                    self.logger.info("Vision tools 已注入")
            except Exception as e:
                self.logger.warning(f"Vision tools 注入失败: {e}")

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
                    """串口发送并立即 flush"""
                    written = self._serial.write(data)
                    self._serial.flush()
                    return written

                self.bridge.set_serial_send(serial_send)
                self.bridge.start()
                self.logger.info(f"串口已连接: {port} @ {baudrate}bps")
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
        当前阶段: 全部手动模式，不区分自动/手动。
        """
        self.logger.info(f"[调试面板] 收到指令: {cmd}, 参数: {payload}")

        if cmd == 'set_mode':
            # 模式切换暂存，后续可恢复自动逻辑
            self._manual_mode = (payload.get('mode', 'manual') == 'manual')
            self.logger.info(f"模式: {'手动' if self._manual_mode else '自动'}")
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

    def _perception_loop(self):
        """
        感知层处理循环
        运行在独立线程，持续处理视觉数据
        """
        self.logger.info("感知层线程启动")

        while self._running:
            try:
                frames = self.camera_manager.get_frames()
                front_frame = frames.get("front")

                if front_frame is not None:
                    offset_mm, is_intersection, debug_frame = self.lane_tracker.process(front_frame)
                    self.current_offset = offset_mm
                    self.is_intersection = is_intersection

                    # 发送车道偏移量到下位机
                    if self.bridge and self._serial is not None:
                        self.bridge.send_lane_offset(offset_mm)

                    # 视觉偏转角微调 (Phase 4)
                    # 直接小幅修正 yaw，不在 Agent 中维护独立方法
                    if not is_intersection:
                        lane_state = self.lane_tracker.last_lane_state
                        if lane_state is not None and not lane_state.get("frame_dropped", False):
                            angle_rad = lane_state.get("lane_angle_rad", 0.0)
                            quality = lane_state.get("quality_score", 0.0)
                            if quality > 0.5 and abs(angle_rad) > 0.01:
                                # 低通滤波: 只修正 5% 避免抖动
                                self.agent.yaw_deg += math.degrees(angle_rad) * 0.05

                    self._push_debug_data(debug_frame, offset_mm, is_intersection)

                    if is_intersection:
                        # 使用 IPM 真实测距，替代硬编码 150mm
                        lane_state = self.lane_tracker.last_lane_state
                        if lane_state is not None:
                            distance = lane_state.get("distance_to_crossroad_mm", -1.0)
                            if distance <= 0 or distance > 2000:
                                distance = 300  # 异常值兜底
                        else:
                            distance = 300
                        from navigation.contracts import CrossroadEvent
                        self.agent.on_crossroad_detected(
                            CrossroadEvent(distance_mm=distance, duty_cycle=0.9))

                # 短暂休眠，避免CPU占用过高
                time.sleep(0.001)

            except Exception as e:
                self.logger.exception(f"感知层处理异常: {e}")
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
