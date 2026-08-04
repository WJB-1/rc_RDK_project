"""
日志工具模块 - 统一日志记录系统
提供格式化的日志输出，支持文件和控制台双输出
"""

import logging
import sys
import os
from datetime import datetime
from pathlib import Path
from typing import Optional


class Logger:
    """统一日志管理类"""
    
    _instance: Optional['Logger'] = None
    _initialized = False
    
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(
        self,
        name: str = "robocup_rescue",
        level: str = "INFO",
        log_dir: str = "./logs",
        console_output: bool = True,
        file_output: bool = True
    ):
        if Logger._initialized:
            return
            
        self.name = name
        self.level = getattr(logging, level.upper(), logging.INFO)
        self.log_dir = Path(log_dir)
        self.console_output = console_output
        self.file_output = file_output
        
        # 创建日志目录
        if self.file_output:
            self.log_dir.mkdir(parents=True, exist_ok=True)
        
        # 配置日志器
        self.logger = logging.getLogger(name)
        self.logger.setLevel(self.level)
        self.logger.handlers = []  # 清除已有处理器
        
        # 日志格式
        formatter = logging.Formatter(
            '[%(asctime)s] [%(levelname)-8s] [%(name)s] - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        # 控制台输出
        if self.console_output:
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setLevel(self.level)
            console_handler.setFormatter(formatter)
            self.logger.addHandler(console_handler)
        
        # 文件输出
        if self.file_output:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_file = self.log_dir / f"{name}_{timestamp}.log"
            file_handler = logging.FileHandler(log_file, encoding='utf-8')
            file_handler.setLevel(self.level)
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)
            
        Logger._initialized = True
        self.info(f"日志系统初始化完成 | 级别: {level} | 目录: {log_dir}")
    
    def debug(self, msg: str):
        """调试日志"""
        self.logger.debug(msg)
    
    def info(self, msg: str):
        """信息日志"""
        self.logger.info(msg)
    
    def warning(self, msg: str):
        """警告日志"""
        self.logger.warning(msg)
    
    def error(self, msg: str):
        """错误日志"""
        self.logger.error(msg)
    
    def critical(self, msg: str):
        """严重错误日志"""
        self.logger.critical(msg)
    
    def exception(self, msg: str):
        """异常日志 (自动捕获异常堆栈)"""
        self.logger.exception(msg)


# 全局日志实例
_log_instance: Optional[Logger] = None


def get_logger(
    name: str = "robocup_rescue",
    level: str = "INFO",
    log_dir: str = "./logs"
) -> Logger:
    """
    获取日志实例 (单例模式)
    
    Args:
        name: 日志器名称
        level: 日志级别
        log_dir: 日志文件目录
        
    Returns:
        Logger实例
    """
    global _log_instance
    if _log_instance is None:
        _log_instance = Logger(name, level, log_dir)
    return _log_instance


# 便捷函数
def debug(msg: str):
    """输出调试日志"""
    get_logger().debug(msg)


def info(msg: str):
    """输出信息日志"""
    get_logger().info(msg)


def warning(msg: str):
    """输出警告日志"""
    get_logger().warning(msg)


def error(msg: str):
    """输出错误日志"""
    get_logger().error(msg)


def critical(msg: str):
    """输出严重错误日志"""
    get_logger().critical(msg)


def exception(msg: str):
    """输出异常日志"""
    get_logger().exception(msg)