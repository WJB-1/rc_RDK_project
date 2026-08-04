"""
config/ — 全局配置模块

从 settings.yaml 加载所有可调参数。
提供类型安全的配置访问。
"""

import json
from pathlib import Path
from typing import Any, Dict

_CONFIG_PATH = Path(__file__).parent / "settings.yaml"
_cache: Dict[str, Any] = {}


def _load() -> Dict[str, Any]:
    global _cache
    if not _cache:
        try:
            import yaml
            with open(_CONFIG_PATH, 'r', encoding='utf-8') as f:
                _cache = yaml.safe_load(f) or {}
        except ImportError:
            # fallback: 用 json 手动解析简单 yaml（仅支持嵌套 dict）
            _cache = _parse_yaml_simple(_CONFIG_PATH)
    return _cache


def _parse_yaml_simple(path: Path) -> Dict[str, Any]:
    """极简 YAML 解析器 — 仅用于 pyyaml 缺失时的兜底"""
    import re
    result = {}
    stack = [(result, -1)]
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            stripped = line.rstrip()
            if not stripped or stripped.startswith('#'):
                continue
            indent = len(line) - len(line.lstrip())
            key_val = stripped.split(':', 1)
            if len(key_val) != 2:
                continue
            key = key_val[0].strip().strip('"').strip("'")
            raw_val = key_val[1].strip().strip('"').strip("'")
            # strip inline comment
            raw_val = re.sub(r'\s+#.*$', '', raw_val)
            # pop deeper
            while stack and stack[-1][1] >= indent:
                stack.pop()
            parent = stack[-1][0]
            if raw_val == '' or raw_val == '{}':
                sub = {}
                parent[key] = sub
                stack.append((sub, indent))
            else:
                try:
                    parent[key] = float(raw_val) if '.' in raw_val else int(raw_val)
                except ValueError:
                    parent[key] = raw_val
    return result


def get(key: str, default: Any = None) -> Any:
    """获取配置项: get('state_machine.edge_stall_timeout_s')"""
    d = _load()
    for part in key.split('.'):
        if isinstance(d, dict) and part in d:
            d = d[part]
        else:
            return default
    return d


def get_track_config() -> Dict[str, float]:
    return get('track', {})


def get_edge_defaults() -> Dict[str, Any]:
    return get('edge.defaults', {})


def get_edge_tunnel() -> Dict[str, Any]:
    return get('edge.tunnel', {})


def get_state_machine_config() -> Dict[str, Any]:
    return get('state_machine', {})


def get_expected_yaw(name: str) -> float:
    return get(f'expected_yaw.{name}', 0.0)


def reload():
    """强制重新加载配置文件"""
    global _cache
    _cache = {}
    _load()
