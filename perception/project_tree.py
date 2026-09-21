#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
project_tree.py —— 输出项目的完整目录架构（精确到所有子目录的每个文件）

示例：
    python project_tree.py                       # 输出当前目录
    python project_tree.py /path/to/proj         # 指定目录
    python project_tree.py -s -l                 # 带文件大小与代码行数
    python project_tree.py -m -o ARCH.md         # 输出 Markdown 到文件
    python project_tree.py --max-depth 3         # 限制展开深度
    python project_tree.py -e "*.log" "tmp*"     # 额外忽略
    python project_tree.py --ext .py .js         # 只看指定后缀
"""

from __future__ import annotations

import argparse
import fnmatch
import os
import sys
from dataclasses import dataclass
from pathlib import Path

# --------------------------------------------------------------------------- #
# 默认忽略规则（可用 --no-default-ignore 关闭，或用 -e 追加）
# --------------------------------------------------------------------------- #
DEFAULT_IGNORE_PATTERNS = [
    # 版本控制
    ".git", ".hg", ".svn",
    # 编辑器 / IDE
    ".idea", ".vscode", ".vs", "*.swp", "*.swo", "*~",
    # Python
    "__pycache__", "*.py[cod]", "*.egg-info", ".eggs",
    ".mypy_cache", ".pytest_cache", ".ruff_cache", ".tox",
    "venv", ".venv", "env", "site-packages", ".ipynb_checkpoints",
    # Node / 前端
    "node_modules", "bower_components", ".npm", ".yarn", ".pnpm-store",
    "dist", "build", ".next", ".nuxt", ".cache", ".parcel-cache",
    # 其他构建产物 / 覆盖率
    "target", "obj", "coverage", ".coverage", "htmlcov",
    # 系统文件
    ".DS_Store", "Thumbs.db", "desktop.ini",
]

# 统计行数时跳过的最大文件（避免卡在大文件上）
MAX_LINE_COUNT_BYTES = 2 * 1024 * 1024


# --------------------------------------------------------------------------- #
# 数据结构
# --------------------------------------------------------------------------- #
@dataclass
class Options:
    root_label: str = "."
    max_depth: int | None = None
    show_size: bool = False
    show_lines: bool = False
    only_dirs: bool = False
    include_ext: set[str] | None = None


@dataclass
class Stats:
    dirs: int = 0
    files: int = 0
    size: int = 0
    lines: int = 0


# --------------------------------------------------------------------------- #
# 工具函数
# --------------------------------------------------------------------------- #
def make_matcher(patterns):
    """把 glob 模式列表编译成一个 name -> bool 的匹配函数。"""
    pats = tuple(p for p in patterns if p)

    def _match(name: str) -> bool:
        return any(fnmatch.fnmatch(name, p) for p in pats)

    return _match


def human_size(num: float) -> str:
    units = ("B", "KB", "MB", "GB", "TB", "PB")
    i = 0
    while num >= 1024 and i < len(units) - 1:
        num /= 1024
        i += 1
    return f"{int(num)} B" if i == 0 else f"{num:.1f} {units[i]}"


def count_lines(path: Path, max_bytes: int = MAX_LINE_COUNT_BYTES):
    """统计文本文件行数；二进制 / 过大 / 读失败返回 None。"""
    try:
        size = path.stat().st_size
        if size == 0:
            return 0
        if size > max_bytes:
            return None
        with path.open("rb") as f:
            head = f.read(8192)
            if b"\x00" in head:          # 含 NUL 字节，基本可判定为二进制
                return None
            f.seek(0)
            return sum(1 for _ in f)
    except OSError:
        return None


def load_gitignore(root: Path) -> list[str]:
    """读取根目录 .gitignore，转成简易 glob 模式（不支持 ! 否定规则）。"""
    gi = root / ".gitignore"
    patterns: list[str] = []
    if not gi.is_file():
        return patterns
    try:
        for line in gi.read_text(encoding="utf-8", errors="ignore").splitlines():
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("!"):
                continue
            patterns.append(line.strip("/"))
    except OSError:
        pass
    return patterns


# --------------------------------------------------------------------------- #
# 核心：递归渲染目录树
# --------------------------------------------------------------------------- #
def render_children(directory, prefix, depth, out, stats, opts, ignore):
    """depth 为 directory 所在层级（根目录为 0）。"""
    if opts.max_depth is not None and depth >= opts.max_depth:
        return

    try:
        with os.scandir(directory) as it:
            entries = list(it)
    except OSError:
        out.append(f"{prefix}└── [无法访问]")
        return

    # 先筛选、再分类，最后排序（目录在前，同类按名称）
    items = []
    for entry in entries:
        name = entry.name
        if ignore(name):
            continue
        try:
            is_dir = entry.is_dir(follow_symlinks=False)
            is_link = entry.is_symlink()
        except OSError:
            continue

        if is_dir:
            items.append((True, False, name, entry))
        else:
            if opts.only_dirs:
                continue
            if opts.include_ext is not None:
                if os.path.splitext(name)[1].lower() not in opts.include_ext:
                    continue
            items.append((False, is_link, name, entry))

    items.sort(key=lambda t: (not t[0], t[2].lower()))

    for idx, (is_dir, is_link, name, entry) in enumerate(items):
        last = idx == len(items) - 1
        branch = "└── " if last else "├── "
        child_prefix = prefix + ("    " if last else "│   ")

        if is_dir:
            stats.dirs += 1
            out.append(f"{prefix}{branch}{name}/")
            render_children(
                entry.path, child_prefix, depth + 1, out, stats, opts, ignore
            )
        else:
            stats.files += 1
            meta = describe_file(entry, is_link, opts, stats)
            out.append(f"{prefix}{branch}{name}{meta}")


def describe_file(entry, is_link: bool, opts: Options, stats: Stats) -> str:
    """返回文件后面的附加信息，如 " (1.2 KB, 42 行)"。"""
    parts = []
    try:
        size = os.lstat(entry.path).st_size      # lstat：不跟随符号链接
    except OSError:
        size = 0
    stats.size += size

    if is_link:
        try:
            parts.append(f"→ {os.readlink(entry.path)}")
        except OSError:
            pass

    if opts.show_size:
        parts.append(human_size(size))

    if opts.show_lines and not is_link:
        n = count_lines(Path(entry.path))
        if n is None:
            parts.append("—")
        else:
            parts.append(f"{n} 行")
            stats.lines += n

    return f"  ({', '.join(parts)})" if parts else ""


def build_tree(root: Path, opts: Options, ignore):
    stats = Stats()
    lines = [f"{opts.root_label}/"]
    render_children(root, "", 0, lines, stats, opts, ignore)
    return lines, stats


# --------------------------------------------------------------------------- #
# 命令行
# --------------------------------------------------------------------------- #
def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="输出项目的完整目录架构（精确到所有子目录的每个文件）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("root", nargs="?", default=".", help="项目根目录，默认当前目录")
    p.add_argument("-o", "--output", help="写入文件（默认打印到终端）")
    p.add_argument("-m", "--markdown", action="store_true",
                   help="用 Markdown 代码块包裹输出")
    p.add_argument("-s", "--size", action="store_true", help="显示文件大小")
    p.add_argument("-l", "--lines", action="store_true",
                   help="显示文本文件行数（较慢）")
    p.add_argument("--max-depth", type=int, default=None,
                   help="最大展开深度（1 表示只列一层）")
    p.add_argument("-e", "--exclude", nargs="+", action="append", default=[],
                   help="额外忽略的模式，支持通配符，可多次指定")
    p.add_argument("--only-dirs", action="store_true", help="只输出目录")
    p.add_argument("--ext", nargs="+", default=None,
                   help="只显示指定扩展名的文件，如 --ext .py .js")
    p.add_argument("--no-default-ignore", action="store_true",
                   help="不启用内置忽略规则")
    p.add_argument("--gitignore", action="store_true",
                   help="把根目录 .gitignore 作为额外忽略规则")
    p.add_argument("--no-stats", action="store_true", help="不输出统计信息")
    p.add_argument("--root-label", default=None, help="根目录显示名称")
    return p.parse_args(argv)


def _setup_stdout():
    """尽量让 Windows 终端也能正确输出 UTF-8 的树形字符。"""
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass


def main(argv=None) -> int:
    _setup_stdout()
    args = parse_args(argv)

    root = Path(args.root).expanduser()
    if not root.exists():
        print(f"错误：路径不存在 {root}", file=sys.stderr)
        return 2
    if not root.is_dir():
        print(f"错误：{root} 不是目录", file=sys.stderr)
        return 2
    root = root.resolve()

    # ---- 组装忽略规则 ----
    patterns: list[str] = []
    if not args.no_default_ignore:
        patterns.extend(DEFAULT_IGNORE_PATTERNS)
    for group in args.exclude or []:
        patterns.extend(group)
    if args.gitignore:
        patterns.extend(load_gitignore(root))
    ignore = make_matcher(patterns)

    # ---- 组装选项 ----
    include_ext = None
    if args.ext:
        include_ext = {
            (e.lower() if e.startswith(".") else "." + e.lower()) for e in args.ext
        }

    opts = Options(
        root_label=args.root_label or (root.name or str(root)),
        max_depth=args.max_depth,
        show_size=args.size,
        show_lines=args.lines,
        only_dirs=args.only_dirs,
        include_ext=include_ext,
    )

    # ---- 生成 ----
    lines, stats = build_tree(root, opts, ignore)

    output: list[str] = []
    if args.markdown:
        output.append("```text")
        output.extend(lines)
        output.append("```")
    else:
        output.extend(lines)

    if not args.no_stats:
        summary = (
            f"合计：目录 {stats.dirs} 个，文件 {stats.files} 个，"
            f"总大小 {human_size(stats.size)}"
        )
        if args.lines:
            summary += f"，总行数 {stats.lines}"
        output.append("")
        output.append(summary)

    text = "\n".join(output) + "\n"

    if args.output:
        Path(args.output).write_text(text, encoding="utf-8")
        print(f"已写入：{args.output}", file=sys.stderr)
    else:
        sys.stdout.write(text)

    return 0


if __name__ == "__main__":
    sys.exit(main())