"""启动 Navigation 2.0 仿真器与可视化 Dashboard。"""

import argparse

from navigation.simulation.composition import build_simulation_runner
from web.app import create_app


def main() -> None:
    """装配仿真导航闭环并启动 Flask 服务。"""

    parser = argparse.ArgumentParser(description="Navigation 2.0 simulation dashboard")
    parser.add_argument("--seed", type=int, default=0, help="仿真真值随机种子")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP 监听地址")
    parser.add_argument("--port", type=int, default=5000, help="HTTP 监听端口")
    args = parser.parse_args()

    runner = build_simulation_runner(seed=args.seed)
    app = create_app(runner)
    print("Navigation Dashboard: http://{}:{}/".format(args.host, args.port))
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
