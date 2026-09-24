from runner import DebugRunner


WEB_PORT = 5002


def create_runner():
    return DebugRunner(
        serial_port="",
        baudrate=0,
        web_port=WEB_PORT,
        enable_vision_recording=True,
        vision_only=True,
    )


def main():
    runner = create_runner()
    try:
        runner.start()
    finally:
        runner.shutdown()


if __name__ == "__main__":
    main()
