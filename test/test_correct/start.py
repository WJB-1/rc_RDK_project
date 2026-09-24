from runner import DebugRunner


SERIAL_PORT = "/dev/ttyS2"
BAUDRATE = 115200
WEB_PORT = 5002


def main():
    runner = DebugRunner(SERIAL_PORT, BAUDRATE, WEB_PORT, enable_vision_recording=True)
    try:
        runner.start()
    finally:
        runner.shutdown()


if __name__ == "__main__":
    main()
