import unittest


class StartConfigurationTests(unittest.TestCase):
    def test_usb_ttl_is_the_fixed_debug_serial_port(self):
        """Detects the debug entry point opening a board UART instead of USB-TTL."""
        from start import SERIAL_PORT

        self.assertEqual(SERIAL_PORT, "/dev/ttyS2")


if __name__ == "__main__":
    unittest.main()
