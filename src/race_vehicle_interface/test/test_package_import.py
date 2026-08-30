import unittest


class RaceVehicleInterfaceImportTest(unittest.TestCase):

    def test_protocol_modules_import(self):
        from race_vehicle_interface import command_mapping
        from race_vehicle_interface import serial_protocol

        self.assertTrue(hasattr(command_mapping, "camera_drive_to_stage"))
        self.assertTrue(hasattr(serial_protocol, "parse_telemetry"))


if __name__ == "__main__":
    unittest.main()
