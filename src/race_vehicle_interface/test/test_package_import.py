import unittest


class RaceVehicleInterfaceImportTest(unittest.TestCase):

    def test_mapping_module_imports(self):
        from race_vehicle_interface import command_mapping

        self.assertTrue(hasattr(command_mapping, "speed_to_stage"))


if __name__ == "__main__":
    unittest.main()
