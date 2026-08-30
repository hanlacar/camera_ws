import unittest


class RaceControlImportTest(unittest.TestCase):

    def test_controller_module_imports(self):
        from race_control import stabilized_path_follower

        self.assertTrue(hasattr(stabilized_path_follower, "StabilizedPathFollower"))


if __name__ == "__main__":
    unittest.main()
