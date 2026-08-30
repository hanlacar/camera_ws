import unittest


class RacePerceptionImportTest(unittest.TestCase):

    def test_segmentation_module_imports(self):
        from race_perception import segmentation_path

        self.assertTrue(hasattr(segmentation_path, "centerline_from_mask"))


if __name__ == "__main__":
    unittest.main()
