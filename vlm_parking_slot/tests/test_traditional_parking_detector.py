import unittest

import cv2
import numpy as np

from traditional_parking_detector import build_masks, detect_slots


class TraditionalParkingDetectorTest(unittest.TestCase):
    def test_masks_distinguish_bay_surface_road_and_white_paint(self):
        image = np.full((120, 240, 3), (146, 175, 146), dtype=np.uint8)
        image[20:100, 20:110] = (133, 115, 95)
        cv2.line(image, (110, 20), (110, 99), (216, 216, 216), 6)

        slot_mask, line_mask = build_masks(image)

        self.assertEqual(int(slot_mask[60, 60]), 255)
        self.assertEqual(int(slot_mask[60, 180]), 0)
        self.assertEqual(int(line_mask[60, 110]), 255)
        self.assertEqual(int(line_mask[60, 180]), 0)

    def test_returns_rotated_marked_bays_and_ignores_gray_object(self):
        image = np.full((500, 760, 3), (146, 175, 146), dtype=np.uint8)
        bay_color = (133, 115, 95)
        marking_color = (216, 216, 216)

        for center in ((200, 245), (540, 245)):
            corners = cv2.boxPoints((center, (180, 120), 17)).astype(np.int32)
            cv2.fillConvexPoly(image, corners, bay_color)
            cv2.polylines(image, [corners], True, marking_color, 8)

        narrow_corners = cv2.boxPoints(((380, 410), (180, 60), 17)).astype(np.int32)
        cv2.fillConvexPoly(image, narrow_corners, bay_color)
        cv2.polylines(image, [narrow_corners], True, marking_color, 8)

        # A gray pillar face is not a parking surface and must not become a slot.
        cv2.rectangle(image, (325, 180), (420, 310), (158, 158, 158), -1)

        accepted, rejected, _, _ = detect_slots(image)

        self.assertEqual(len(accepted), 2)
        self.assertEqual(len(rejected), 1)
        self.assertIn("slot_width_out_of_range", rejected[0]["rejection_reasons"])
        for slot in accepted:
            self.assertEqual(len(slot["corners_px"]), 4)
            self.assertGreaterEqual(slot["supported_edge_count"], 2)
            self.assertGreater(slot["aspect_ratio"], 1.5)


if __name__ == "__main__":
    unittest.main()
