import unittest
from visualization_msgs.msg import Marker, MarkerArray
from marker_encoder import marker_encode_ds,marker_encode_f_kds

"""
To run specific testing class, use '-k' to specify:
    python your_test_script.py -k TestMarkerEncodeDS
This is hte unittest for all functions in marker_encoder
"""
class TestMarkerEncodeDS(unittest.TestCase):

    def test_replace_ds(self):
        marker = Marker()
        marker.text = "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.0,0.0,0.0>, <kds= 0.0>"
        new_ds = 0.35

        updated_marker = marker_encode_ds(marker, new_ds)

        expected_text = "6.14 m/s , <ID= 4>, <ds= 0.3500>, <ss= 0.0,0.0,0.0>, <kds= 0.0>"
        self.assertEqual(updated_marker.text, expected_text)

    def test_no_ds(self):
        marker = Marker()
        marker.text = "6.14 m/s , <ID= 4>, <ss= 0.0,0.0,0.0>, <kds= 0.0>"
        new_ds = 0.35

        with self.assertRaises(AssertionError):
            marker_encode_ds(marker, new_ds)

class TestMarkerEncodeFKDS(unittest.TestCase):
    def test_replace_kds1(self):
        marker = Marker()
        marker.text = "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.3,0.4,0.5>, <kds= -1.0000>"
        f_kds = 0.78

        updated_marker = marker_encode_f_kds(marker, f_kds)

        expected_text = "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.3,0.4,0.5>, <kds= 0.7800>"
        self.assertEqual(updated_marker.text, expected_text)

    def test_replace_kds2(self):
        marker = Marker()
        marker.text = "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.3,0.4,0.5>, <kds= E123.4000>"
        f_kds = 0.88

        updated_marker = marker_encode_f_kds(marker, f_kds)
        # print(updated_marker.text)

        expected_text = "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.3,0.4,0.5>, <kds= 0.8800>"
        self.assertEqual(updated_marker.text, expected_text)

    def test_no_kds(self):
        marker = Marker()
        marker.text = "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.3,0.4,0.5>"
        f_kds = 0.78

        with self.assertRaises(AssertionError) as context:
            marker_encode_f_kds(marker, f_kds)
        self.assertIn("no KDS in the text!", str(context.exception))

if __name__ == '__main__':
    unittest.main()