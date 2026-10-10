"""Integration checks using actual ROS MCAP writers (no ROS installation required)."""

import csv
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

from mcap.writer import CompressionType, Writer as McapWriter
from mcap_ros2.writer import Writer as Ros2Writer


SCRIPT = Path(__file__).with_name("decompose_mcap.py")
ROS2_SCHEMA = """string label
example_msgs/Point position
float64[] samples
example_msgs/Point[] points
uint8[] payload
================================================================================
MSG: example_msgs/Point
float64 x
float64 y
"""


class ExportTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.bag = self.root / "recording.mcap"
        self.output = self.root / "combined.csv"

    def run_export(self, *args):
        return subprocess.run(
            [sys.executable, str(SCRIPT), str(self.bag), "-o", str(self.output), *args],
            capture_output=True,
            text=True,
        )

    def read_exports(self):
        self.assertTrue(self.output.is_file(), "Expected one CSV file, not a directory")
        self.assertEqual(list(self.root.glob("*.csv")), [self.output])
        rows = {}
        with self.output.open(newline="", encoding="utf-8") as stream:
            messages = list(csv.DictReader(stream))
        self.assertTrue(messages)
        for message in messages:
            rows.setdefault(message["topic"], []).append(message)
        return rows

    def write_ros2(self, compression=CompressionType.ZSTD):
        with self.bag.open("wb") as stream:
            writer = Ros2Writer(stream, compression=compression)
            schema = writer.register_msgdef("example_msgs/msg/Sample", ROS2_SCHEMA)
            for i in range(2):
                writer.write_message(
                    "/robot/pose",
                    schema,
                    {
                        "label": 'hello, "robot"\n' if i == 0 else "Olá",
                        "position": {"x": 1.25 + i, "y": -2.0},
                        "samples": [] if i == 0 else [3.5, 4.5],
                        "points": [] if i == 0 else [{"x": 5.0, "y": 6.0}],
                        "payload": bytes([0, 128, 255]),
                    },
                    log_time=1_700_000_000_000_000_003 - i,
                    publish_time=1_700_000_000_000_000_000 + i,
                    sequence=i,
                )
            # A later topic introduces a field that must appear in the shared header.
            text_schema = writer.register_msgdef("std_msgs/msg/String", "string data")
            writer.write_message("/a/b", text_schema, {"data": "first"}, log_time=5)
            writer.write_message("/a_b", text_schema, {"data": "second"}, log_time=6)
            writer.finish()

    def test_ros2_nested_arrays_timestamps_and_distinct_topics(self):
        self.write_ros2()
        result = self.run_export()
        self.assertEqual(result.returncode, 0, result.stderr)
        exports = self.read_exports()
        self.assertEqual(set(exports), {"/robot/pose", "/a/b", "/a_b"})
        first, second = exports["/robot/pose"]
        self.assertEqual(first["message_type"], "example_msgs/msg/Sample")
        self.assertEqual(first["log_time_ns"], "1700000000000000003")
        self.assertEqual(first["publish_time_ns"], "1700000000000000000")
        self.assertEqual(first["sequence"], "0")
        self.assertEqual(first["msg.label"], 'hello, "robot"\n')
        self.assertEqual(second["msg.label"], "Olá")
        self.assertEqual(first["msg.position.x"], "1.25")
        self.assertEqual(second["msg.position.x"], "2.25")
        self.assertEqual(json.loads(first["msg.samples"]), [])
        self.assertEqual(json.loads(second["msg.samples"]), [3.5, 4.5])
        self.assertEqual(json.loads(second["msg.points"]), [{"x": 5.0, "y": 6.0}])
        self.assertEqual(json.loads(first["msg.payload"]), [0, 128, 255])
        self.assertEqual(first["msg.data"], "")
        self.assertEqual(exports["/a/b"][0]["msg.data"], "first")
        self.assertEqual(exports["/a/b"][0]["msg.position.x"], "")
        with self.output.open(newline="", encoding="utf-8") as stream:
            self.assertEqual(
                [row["topic"] for row in csv.DictReader(stream)],
                ["/robot/pose", "/robot/pose", "/a/b", "/a_b"],
            )

    def test_topic_filter_and_lz4_compression(self):
        self.write_ros2(CompressionType.LZ4)
        result = self.run_export("--topic", "/a/b", "--topic", "/a_b")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(set(self.read_exports()), {"/a/b", "/a_b"})

    def test_ros1_without_indexes(self):
        # std_msgs/String uses a little-endian byte count followed by UTF-8 bytes.
        with self.bag.open("wb") as stream:
            writer = McapWriter(stream, use_chunking=False, use_statistics=False,
                                repeat_channels=False, repeat_schemas=False)
            writer.start(profile="ros1")
            schema = writer.register_schema("std_msgs/String", "ros1msg", b"string data\n")
            channel = writer.register_channel("/chatter", "ros1", schema)
            text = 'ROS 1, "CSV"'.encode()
            writer.add_message(channel, 123, struct.pack("<I", len(text)) + text, 120, 7)
            writer.finish()
        result = self.run_export()
        self.assertEqual(result.returncode, 0, result.stderr)
        row = self.read_exports()["/chatter"][0]
        self.assertEqual(row["msg.data"], 'ROS 1, "CSV"')
        self.assertEqual(row["sequence"], "7")

    def test_existing_csv_is_preserved_unless_overwrite_is_requested(self):
        self.write_ros2()
        result = self.run_export()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(self.output.is_file(), "Expected one combined CSV")
        before = self.output.read_bytes()
        result = self.run_export()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("--overwrite", result.stderr)
        self.assertEqual(before, self.output.read_bytes())
        result = self.run_export("--overwrite")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(before, self.output.read_bytes())

    def test_default_output_is_csv_beside_input(self):
        self.write_ros2()
        result = subprocess.run(
            [sys.executable, str(SCRIPT), str(self.bag)], capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(self.bag.with_suffix(".csv").is_file())

    def test_output_cannot_replace_input_bag(self):
        self.write_ros2()
        before = self.bag.read_bytes()
        self.output = self.bag
        result = self.run_export("--overwrite")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("different", result.stderr.lower())
        self.assertEqual(before, self.bag.read_bytes())

    def test_missing_file_and_no_matching_topic_report_errors(self):
        result = self.run_export()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("error:", result.stderr.lower())
        self.write_ros2()
        result = self.run_export("--topic", "/absent")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("no messages", result.stderr.lower())

    def test_unsupported_schema_reports_topic_and_encoding(self):
        with self.bag.open("wb") as stream:
            writer = McapWriter(stream)
            writer.start()
            schema = writer.register_schema("example_msgs/msg/IDL", "ros2idl", b"module example {};")
            channel = writer.register_channel("/idl", "cdr", schema)
            writer.add_message(channel, 1, b"unsupported", 1)
            writer.finish()
        result = self.run_export()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("/idl", result.stderr)
        self.assertIn("ros2idl", result.stderr)
        self.assertIn("unsupported", result.stderr.lower())


if __name__ == "__main__":
    unittest.main()
