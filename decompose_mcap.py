#!/usr/bin/env python3
"""Export all selected ROS 1/2 MCAP topics into one CSV file.

Setup: python3 -m venv .venv
       .venv/bin/python -m pip install -r requirements.txt
Usage: .venv/bin/python decompose_mcap.py recording.mcap [-o output.csv] [--topic /imu]
"""

import argparse
from array import array
from collections.abc import Mapping
import csv
import json
from pathlib import Path
from shlex import quote
from typing import Any


def to_plain(value: Any) -> Any:
    """Convert dynamically decoded ROS messages into JSON-compatible values."""
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, (bytes, bytearray, memoryview)):
        return list(value)
    if isinstance(value, Mapping):
        return {key: to_plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, array)):
        return [to_plain(item) for item in value]
    # ROS 2 dynamic messages and ROS 1 genpy objects store fields in slots.
    if hasattr(value, "__slots__"):
        return {name: to_plain(getattr(value, name)) for name in value.__slots__}
    if hasattr(value, "__dict__"):
        return {name: to_plain(item) for name, item in vars(value).items()}
    raise TypeError(f"Cannot export a value of type {type(value).__name__}")


def flatten_fields(value: Any, prefix: str = "msg") -> dict[str, Any]:
    """Flatten nested objects; keep variable-length arrays in a single JSON cell."""
    if isinstance(value, dict) and value:
        fields = {}
        for name, item in value.items():
            fields.update(flatten_fields(item, f"{prefix}.{name}"))
        return fields
    if isinstance(value, (list, dict)):
        value = json.dumps(value, ensure_ascii=False, separators=(",", ":"))
    return {prefix: value}


def decompose_mcap(
    input_file: Path,
    output_file: Path,
    topics: list[str] | None = None,
    overwrite: bool = False,
) -> int:
    """Discover columns, then stream every message to one CSV in file order."""
    try:
        from mcap.reader import make_reader
        from mcap_ros1.decoder import DecoderFactory as Ros1DecoderFactory
        from mcap_ros2.decoder import DecoderFactory as Ros2DecoderFactory
    except ImportError as exc:
        requirements = Path(__file__).with_name("requirements.txt")
        raise RuntimeError(
            "Missing MCAP dependencies. Create and use a virtual environment:\n"
            "  python3 -m venv .venv\n"
            f"  .venv/bin/python -m pip install -r {quote(str(requirements))}\n"
            f"  .venv/bin/python {quote(str(Path(__file__)))} <bag.mcap>"
        ) from exc

    factories = [Ros1DecoderFactory(), Ros2DecoderFactory()]
    decoders = {}

    def build_row(schema, channel, message):
        if channel.id not in decoders:
            decoder = None
            for factory in factories:
                decoder = factory.decoder_for(channel.message_encoding, schema)
                if decoder is not None:
                    break
            if decoder is None:
                encoding = schema.encoding if schema is not None else "missing"
                raise ValueError(
                    f"Unsupported topic {channel.topic!r}: message encoding "
                    f"{channel.message_encoding!r}, schema encoding {encoding!r}. "
                    "Expected ROS 1 ros1/ros1msg or ROS 2 cdr/ros2msg."
                )
            decoders[channel.id] = decoder
        try:
            decoded = decoders[channel.id](message.data)
            fields = flatten_fields(to_plain(decoded))
        except Exception as exc:
            raise ValueError(
                f"Could not decode topic {channel.topic!r} at "
                f"log_time_ns={message.log_time}: {exc}"
            ) from exc
        return {
            "topic": channel.topic,
            "message_type": schema.name if schema is not None else "",
            "log_time_ns": message.log_time,
            "publish_time_ns": message.publish_time,
            "sequence": message.sequence,
            **fields,
        }

    with input_file.open("rb") as stream:
        if output_file.exists():
            if input_file.samefile(output_file):
                raise ValueError("The input bag and output CSV must be different files.")
            if not overwrite:
                raise FileExistsError(
                    f"{output_file} already exists; use --overwrite to replace it."
                )

        # ROS fields are fixed by each channel's schema. Decoding its first
        # message discovers the fields; JSON array cells keep columns stable.
        columns = {}
        reader = make_reader(stream)
        for schema, channel, message in reader.iter_messages(
            topics=topics, log_time_order=False
        ):
            if channel.id not in decoders:
                columns.update(dict.fromkeys(build_row(schema, channel, message)))
        if not columns:
            raise ValueError("No messages found in the MCAP file for the selected topics.")

        # Read again to write a complete header without retaining all the rows.
        stream.seek(0)
        reader = make_reader(stream)
        output_file.parent.mkdir(parents=True, exist_ok=True)
        with output_file.open(
            "w" if overwrite else "x", newline="", encoding="utf-8"
        ) as output:
            writer = csv.DictWriter(output, fieldnames=list(columns))
            writer.writeheader()
            count = 0
            for schema, channel, message in reader.iter_messages(
                topics=topics, log_time_order=False
            ):
                writer.writerow(build_row(schema, channel, message))
                count += 1
    return count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_file", type=Path, help="Input .mcap ROS bag file")
    parser.add_argument(
        "-o", "--output", "--output-file", dest="output_file", type=Path,
        help="Output CSV file (default: <input_stem>.csv beside the bag)",
    )
    parser.add_argument(
        "--topic", action="append", dest="topics", metavar="TOPIC",
        help="Export only this exact topic; repeat to include more topics",
    )
    parser.add_argument(
        "--overwrite", action="store_true", help="Replace an existing output CSV file",
    )
    args = parser.parse_args()
    output_file = args.output_file or args.input_file.with_suffix(".csv")
    try:
        count = decompose_mcap(args.input_file, output_file, args.topics, args.overwrite)
    except Exception as exc:
        parser.exit(1, f"error: {exc}\n")
    except KeyboardInterrupt:
        parser.exit(130, "Export interrupted; the output CSV may be incomplete.\n")
    print(f"Exported {count} messages to {output_file}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
