"""Record the Gazebo chase camera to an MP4 file using system ffmpeg."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


PIXEL_FORMATS: Dict[str, Tuple[str, int]] = {
    "rgb8": ("rgb24", 3),
    "bgr8": ("bgr24", 3),
    "rgba8": ("rgba", 4),
    "bgra8": ("bgra", 4),
    "mono8": ("gray", 1),
}


class VideoRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("video_recorder_node")
        self.declare_parameter("video_path", "results/gazebo.mp4")
        self.declare_parameter("camera_topic", "/simulation/recording_camera/image")
        self.declare_parameter("fps", 30.0)
        self.video_path = Path(str(self.get_parameter("video_path").value)).expanduser().resolve()
        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.fps = float(self.get_parameter("fps").value)
        self.process: Optional[subprocess.Popen] = None
        self.frame_spec: Optional[Tuple[int, int, str, int]] = None
        self.frame_count = 0
        self.failed = False
        self.create_subscription(
            Image,
            self.camera_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Waiting for Gazebo camera {self.camera_topic}; MP4 output: {self.video_path}"
        )

    def _start_ffmpeg(self, message: Image) -> bool:
        encoding = message.encoding.lower()
        if encoding not in PIXEL_FORMATS:
            self.get_logger().error(
                f"Unsupported camera encoding {message.encoding!r}; "
                f"supported encodings: {', '.join(sorted(PIXEL_FORMATS))}"
            )
            self.failed = True
            return False
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            self.get_logger().error("ffmpeg is not installed; run: sudo apt-get install -y ffmpeg")
            self.failed = True
            return False
        pixel_format, bytes_per_pixel = PIXEL_FORMATS[encoding]
        self.video_path.parent.mkdir(parents=True, exist_ok=True)
        command = [
            ffmpeg,
            "-y",
            "-nostdin",
            "-loglevel",
            "warning",
            "-f",
            "rawvideo",
            "-pixel_format",
            pixel_format,
            "-video_size",
            f"{message.width}x{message.height}",
            "-framerate",
            f"{self.fps:.3f}",
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "veryfast",
            "-crf",
            "22",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(self.video_path),
        ]
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.frame_spec = (message.width, message.height, encoding, bytes_per_pixel)
        self.get_logger().info(
            f"Recording {message.width}x{message.height} {encoding} at {self.fps:.1f} fps"
        )
        return True

    def _packed_frame(self, message: Image, bytes_per_pixel: int) -> bytes:
        packed_step = message.width * bytes_per_pixel
        data = bytes(message.data)
        if message.step == packed_step:
            return data
        rows = []
        for row in range(message.height):
            start = row * message.step
            rows.append(data[start:start + packed_step])
        return b"".join(rows)

    def _image_callback(self, message: Image) -> None:
        if self.failed:
            return
        if self.process is None and not self._start_ffmpeg(message):
            return
        width, height, encoding, bytes_per_pixel = self.frame_spec
        if (
            message.width != width
            or message.height != height
            or message.encoding.lower() != encoding
        ):
            self.get_logger().error("Gazebo camera format changed while recording; stopping video")
            self.failed = True
            self._close_video()
            return
        if self.process is None or self.process.stdin is None:
            return
        try:
            self.process.stdin.write(self._packed_frame(message, bytes_per_pixel))
            self.frame_count += 1
        except (BrokenPipeError, OSError) as error:
            self.get_logger().error(f"ffmpeg stopped while recording: {error}")
            self.failed = True
            self._close_video()

    def _close_video(self) -> None:
        if self.process is None:
            return
        process = self.process
        self.process = None
        if process.stdin is not None and not process.stdin.closed:
            try:
                process.stdin.close()
            except OSError:
                pass
        try:
            return_code = process.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                return_code = process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                return_code = process.wait()
        if return_code == 0 and self.frame_count > 0:
            self.get_logger().info(
                f"Saved Gazebo video with {self.frame_count} frames: {self.video_path}"
            )
        else:
            self.get_logger().error(
                f"Video encoder exited with code {return_code}; frames written: {self.frame_count}"
            )

    def destroy_node(self):
        self._close_video()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VideoRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
