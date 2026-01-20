from typing import Generator

import io
import os
import pathlib
import cv2
import numpy as np
import folium
import math
import argparse

from matplotlib import pyplot as plt
from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    earth_radius: int = 6378137 # meters
    dron_camera_view_angle: int = 80  # dji mavic example
    dron_altitude: int = 55  # meters

    video_read_frame: int = 5
    video_resize_width: int = 960

    # GSD
    def meters_per_pixel(self, video_width: int) -> float:
        radians_koef = math.tan(math.radians(self.dron_camera_view_angle / 2))
        return (
            (2 * self.dron_altitude * radians_koef) / video_width
        )


def save_path_plot(drone_path: list[tuple[float, float]], filename: str = "drone_path_plot.png"):
    x_coords = [p[0] for p in drone_path]
    y_coords = [p[1] for p in drone_path]

    plt.figure(figsize=(10, 10))
    plt.plot(x_coords, y_coords, color='blue', linewidth=2, label='Drone Path')
    plt.scatter(x_coords[0], y_coords[0], color='green', label='Start')  # Start Point
    plt.scatter(x_coords[-1], y_coords[-1], color='red', label='End')  # End Point

    plt.title("Relative Flight Trajectory (Meters)")
    plt.xlabel("X Displacement (m)")
    plt.ylabel("Y Displacement (m)")
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    plt.axis('equal')

    with io.BytesIO() as buffer:
        plt.savefig(buffer, format='png', dpi=300)
        buffer.seek(0)

        with open(filename, 'wb') as f:
            f.write(buffer.getvalue())

    plt.close()


class DronePathBuilder:
    def __init__(
        self,
        settings: Settings,
        video_file_path: str,
        result_dir: str,
        current_lat: float = 0.0,
        current_lon: float = 0.0
    ):
        self.settings = settings
        self.video_file_path = video_file_path
        self.result_dir_path = result_dir
        self.current_lat = current_lat
        self.current_lon = current_lon

        # optimization
        self.target_width = self.settings.video_resize_width
        self.video_width = self.settings.video_resize_width

        # coordinates
        self.earth_radius = self.settings.earth_radius

        # map setup
        self.zoom_start = 20
        self.folium_settings = {
            "color": "black",
            "weight": 2.5,
            "opacity": 1,
        }

        self._track_features_config = dict(
            mask=None,
            maxCorners=100,
            qualityLevel=0.3,
            minDistance=7,
        )

        self._drone_flight_plan_origin = []

    @staticmethod
    def _get_cvt_color(
        frame: np.ndarray,
        dimension: tuple,
        color: int = cv2.COLOR_BGR2GRAY,
    ) -> np.ndarray:
        return cv2.cvtColor(src=cv2.resize(frame, dimension), code=color)

    def _track_features(self, frame: np.ndarray) -> np.ndarray:
        return cv2.goodFeaturesToTrack(
            frame,
            **self._track_features_config,
        )

    def calculate_flight_plan(self):
        video_capture = cv2.VideoCapture(video_path)
        if not video_capture.isOpened():
            raise RuntimeError(f"Could not open video file {video_path}")

        # video params
        self.video_width = int(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        video_height = int(video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        video_total_frames = int(video_capture.get(cv2.CAP_PROP_FRAME_COUNT))

        ret, old_frame = video_capture.read()
        if not ret:
            raise RuntimeError(f"Could not read video file {video_path}")

        scale_factor = self.target_width / self.video_width
        dimension = (self.target_width, int(video_height * scale_factor))
        old_gray_color = self._get_cvt_color(old_frame, dimension)
        first_frame = self._track_features(old_gray_color)

        frame_idx = 0
        print(f"Total frames: {video_total_frames}, start processing...")
        while True:
            # large files optimization: read not all frames
            for _ in range(self.settings.video_read_frame - 1):
                video_capture.grab()
                frame_idx += 1

            ret, frame = video_capture.read()
            if not ret:
                break

            frame_idx += 1
            frame_gray = self._get_cvt_color(frame, dimension)
            next_frame, st, err = cv2.calcOpticalFlowPyrLK(
                old_gray_color, frame_gray, first_frame, None,
            )

            if next_frame is not None and len(next_frame[st == 1]) > 0:
                # Calculate average pixel movement
                diff = (next_frame[st == 1] - first_frame[st == 1]) * (1 / scale_factor)
                if len(diff) > 0:
                    avg_dx, avg_dy = np.mean(diff, axis=0)
                    self._drone_flight_plan_origin.append((avg_dx, avg_dy))

                # Reset tracking points periodically to prevent "drift"
                old_gray = frame_gray.copy()
                first_frame = self._track_features(old_gray)

            if frame_idx % 1000 == 0:
                print(f"Progress: {100 * frame_idx / video_total_frames:.1f}%")

        video_capture.release()

    def get_dfp_in_meters(self) -> Generator[tuple[float, float], None, None]:
        mpp = self.settings.meters_per_pixel(self.video_width)
        scale_factor = self.video_width / self.target_width
        frame_scale = self.settings.video_read_frame

        if not self._drone_flight_plan_origin:
            self.calculate_flight_plan()

        for avg_dx, avg_dy in self._drone_flight_plan_origin:
            dx_m = avg_dx * mpp * scale_factor * frame_scale
            dy_m = -avg_dy * mpp * scale_factor * frame_scale
            yield dx_m, dy_m

    def get_dfp2coordinates(self) -> list[tuple[float, float]]:
        curr_lat = self.current_lat
        curr_lon = self.current_lon
        open_angle = self.settings.dron_camera_view_angle
        gps_coordinates = [(curr_lat, curr_lon)]

        distance_x_init, distance_y_init = 0, 0
        for dx_m, dy_m in self.get_dfp_in_meters():
            # calculate diff between coords
            _dx_m = dx_m - distance_x_init
            _dy_m = dy_m - distance_y_init

            # convert to GPS
            curr_lat += (_dy_m / self.earth_radius) * (open_angle / math.pi)
            curr_lon += (_dx_m / (self.earth_radius * math.cos(math.pi * curr_lat / open_angle))) * (open_angle / math.pi)

            # update state
            gps_coordinates.append((curr_lat, curr_lon))
            distance_x_init = dx_m
            distance_y_init = dy_m

        return gps_coordinates

    def is_valid_current_coordinates(self) -> bool:
        return self.current_lon is not None and self.current_lat is not None

    def get_folium_map(self) -> folium.Map:
        start_gps = self.current_lat, self.current_lon
        gps_coordinates = self.get_dfp2coordinates()

        folium_map = folium.Map(location=start_gps)
        folium.PolyLine(gps_coordinates, **self.folium_settings).add_to(folium_map)

        return folium_map

    def build_drone_map(self):
        drone_path = list(self.get_dfp_in_meters())

        output_dir = pathlib.Path(self.result_dir_path)
        output_dir.mkdir(parents=True, exist_ok=True)

        image_path = os.path.join(self.result_dir_path, "drone_map.png")
        save_path_plot(drone_path, image_path)

        if self.is_valid_current_coordinates():
            folium_map = self.get_folium_map()

            with io.BytesIO() as map_buffer:
                folium_map.save(map_buffer, close_file=False)
                map_buffer.seek(0)

                with open(os.path.join(self.result_dir_path, "drone_map.html"), "wb") as fl:
                    fl.write(map_buffer.getvalue())


def process_video(
    video_file_path: str,
    result_dir: str,
    start_lat: float | None = None,
    start_lon: float | None = None,
):
    path_builder = DronePathBuilder(
        settings=Settings(),
        video_file_path=video_file_path,
        result_dir=result_dir,
        current_lat=start_lat,
        current_lon=start_lon,
    )
    path_builder.build_drone_map()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("video_file_path", help="Path to the video file.")
    parser.add_argument("-rd", required=False, help="Result dir", default="result-drone-path")
    parser.add_argument("-lt", required=False, help="Start latitude", default=None, type=float)
    parser.add_argument("-ln", required=False, help="Start longitude", default=None, type=float)

    args = parser.parse_args()

    video_path = args.video_file_path
    result_directory = args.rd
    lat = args.lt or None
    lon = args.ln or None

    if (
        (lat is None and lon is None)
        or (lat and lon)
    ):
        process_video(video_path, result_directory, lat, lon)
    else:
        print("Provide BOTH or NEITHER: lat, lon")
        exit(1)
