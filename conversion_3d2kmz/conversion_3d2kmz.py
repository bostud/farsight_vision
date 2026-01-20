import argparse
import pyproj
import trimesh
import zipfile
import io
import os


def create_kml_file_content(lat: float, lon: float, model_name: str) -> str:
    return f"""
        <?xml version="1.0" encoding="UTF-8"?>
        <kml xmlns="http://www.opengis.net/kml/2.2">
        <Folder>
            <name>ATAK Georeferenced Model</name>
            <Placemark>
                <name>{model_name}</name>
                <Model>
                    <altitudeMode>clampToGround</altitudeMode>
                    <Location>
                        <longitude>{lon}</longitude>
                        <latitude>{lat}</latitude>
                        <altitude>0</altitude>
                    </Location>
                    <Orientation>
                        <heading>0</heading>
                        <tilt>0</tilt>
                        <roll>0</roll>
                    </Orientation>
                    <Scale>
                        <x>1</x>
                        <y>1</y>
                        <z>1</z>
                    </Scale>
                    <Link>
                        <href>models/model.dae</href>
                    </Link>
                </Model>
            </Placemark>
        </Folder>
    </kml>
    """.strip().strip("\n")


def write_kmz_file(
    mesh_model: trimesh.Geometry,
    kmz_save_path: str,
    kml_file_content: str,
) -> None:
    with io.BytesIO() as dae_buffer:
        mesh_model.export(dae_buffer, file_type="dae")
        dae_buffer.seek(0)

        with zipfile.ZipFile(kmz_save_path, "w", compression=zipfile.ZIP_DEFLATED) as kmz_file:
            kmz_file.writestr("doc.kml", kml_file_content)
            kmz_file.writestr("models/model.dae", dae_buffer.read())


class Wgs84CoordParser:
    def __init__(self, file_path: str, line_splitter: str = " "):
        self.transformer = pyproj.Transformer.from_crs(
            "EPSG:32635", "EPSG:4326",
            always_xy=True,
        )
        self.geo_coords_format = "WGS84 UTM 35N"
        self.line_splitter = line_splitter
        self.file_path = file_path
        self._x_coords = []
        self._y_coords = []

    def calculate_lon_lat(self, force_read: bool = False) -> tuple[float, float]:
        if force_read or not (self._x_coords and self._y_coords):
            self._x_coords, self._y_coords = self._read_coords()
        lons, lats = self.transformer.transform(self._x_coords, self._y_coords)
        # return only first pair of coordinates
        return lons[0], lats[0]

    def parse_line(self, line: str) -> tuple[float, float]:
        line_split = line.strip().split(self.line_splitter)
        if len(line_split) == 2:
            return tuple(float(v) for v in line_split)  # noqa

        raise ValueError(f"Invalid line: {line}")

    def _read_coords(self) -> tuple[list, list]:
        x_coords = []
        y_coords = []

        with open(self.file_path, 'r') as fl:
            for line in fl:
                file_line = line.strip()
                # skip empty lines
                if not file_line:
                    continue

                # skip format line
                if self.geo_coords_format and file_line.find(self.geo_coords_format) >= 0:
                    continue

                # prepare coordinates
                try:
                    east, north = self.parse_line(file_line)
                except ValueError:
                    continue

                x_coords.append(float(east))
                y_coords.append(float(north))

        return x_coords, y_coords


class Model3dOBJLoader:
    def __init__(self, model_3d_obj_path: str):
        self.model_3d_obj_path = model_3d_obj_path

    def load_model_obj(
        self,
        optimize: bool = False,
        faces_count: int = 50000,
        zero_position: bool = False,
    ) -> trimesh.Geometry:
        mesh = trimesh.load(self.model_3d_obj_path)

        if isinstance(mesh, trimesh.Scene):
            mesh = mesh.to_geometry()

        if zero_position:
            bounds = mesh.bounds
            center_x = (bounds[0][0] + bounds[1][0]) / 2
            center_y = (bounds[0][1] + bounds[1][1]) / 2
            bottom_z = bounds[0][2]

            mesh.apply_translation([-center_x, -center_y, -bottom_z])

        if optimize:
            if len(mesh.faces) > faces_count:
                reduction_ratio = 1.0 - (faces_count / len(mesh.faces))
                # import pdb; pdb.set_trace()
                reduction_ratio = max(0.0, min(0.99, reduction_ratio))

                mesh = mesh.simplify_quadric_decimation(reduction_ratio)

        return mesh


def convert(
    model_obj_file_path: str,
    georeferencing_file_path: str,
    output_kmz_path: str,
) -> None:
    """
    Scrip implementation of 3d model conversion to .kmz file

    can be written as class / but selected as scope of functions without state

    :Params:
        model_obj_file_path: str - 3d model .obj file path,
        georeferencing_file_path: str - georeferencing file path,
        output_kmz_path: str - output .kmz file path,

    :Steps:
        read input files
        calculate coordinates to lat and lon
        convert 3d obj to dae file
        save kmz: with doc.kml and .dae files

    :return: None
    """
    coords_parser = Wgs84CoordParser(georeferencing_file_path)
    model_loader = Model3dOBJLoader(model_obj_file_path)

    lon, lat = coords_parser.calculate_lon_lat()
    mesh_model = model_loader.load_model_obj(optimize=False, zero_position=True)
    # write to kmz
    kml_content = create_kml_file_content(lat, lon, os.path.basename(model_obj_file_path))
    write_kmz_file(mesh_model, output_kmz_path, kml_content)


if __name__ == "__main__":
    # init argument parser
    parser = argparse.ArgumentParser(description='Converts 3D models to KMZ')

    parser.add_argument("model_obj_file_path", help="Path to the 3D model file", type=str)
    parser.add_argument("georeferencing_file_path", help="Path to the georeferencing", type=str)
    parser.add_argument("output_kmz_path", help="Path where should be saved kmz file", type=str)

    args = parser.parse_args()

    print("Start: converting 3D model to kmz file")
    convert(
        args.model_obj_file_path,
        args.georeferencing_file_path,
        args.output_kmz_path,
    )
    print("Finish: converting 3D model to kmz file")
