# Build drone path prediction

## How to use:
1. Install python version - `3.12.4`
2. Install required packages - `pip install requirements.txt`
3. Run script `drone_path_prediction.py`
    ```
    Input params:
       > video_file_path - path to video file (tested on MP4 format)
       > -rd - result dir (optional)
       > -lt - video init latitude (optional)
       > -ln - video init longitude (optional)
   
    Pay attention:
      > lt&ln - provide both or neither
    ```
4. Usage example: 
    ```
    python drone_path_prediction.py /absolute/path/to/my/video.mp4 [-rd /save/here/dir/] [-lt 1.0 -ln 1.0]
    ```

## Short description
Script will create result dir and 2 files:
   - drone_path.png - plot image in meters scale
   - drone_path.html - if lt&ln provided -> folium.Map will be saved


## Optimization places:
1. Drone path "noise" - improve path visibility on map, deleting places where drone stays in place, watching down or other non-movements actions.
