# husmart-sim — Hüsmart autonomous vehicle, Beytepe campus simulation

Autonomous driving with **Autoware** on Hacettepe University's Beytepe campus, simulated in **AWSIM**.
Vehicle: Hüsmart. Sensors: Hesai OT128 on the roof, 8 × Hesai JT128 lidars around the body, 2 × ZED X stereo cameras, Xsens MTi-7.

Two repositories are used together:

| Repository | Contents |
|---|---|
| **husmart-sim** (this repo) | Everything added to Autoware from the outside: sensor kit, parameters, overtake manager, map, tests, launch scripts |
| **[husmart-awsim](https://github.com/Mehmetmrtgl/husmart-awsim)** (branch `beytepe`) | Fork of AWSIM: Beytepe scene, buildings, speed bumps, pedestrians, sensor simulation |

**Autoware's source code is not modified.** The official Autoware Docker image is run as is. The parameter files in this repo
and the `husmart_sensor_kit` package are mounted read-only into the container, so Autoware does not need to be built.

---

## 1. Requirements

- Ubuntu 22.04, an NVIDIA GPU and driver (tested on an RTX 5090).
- Docker and the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).
- Disk: Autoware image ~20 GB, AWSIM player ~0.6 GB, map ~0.1 GB.
- Only if you build AWSIM yourself: Unity Hub, Unity **6000.0.61f1**, a Unity Personal license.

## 2. Setup (once)

### 2.1 Clone the repo

    git clone https://github.com/Mehmetmrtgl/husmart-sim.git ~/husmart-sim
    cd ~/husmart-sim

All commands below are run from this folder. The scripts resolve paths relative to their own location, so the repo can live anywhere.

### 2.2 Autoware image

    docker pull ghcr.io/autowarefoundation/autoware@sha256:b7490c70ba67c85a20d94feba68137ba911b5032c621353d50f0c63d3ee63977

This is the tested version (`universe-cuda`, 2026-04-13, autoware_launch 0.50.0). The `universe-cuda` tag moves over time
and a newer image may not match these parameter files, so the scripts pin this digest.

### 2.3 Autoware AI models (`~/autoware_data`)

The perception models (`lidar_centerpoint`, `tensorrt_yolox`, `traffic_light_classifier`, `traffic_light_fine_detector`) are not in the image.
Download them into `~/autoware_data` with the "artifacts" step of the Autoware documentation:
[Autoware source installation → artifacts](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/).
Copying `~/autoware_data` from another machine in the lab also works.

### 2.4 Map

The Lanelet2 map is in the repo: `harita/beytepe_yeni_v2/lanelet2_map.osm`. The point cloud is too large for git and is on the
**[Releases](https://github.com/Mehmetmrtgl/husmart-sim/releases)** page:

    # download beytepe_yeni_v2_karolar.tar.gz from Releases, then:
    tar -xzf beytepe_yeni_v2_karolar.tar.gz -C harita/
    # (optional) single-file point cloud: pointcloud_map.pcd -> harita/beytepe_yeni_v2/

### 2.5 AWSIM player

**A) Prebuilt player (easy):** download `AWSIM-Beytepe-linux.tar.gz` from
[husmart-awsim Releases](https://github.com/Mehmetmrtgl/husmart-awsim/releases):

    mkdir -p ~/awsim && tar -xzf AWSIM-Beytepe-linux.tar.gz -C ~/awsim/     # -> ~/awsim/Beytepe-Build/AWSIM-Beytepe.x86_64

**B) Build it yourself** (if you changed the Beytepe scene):

    git clone -b beytepe https://github.com/Mehmetmrtgl/husmart-awsim.git ~/awsim/AWSIM-src
    ./build_beytepe_awsim.sh          # headless, 15-30 min; AWSIM must not be running

You must be signed in to Unity Hub with an active Personal license. Log: `~/awsim/beytepe_unity_build.log`, summary:
`~/awsim/beytepe_build_report.txt`. Other paths: `UNITY=... AWSIM_SRC=... BEYTEPE_BUILD=... ./build_beytepe_awsim.sh`.

### 2.6 Network settings (after every reboot, sudo)

AWSIM and Autoware talk over ROS 2 (CycloneDDS) on the same machine:

    sudo sysctl -w net.core.rmem_max=2147483647
    sudo ip link set lo multicast on

## 3. Running

    ./run_awsim.sh                 # starts AWSIM (ready after ~1 min). Stop: pkill -f AWSIM-Beytepe.x86_64
    ./run_autoware_beytepe.sh      # Autoware + RViz. Stop: docker rm -f autoware_e2e
    SOLLAMA=1 ./run_autoware_beytepe.sh   # also starts the overtake manager

`run_autoware_beytepe.sh` waits until localization succeeds (up to 4 min). Then set a goal in RViz with **2D Goal Pose**
and press **AUTO**.

Rules:
- Restart Autoware every time AWSIM is restarted.
- Stop AWSIM before building it.
- The default is `ROS_DOMAIN_ID=61`. If you change it, give both scripts the same value.

### Teleporting the vehicle and reading its position

With AWSIM running (Autoware is not needed):

    ./isinla.sh                      # list of known places
    ./isinla.sh nizamiye             # teleport to a known place (baslangic, nizamiye, yurt)
    ./isinla.sh -150.2 2640.5 270    # teleport to x y [heading°] (map frame; 0 = east, 90 = north)
    ./konum.sh                       # print the current x, y, heading and the matching ./isinla.sh command
    ./konum.sh "missing road here"   # also append the position and the note to notlar/konumlar.txt

The height is taken from the nearest lane in the map. To add a named place, add a line to the `YERLER` list at the top of
`isinla.sh`. If Autoware is running, localization may lose track after a teleport; re-initialize with **2D Pose Estimate** in RViz.

## 4. Test tour

The campus tour (start → main gate → back, with NPC traffic and pedestrians) is driven by `beytepe_tests_v2/analysis/run_tests.py`.
With AWSIM and Autoware running:

    IMG=ghcr.io/autowarefoundation/autoware@sha256:b7490c70ba67c85a20d94feba68137ba911b5032c621353d50f0c63d3ee63977
    COMMON="--network host --ipc host -u $(id -u):$(id -g) -e HOME=/tmp -e ROS_LOG_DIR=/tmp/roslog -e ROS_DOMAIN_ID=61 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///autoware_cfg/cyclonedds.xml -v $PWD/cyclonedds.xml:/autoware_cfg/cyclonedds.xml:ro"
    docker run -d --name autoware_tur $COMMON -v $PWD/harita/beytepe_yeni_v2:/autoware_map:ro -v $PWD/beytepe_tests_v2:/tests $IMG \
      bash -c "source /opt/autoware/setup.bash && python3 /tests/analysis/run_tests.py /tests/analysis/specs_awsim_beytepe_tur.json /tests/results_awsim_beytepe > /tests/results_awsim_beytepe/tur.log 2>&1"

The result is written to `beytepe_tests_v2/results_awsim_beytepe/awsim_beytepe_03_tur.json` (the next tour overwrites it).
To record a tour with `ros2 bag record` and analyse it, see `beytepe_tests_v2/analysis/tur_analiz/README.txt`.
Never run two `ros2 bag record` at once: their node names collide and autonomous mode is blocked.

## 5. Editing the map (main gate road, median island)

The main gate road and the median island are not hand-edited: they are generated from data by one tool,
`tools/beytepe_duzenle/duzenle.py`, which writes the lanelet map, the AWSIM scene data and the point cloud together,
so the three always match. Settings (paths, lane width, curb size, lamp spacing, ...) are in `tools/beytepe_duzenle/ayarlar.yaml`.

    pip install -r tools/beytepe_duzenle/requirements.txt     # numpy, scipy, shapely >= 2, pyyaml, matplotlib (opencv for "kontrol")
    python3 tools/beytepe_duzenle/duzenle.py hepsi            # drawing -> lanelet map -> AWSIM data -> Google Earth KML
    ./build_beytepe_awsim.sh                                  # rebuild the AWSIM scene (writes the scene point cloud)
    python3 tools/beytepe_duzenle/duzenle.py bulut            # put the scene points into the point-cloud map + 50 m tiles

| Step | Input | Output |
|---|---|---|
| `cizim` | Google Earth drawing `tools/beytepe_duzenle/veri/nizamiye_cizim.kml` (vehicle routes = lane centres, asphalt areas, bus stop) | `uretilen/cizim_xy.json` (map frame; the imagery offset of 0.9 m / 1.25 m is corrected) |
| `lanelet` | base map `harita/beytepe_yeni_v2/temel/lanelet2_map.osm` + drawing | `harita/beytepe_yeni_v2/lanelet2_map.osm` (17 gate lanes) |
| `awsim` | lanelet map + drawing + `veri/refuj_nesneler.json`, `veri/refuj_guney_nesneler.json`, `veri/nizamiye_nesneler.json`, `veri/korkuluklar.json`, `veri/yol_kenari_nesneler.json` | `~/awsim/AWSIM-src/Assets/Beytepe/Data/duzenlemeler.json` (read by `BeytepeDuzenleme.cs`) |
| `kml` | lanelet map | `docs/ust_bakis/beytepe_sim.kml` and `uretilen/google_earth_hizali.kml` (sits on the Google Earth imagery) |
| `bulut` | base cloud `temel/pointcloud_map.pcd` + AWSIM scene points | `pointcloud_map.pcd` and `harita/beytepe_yeni_v2_karolar/` |

To change the gate road: edit the drawing in Google Earth (keep the placemark names listed in `ayarlar.yaml`), save it over
`veri/nizamiye_cizim.kml`, then run the commands above. Every overwritten file is first copied to `~/yedek/beytepe_duzenle/<time>/`.
The base map and base cloud (`harita/beytepe_yeni_v2/temel/`) are the state before these edits; the base cloud is in the Releases.

What the scene gets from these files:

- The main road median is one island from the campus junction (y ~300) to the gate U-turn (y ~2668): curbs, grass, the
  asphalt strip along the yellow edge line, two-arm lamps and trees measured in the real lidar map (`veri/refuj_*nesneler.json`).
- `veri/nizamiye_nesneler.json` lists the permanent gate objects measured from the real lidar map and camera frames
  (barriers with the arms up, flagpoles, booths, signs, banners, bins, the stone wall with its sidewalk, wire fence).
  `duzenle.py awsim` refuses objects that touch a lane. Temporary things (parked cars, cones) are deliberately left out.
- `veri/korkuluklar.json` lists the guardrails and the white concrete parapet along the tour route: where each one starts and
  ends was read in the real lidar map and camera frames; the line between follows the simulated road edge 1.3 m outside it.
  Ends marked `gomulu` slope down into the ground as on the real road. In the scene a post never stands on a RoadRunner
  sidewalk or curb (it moves outwards).
- `veri/yol_kenari_nesneler.json` (step `kenar-tespit`) holds the trees within 22 m of the tour route, measured in the real
  lidar map; they replace the RoadRunner trees there.
- Lane markings on the tour roads get the real colours (RoadRunner has them the other way round): lane dividers and the
  centre line of two-way roads white, road edges yellow; the exceptions seen in the camera frames (white outer edge of the
  southbound main road, yellow centre line of the west campus road) are in `ayarlar.yaml` (`cizgiler`). The rules are applied
  per line segment; yellow centre lines through junctions (virtual lanelet borders) become white. On the campus roads
  RoadRunner has no painted edge line but a light concrete gutter strip along the curb: along the tour edges (also the
  virtual outer borders of junction lanelets, where the curb runs past a side road) the gutter becomes asphalt and its
  road-side 0.15 m (`oluk_cizgi_m`) the edge colour, as the yellow line by the curb in reality. Short gutter pieces on curves
  and at crossings take their direction from the strip around them.
- Every object and a 3 m margin around it is part of the point-cloud update region, so the map matches the scene.

Four more steps need the real-vehicle recordings (not in the repo; paths in `ayarlar.yaml`):

    python3 tools/beytepe_duzenle/duzenle.py refuj-tespit                    # median lamps + trees from the real lidar map (north of y 1705)
    python3 tools/beytepe_duzenle/duzenle.py refuj-guney-tespit              # the same for the south part (y 300 .. 1705)
    python3 tools/beytepe_duzenle/duzenle.py kenar-tespit                    # trees beside the tour route (real lidar map)
    python3 tools/beytepe_duzenle/duzenle.py kontrol 2560 2680 --yon kuzey   # draw the map onto real ZED camera frames

`kontrol` writes `tools/beytepe_duzenle/uretilen/kontrol/kontrol_<y0>_<y1>_<yon>.jpg`: lane borders (yellow = gate lanes),
median lamps (magenta) and trees (green) drawn on the real camera images; if they sit on the real road and poles, the map is right.

## 6. Folders

| Folder / file | Purpose |
|---|---|
| `run_awsim.sh`, `run_autoware_beytepe.sh`, `build_beytepe_awsim.sh` | Launch and build |
| `cyclonedds.xml` | ROS 2 network settings (local machine only) |
| `awsim_ayar/beytepe-config-husmart.json` | AWSIM settings: sensor layout (`SensorRig`), NPC count, NPCs yielding to the ego (`NpcEgo`), sky. No rebuild needed; restart AWSIM. |
| `husmart_sensor_kit/` | Autoware sensor kit (lidar preprocessing, IMU, calibration). Details: [README](husmart_sensor_kit/README.md) |
| `diag_graph/` | Autoware diagnostic graph (MRM triggers); changes listed in `README.txt` |
| `planning_common/`, `planning_preset/`, `planning_crosswalk/` | Planning parameters (50 km/h limit, yielding to pedestrians at crosswalks) |
| `overtake/overtake_manager.py` | Overtake manager (MOBIL + IDM, keep-right rules) |
| `harita/beytepe_yeni_v2/` | Lanelet2 map (generated from RoadRunner, corrected by hand) and projector settings |
| `roadrunner/` | RoadRunner sources (`.rrscene`, `.xodr`, `.geojson`), the first Autoware maps, conversion notes |
| `beytepe_tests_v2/` | Tour scenarios (`analysis/specs_*.json`), tour runner, analysis scripts, older test reports |
| `docs/ust_bakis/` | Top-down views of the simulation map (with buildings) and `beytepe_sim.kml` for Google Earth |
| `isinla.sh`, `konum.sh` | Teleport the vehicle in AWSIM / print its position (section 3) |
| `tools/beytepe_duzenle/` | Map / scene editing tool for the gate road and the median (section 5) |
| `tools/` | Other helpers (texture generation, bag inspection) |
| `run_e2e_sim.sh`, `run_planning_sim.sh`, `record_bag.sh`, `run_unityhub.sh` | Old scripts (Shinjuku demo / Lexus sensors); not needed for Beytepe |

## 7. Relation to the lab's lidar work

The lidar names, layout, IP/port table and the ring-filter fix come from the lab kit
[HUSMART-AV/Autoware_1.9.0_launch](https://github.com/HUSMART-AV/Autoware_1.9.0_launch).
Difference: there, Autoware's `sample_sensor_kit` package is modified and Autoware is built from source; here a separate package
(`husmart_sensor_kit`) is mounted into the official image. Two diagnostic checks that are disabled there
(localization sensor-fusion status, routing state) remain **enabled** here.

## 8. Known limitations

- Sensor poses are **estimates**; the vehicle has not been measured. Once it is, update
  `husmart_sensor_kit/.../sensors_calibration.yaml` and `awsim_ayar/beytepe-config-husmart.json` together.
- The JT128s are mounted upright at 0.5 m: low objects (below ≈0.4 m) within ~2 m of the body are not seen.
- Main gate (2026-11-13): rebuilt from a hand drawing over Google Earth imagery (U-turn south of the guardhouse, 3 exit lanes,
  2 entry lanes, bus-stop bay). The imagery is offset from the map by about 0.9 m west / 1.25 m south (measured with lidar), and
  the drawing was shifted by that amount. Not yet validated with a full tour; small details may still differ from reality.
- Buildings may be offset by 1-2 m (OpenStreetMap footprints).
- Real-vehicle drivers are not wired up (OT128: Nebula `Pandar128E4X`; JT128 needs Hesai's HesaiLidar_ROS_2.0).

## 9. License and sources

- This repo: Apache-2.0 (same as Autoware).
- AWSIM and the prebuilt player: code is Apache-2.0; AWSIM's assets and the built player are **CC BY-NC** (no commercial use). See the husmart-awsim `LICENSE`.
- Building data: © OpenStreetMap contributors, ODbL.
- Textures: ambientCG, CC0.
