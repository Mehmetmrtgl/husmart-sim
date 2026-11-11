TOUR ANALYSIS SCRIPTS (Hüsmart / Beytepe)
=========================================
The Python scripts in this folder read and inspect a tour recording (ros2 bag, MCAP format). They do not change any
setting of Autoware, AWSIM or the map.


1) HOW TO RUN
-------------
The scripts use the ROS 2 libraries (rosbag2_py), so they run inside the Autoware image.
In the container this folder is /an and the recordings folder (~/autoware_bags) is /bags.

Example: summary of tour 11.
  B=<repo>/beytepe_tests_v2/analysis/tur_analiz
  IMG=ghcr.io/autowarefoundation/autoware@sha256:b7490c70ba67c85a20d94feba68137ba911b5032c621353d50f0c63d3ee63977
  N=11; A=~/autoware_bags/husmart_v2_tur_${N}_analiz; mkdir -p $A
  docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -e MPLCONFIGDIR=/tmp -v ~/autoware_bags:/bags -v $B:/an:ro $IMG \
    bash -c "source /opt/autoware/setup.bash; python3 /an/analiz_surus.py /bags/husmart_v2_tur_$N /bags/husmart_v2_tur_${N}_analiz" \
    > $A/cikti.txt 2>&1

For another script, change only the "python3 /an/..." part and the output file name (list in section 2).
If a script only reads, mounting the recordings folder read-only is safer: -v ~/autoware_bags:/bags:ro

Abbreviations:
  BAG    = /bags/husmart_v2_tur_NN (the recording folder of one tour)
  t0 t1  = seconds from the start of the recording
  x,y    = map coordinates in metres


2) SCRIPTS
----------
analiz_surus.py BAG OUTPUT_DIR
    Tour summary: speed, localization error (against AWSIM's noise-free GNSS), NDT scores.
    Writes ozet.json and plots (.png) to OUTPUT_DIR. The recordings folder must be mounted writable (without :ro).
    Reads the roof lidar from /sensing/lidar/ot128_top/... (since 2026-11-11) or the older /sensing/lidar/top/...

analiz_yaya_tur.py BAG
    For every crosswalk pass: vehicle speed, STOP decision, and the pedestrian's closest distance to the vehicle body.

analiz_yaya.py BAG t0 t1
    One pedestrian event: crosswalk and run_out planning factors (TTC/TTV), pedestrians and the vehicle.

analiz_yaya_karar.py BAG t0 t1
    Decision sequence of the crosswalk module: STOP or SLOW DOWN, and whether the pedestrian was judged safe.

analiz_ndt_uyari.py BAG [BAG2 ...]
    NDT warning periods: duration, message, position and vehicle speed of each warning.

analiz_ndt_salinim.py BAG [BAG2 ...]
    Places where the NDT oscillation counter and iteration count are high.

analiz_ndt_olay.py BAG t0 t1
    Cycle-by-cycle dump of one NDT event: iterations, NVTL, time, error against the true pose, speed.

analiz_fren.py BAG x,y [x2,y2 ...]
    Finds when the vehicle stopped at the given place and dumps what happened before and after the hard brake.

analiz_uturn.py BAG x,y [radius_m]
    Difference between the planned path and the actual path at a place (e.g. a U-turn).

analiz_harita_uyum.py BAG TILES t1 [t2 ...]
    How well a lidar scan matches the point cloud map. Mount the tiles separately:
      -v <repo>/harita/beytepe_yeni_v2_karolar:/k:ro   and use /k as TILES

analiz_kamera.py BAG OUTPUT_DIR
    ZED X stereo timing, stereo depth compared with the roof lidar, MTi-7 noise at standstill.

pcd_karola.py input.pcd OUTPUT_DIR [50]
    Splits a single-file PCD map into 50 m tiles. No Docker needed:
      python3 -I pcd_karola.py <repo>/harita/beytepe_yeni_v2/pointcloud_map.pcd <new_folder> 50
    If the PCD map changes, harita/beytepe_yeni_v2_karolar must be regenerated with this.

tanilama_deneyi/test_hysteresis.py   (with test_graph.yaml and test_graph2.yaml)
    Tests the "hysteresis" and "warn-to-ok" behaviour of the diagnostic graph in a separate ROS domain
    (ROS_DOMAIN_ID=98, NO --net host). It does not touch a running Autoware. Command:
      docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -e ROS_DOMAIN_ID=98 -v $B/tanilama_deneyi:/an:ro $IMG bash -c 'source /opt/autoware/setup.bash; ros2 run autoware_diagnostic_graph_aggregator aggregator_node --ros-args --params-file /opt/autoware/share/autoware_diagnostic_graph_aggregator/config/default.param.yaml -p graph_file:=/an/test_graph.yaml -p use_command_mode_mappings:=false -p use_operation_mode_availability:=false > /tmp/agg.log 2>&1 & sleep 4; python3 /an/test_hysteresis.py'


3) WHERE THE OUTPUTS GO
-----------------------
Each tour's results live in their own folder: ~/autoware_bags/husmart_v2_tur_NN_analiz/
  cikti.txt, ozet.json, *.png   analiz_surus.py
  yaya.txt                      analiz_yaya_tur.py
  ndt_uyari.txt                 analiz_ndt_uyari.py
  fren.txt                      analiz_fren.py
  mrm.txt                       MRM (emergency stop) causes extracted from /rosout (no separate script)


4) BEFORE CHANGING A SCRIPT
---------------------------
Back it up first, e.g.:  cp analiz_surus.py ~/yedek/analiz_surus.py.oncesi
