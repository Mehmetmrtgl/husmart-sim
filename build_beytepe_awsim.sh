#!/usr/bin/env bash
# AWSIM projesinde Beytepe sahnesini kurar ve Linux oynatıcısı olarak derler (Unity batchmode, ekran açmaz).
# Ön koşul: Unity Hub'da Unity hesabıyla giriş yapılmış ve Personal lisans etkin olmalı.
# Çıktı:  ~/awsim/Beytepe-Build/AWSIM-Beytepe.x86_64
# Log:    ~/awsim/beytepe_unity_build.log      Özet rapor: ~/awsim/beytepe_build_report.txt
# Gerçek veriden dünya (2026-11-09) için ortam değişkenleriyle:
#   BEYTEPE_WORLD=gercek BEYTEPE_OSM=$HOME/autoware_map/beytepe_gercek_v1/lanelet2_map.osm \
#   BEYTEPE_BUILD=$HOME/awsim/Beytepe-Gercek-Build/AWSIM-Beytepe.x86_64 ~/autoware/build_beytepe_awsim.sh
# Değişken verilmezse eskisi gibi RoadRunner modeli + beytepe_yeni_v2 + ~/awsim/Beytepe-Build derlenir.
set -e
UNITY=${UNITY:-$HOME/Unity/Hub/Editor/6000.0.61f1/Editor/Unity}
PROJECT=${AWSIM_SRC:-$HOME/awsim/AWSIM-src}   # husmart-awsim reposunun klonu (beytepe dalı)
LOG=${BEYTEPE_LOG:-$HOME/awsim/beytepe_unity_build.log}
# AWSIM belgesi: Unity, ROS 2 source edilmiş ortamda çalıştırılmamalı -> temiz ortam
exec env -i HOME="$HOME" USER="$USER" PATH=/usr/bin:/bin LANG=en_US.UTF-8 \
  BEYTEPE_OSM="${BEYTEPE_OSM:-$HOME/autoware_map/beytepe_yeni_v2/lanelet2_map.osm}" \
  BEYTEPE_BUILD="${BEYTEPE_BUILD:-$HOME/awsim/Beytepe-Build/AWSIM-Beytepe.x86_64}" \
  BEYTEPE_WORLD="${BEYTEPE_WORLD:-roadrunner}" \
  "$UNITY" -batchmode -nographics -quit -projectPath "$PROJECT" \
  -executeMethod Beytepe.EditorTools.BeytepeSceneBuilder.BuildAll -logFile "$LOG"
