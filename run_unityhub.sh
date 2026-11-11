#!/usr/bin/env bash
# Unity Hub (sudo'suz, ~/opt/unityhub altına açılmış .deb). Yalnızca Unity hesabıyla giriş / lisans için.
# AWSIM belgesi: Unity Hub/Editor ROS 2 source edilmiş ortamda çalıştırılmamalı -> env -i
HUB=$HOME/opt/unityhub/root/usr/lib/unityhub/unityhub
env -i HOME="$HOME" USER="$USER" PATH=/usr/bin:/bin DISPLAY="${DISPLAY:-:21.0}" \
  XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}" \
  DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=/run/user/$(id -u)/bus}" LANG="${LANG:-en_US.UTF-8}" \
  setsid "$HUB" --no-sandbox "$@" >> "$HOME/opt/unityhub/hub.log" 2>&1 < /dev/null &
echo "Unity Hub başlatıldı (pid $!). Log: ~/opt/unityhub/hub.log"
