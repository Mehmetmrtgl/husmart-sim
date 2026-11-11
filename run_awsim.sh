#!/usr/bin/env bash
# AWSIM Beytepe oynatıcısını Hüsmart sensörleriyle başlatır. AWSIM kendi ROS 2 kütüphaneleriyle gelir:
# ROS 2 source edilmiş bir ortamda ÇALIŞTIRILMAMALI, bu yüzden temiz ortam (env -i) kullanılır.
# Ön koşul (her açılışta, sudo): net.core.rmem_max=2147483647 ve `ip link set lo multicast on`.
# Kullanım: ./run_awsim.sh          Durdurmak: pkill -f AWSIM-Beytepe.x86_64
# --json_path verilmezse bu repodaki awsim_ayar/beytepe-config-husmart.json kullanılır (sensörler, NPC, gökyüzü).
# Oynatıcı yolu: AWSIM_BIN (varsayılan ~/awsim/Beytepe-Build/AWSIM-Beytepe.x86_64; README "AWSIM").
set -e
AWSIM_BIN=${AWSIM_BIN:-$HOME/awsim/Beytepe-Build/AWSIM-Beytepe.x86_64}
LOG=${AWSIM_LOG:-$HOME/awsim/awsim.log}
DIR=$(dirname "$(readlink -f "$0")")
case " $* " in *" --json_path "*) ;; *) set -- "$@" --json_path "$DIR/awsim_ayar/beytepe-config-husmart.json";; esac
[ -x "$AWSIM_BIN" ] || { echo "HATA: $AWSIM_BIN yok (derleyin: ./build_beytepe_awsim.sh, ya da GitHub Release'ten indirin)"; exit 1; }

if ! ip link show lo | grep -q MULTICAST; then
  echo "UYARI: lo üzerinde multicast kapalı. Önce: sudo ip link set lo multicast on"; exit 1
fi

env -i HOME="$HOME" USER="$USER" PATH=/usr/bin:/bin DISPLAY="${DISPLAY:-:21.0}" \
  XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}" \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  CYCLONEDDS_URI="file://$DIR/cyclonedds.xml" \
  setsid "$AWSIM_BIN" "$@" > "$LOG" 2>&1 < /dev/null &
echo "AWSIM başlatıldı (pid $!). Log: $LOG"
