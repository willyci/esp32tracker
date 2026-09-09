#!/usr/bin/env bash
#
# Build (and optionally flash) any sketch in this directory with a FULLY EXPLICIT board
# configuration.
#
# WHY THIS EXISTS
#
# Arduino IDE stores board options per sketch FOLDER and writes nothing into the folder, so
# a copied sketch silently inherits whatever the IDE happened to have selected. That has
# cost this project real hours: left-panel/right-panel were byte-identical in behaviour to a
# working pedal-panel and still booted to a black screen with no BLE, and the same class of
# problem produced an "ESP_I2S.h: No such file or directory" against a header that was
# present on disk (a library filtered out by the selected board's architecture).
#
# Everything that matters is on the command line here, so nothing can be inherited: board,
# PSRAM type, flash size, partition scheme, USB mode and CDC-on-boot are stated per sketch.
#
# USAGE
#   ./build.sh                       compile every sketch the installed core supports
#   ./build.sh left-panel            compile one
#   ./build.sh left-panel right-mini compile several
#   ./build.sh --clean left-panel    discard cached objects first
#   ./build.sh --upload -p COM7 left-panel
#   ./build.sh --options left-panel  list every valid FQBN option for that board
#
# Run from Git Bash. Needs arduino-cli on PATH:  https://arduino.github.io/arduino-cli/
#
set -uo pipefail
cd "$(dirname "$0")"

# ---------------------------------------------------------------------------
# Per-sketch configuration.
#
# Each row: sketch|required core major|FQBN
#
# The panel FQBN comes from Waveshare's own CI (ESP32-S3-Touch-AMOLED-2.06/docs/ci.md),
# which builds their examples on this exact hardware, with ONE deliberate change:
# CDCOnBoot=default (Disabled) instead of cdc. Their examples do not use IO19; ours puts the
# SoftPot wiper there, and IO19 is USB D-, so the port cannot enumerate and Serial must go
# to UART0 on RXD/TXD (44/43) instead.
#
# PSRAM=opi is NOT interchangeable with PSRAM=enabled: boards.txt maps `enabled` to "QSPI
# PSRAM" and `opi` to "OPI PSRAM". This module is a WROOM-1-N16R8 and the R8 part is 8 MB
# OCTAL PSRAM, so `enabled` leaves PSRAM dead.
#
# CORE SPLIT: the AMOLED panels need core 3.x (ESP_I2S.h, the ES8311 audio API, is 3.x-only).
# Every other board here needs 2.0.17, because NimBLE-Arduino 1.4.x crashes on 3.x with a
# Guru Meditation at BLE init. arduino-cli holds only one version of a platform at a time,
# so this script builds whichever group matches what is installed and names the rest.
# ---------------------------------------------------------------------------
PANEL_FQBN="esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default,PSRAM=opi,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB"
S3_PEDAL_FQBN="esp32:esp32:esp32s3:CDCOnBoot=cdc"
C3_FQBN="esp32:esp32:esp32c3:CDCOnBoot=cdc"

SKETCHES=(
  "left-panel|3|$PANEL_FQBN"
  "right-panel|3|$PANEL_FQBN"
  "pedal-panel|3|$PANEL_FQBN"
  # Diagnostic only: byte-identical content to pedal-panel, different folder and filename.
  # Isolates "a new sketch folder" from "the generated changes". Delete once it has served.
  "panel-probe|3|$PANEL_FQBN"
  # Bisect baseline: git HEAD (pre-session) with only its two broken string literals
  # repaired, so it compiles. None of this session's additions. Diagnostic only.
  "panel-base|3|$PANEL_FQBN"
  "left-foot|2|$S3_PEDAL_FQBN"
  "right-foot|2|$S3_PEDAL_FQBN"
  "dsa-foot|2|$S3_PEDAL_FQBN"
  "left-mini|2|$C3_FQBN"
  "right-mini|2|$C3_FQBN"
  "left|2|$C3_FQBN"
  "right|2|$C3_FQBN"
)

# arduino-cli is a NATIVE Windows binary, so it needs a Windows path here. Git Bash's
# automatic POSIX->Windows argument conversion cannot be relied on for this, so convert
# explicitly when cygpath exists and fall back to the POSIX form elsewhere (WSL, Linux).
LIBRARIES="$HOME/Documents/Arduino/libraries"
if command -v cygpath >/dev/null 2>&1; then
  LIBRARIES="$(cygpath -w "$LIBRARIES")"
fi
[ -d "$HOME/Documents/Arduino/libraries" ] || echo "warning: $LIBRARIES does not exist" >&2

# ---------------------------------------------------------------------------
CLEAN=""; UPLOAD=""; PORT=""; SHOW_OPTIONS=""; WANTED=()
while [ $# -gt 0 ]; do
  case "$1" in
    --clean)   CLEAN="--clean" ;;
    --upload)  UPLOAD="1" ;;
    --options) SHOW_OPTIONS="1" ;;
    -p)        shift; PORT="${1:-}" ;;
    -h|--help) sed -n '3,30p' "$0"; exit 0 ;;
    -*)        echo "unknown flag: $1" >&2; exit 2 ;;
    *)         WANTED+=("$1") ;;
  esac
  shift
done

command -v arduino-cli >/dev/null 2>&1 || {
  echo "arduino-cli not found on PATH."
  echo "Install: https://arduino.github.io/arduino-cli/latest/installation/"
  echo "Then:    arduino-cli core install esp32:esp32@3.3.11"
  exit 127
}

row_for() {   # sketch -> "sketch|core|fqbn", or empty
  local want="$1" row
  for row in "${SKETCHES[@]}"; do
    [ "${row%%|*}" = "$want" ] && { echo "$row"; return; }
  done
}

# --options: dump the board's real menu so an FQBN can be fixed from evidence, not guesswork
if [ -n "$SHOW_OPTIONS" ]; then
  [ ${#WANTED[@]} -eq 1 ] || { echo "--options takes exactly one sketch name" >&2; exit 2; }
  row="$(row_for "${WANTED[0]}")"
  [ -n "$row" ] || { echo "unknown sketch: ${WANTED[0]}" >&2; exit 2; }
  fqbn="${row##*|}"
  echo "== valid options for ${fqbn%%:*}:${fqbn#*:} =="
  arduino-cli board details --fqbn "${fqbn%%:*}:$(echo "$fqbn" | cut -d: -f2,3)"
  exit $?
fi

# Which core is actually installed? Only one version of a platform can be, so this decides
# which half of the repo is buildable right now.
INSTALLED="$(arduino-cli core list 2>/dev/null | awk '$1=="esp32:esp32"{print $2}')"
[ -n "$INSTALLED" ] || {
  echo "The esp32:esp32 platform is not installed."
  echo "  arduino-cli core install esp32:esp32@3.3.11   # for the AMOLED panels"
  echo "  arduino-cli core install esp32:esp32@2.0.17   # for everything else"
  exit 1
}
INSTALLED_MAJOR="${INSTALLED%%.*}"
echo "esp32:esp32 core installed: $INSTALLED  (major $INSTALLED_MAJOR)"
echo "libraries: $LIBRARIES"
echo

if [ ${#WANTED[@]} -eq 0 ]; then
  for row in "${SKETCHES[@]}"; do WANTED+=("${row%%|*}"); done
fi

if [ -n "$UPLOAD" ] && [ -z "$PORT" ]; then
  echo "--upload needs a port, e.g. -p COM7. Ports:"; arduino-cli board list; exit 2
fi
if [ -n "$UPLOAD" ] && [ ${#WANTED[@]} -ne 1 ]; then
  echo "--upload takes exactly one sketch (one board is attached to one port)" >&2; exit 2
fi

built=(); skipped=(); failed=()

for name in "${WANTED[@]}"; do
  row="$(row_for "$name")"
  if [ -z "$row" ]; then
    echo "!! unknown sketch: $name" >&2; failed+=("$name (unknown)"); continue
  fi
  need_major="$(echo "$row" | cut -d'|' -f2)"
  fqbn="${row##*|}"

  if [ "$need_major" != "$INSTALLED_MAJOR" ]; then
    # Not a failure: it is simply the other half of the repo. Say what to run.
    printf '.. %-13s SKIPPED - needs core %sx, have %s\n' "$name" "$need_major" "$INSTALLED"
    skipped+=("$name (needs core ${need_major}.x)")
    continue
  fi

  echo "== $name"
  echo "   $fqbn"
  if arduino-cli compile $CLEAN --fqbn "$fqbn" --libraries "$LIBRARIES" \
       --warnings default "$name"; then
    built+=("$name")
    if [ -n "$UPLOAD" ]; then
      echo "-- uploading $name to $PORT"
      arduino-cli upload -p "$PORT" --fqbn "$fqbn" "$name" || failed+=("$name (upload)")
    fi
  else
    rc=$?
    failed+=("$name (compile)")
    # The two failures this project actually hits, with the evidence to check.
    echo "   hint: 'No such file or directory' for a header that exists usually means the"
    echo "         library was filtered out by board architecture - check the FQBN above,"
    echo "         and run  ./build.sh --options $name  to see the board's real menu."
    echo "   hint: a stale object can survive a source fix - retry with --clean."
  fi
  echo
done

echo "---------------------------------------------------------------"
[ ${#built[@]}   -gt 0 ] && printf 'built:   %s\n' "${built[*]}"
[ ${#skipped[@]} -gt 0 ] && printf 'skipped: %s\n' "${skipped[*]}"
[ ${#failed[@]}  -gt 0 ] && printf 'FAILED:  %s\n' "${failed[*]}"

if [ ${#skipped[@]} -gt 0 ]; then
  other=$([ "$INSTALLED_MAJOR" = "3" ] && echo "2.0.17" || echo "3.3.11")
  echo
  echo "To build the skipped sketches, switch the core (only one version can be installed):"
  echo "  arduino-cli core install esp32:esp32@$other"
  echo "...then switch back with:"
  echo "  arduino-cli core install esp32:esp32@$INSTALLED"
fi

[ ${#failed[@]} -eq 0 ] || exit 1
