#!/bin/sh
set -eu
case "${1:-}" in
    cn|gb) region=$1; shift ;;
    *) echo "Usage: $0 cn|gb [idf.py arguments]" >&2; exit 2 ;;
esac
cd "$(dirname "$0")/.."
root=$(pwd)
build="$root/build-$region"
if [ "$#" -eq 0 ]; then set -- build; fi
exec idf.py -B "$build" -D "SDKCONFIG=$build/sdkconfig" \
    -D "SDKCONFIG_DEFAULTS=$root/sdkconfig.defaults;$root/configs/hover-$region.defaults" "$@"
