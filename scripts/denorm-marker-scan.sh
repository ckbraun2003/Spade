#!/usr/bin/env bash
# Asserts that an installed Spade, or a built spade_compute archive, carries no
# trace of the measurement build (SPADE_MEASURE_UNPINNED_DENORMS; docs/design/
# core/plans/2026-10-04-nvidia-denorm-measurement-plan.md). spade_compute
# compiles the marker string "SPADE_MEASURE_UNPINNED_DENORMS" only under that
# option, and a static archive keeps every object, so a byte search of the
# archive finds it. The installed headers must not name the option either.
#
#   scripts/denorm-marker-scan.sh <install prefix | archive>...
#
# A prefix must hold a spade_compute archive (lib*/libspade_compute.a, or
# lib*/spade_compute.lib on Windows) and an include/ directory. A missing one
# FAILS (TD-5), so an empty or wrong prefix never passes; a SPADE_VULKAN=OFF
# prefix has no spade_compute and is not a target for this scan.
#
# Exit: 0 clean, 1 a trace or a missing archive, 2 usage.
set -u
marker=SPADE_MEASURE_UNPINNED_DENORMS
[ $# -ge 1 ] || { echo "usage: scripts/denorm-marker-scan.sh <install prefix | archive>..." >&2; exit 2; }
rc=0

scan_archive() {
    if grep -q -a -F "$marker" "$1"; then
        echo "denorm-marker-scan: FAIL $1 carries \"$marker\": it was built with the measurement option"
        rc=1
    else
        echo "denorm-marker-scan: clean $1"
    fi
}

for target in "$@"; do
    if [ -f "$target" ]; then
        scan_archive "$target"
        continue
    fi
    if [ ! -d "$target" ]; then
        echo "denorm-marker-scan: FAIL $target: no such prefix or archive"
        rc=1
        continue
    fi
    # Each lib*/ directory searched as itself: a `find -path` pattern built
    # from the prefix would read a Windows path's backslashes as escapes.
    found=0
    for libdir in "$target"/lib*; do
        [ -d "$libdir" ] || continue
        while IFS= read -r archive; do
            found=1
            scan_archive "$archive"
        done < <(find "$libdir" -type f \( -name libspade_compute.a -o -name spade_compute.lib \))
    done
    if [ "$found" = 0 ]; then
        echo "denorm-marker-scan: FAIL $target: no spade_compute archive under lib*/ (an empty prefix never passes)"
        rc=1
    fi
    if [ ! -d "$target/include" ]; then
        echo "denorm-marker-scan: FAIL $target: no include/ directory"
        rc=1
    elif hits=$(grep -r -l -F "$marker" "$target/include"); then
        echo "denorm-marker-scan: FAIL the installed headers name $marker:"
        printf '  %s\n' $hits
        rc=1
    else
        echo "denorm-marker-scan: clean $target/include"
    fi
done
exit $rc
