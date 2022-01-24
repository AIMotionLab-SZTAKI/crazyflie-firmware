#!/bin/bash
#
# Builds release versions of the Crazyflie firmware and the drone show app
# by re-packaging an official Crazyflie firmware release

# Our firmware version number
VERSION=`date +%Y.%m`

# Variants of the firmware to build
VARIANTS="stock-uwb stock-lh show-proto-v1 show-proto-v2-uwb show-proto-v2-lh"

# Upstream Crazyflie version on top of which this firmware is built
UPSTREAM_CF_VERSION=2021.06

###############################################################################

set -e

CURRENT_DIR="`pwd`"

cd "`dirname $0`"/../../..
REPO_ROOT="`pwd`"

APP_ROOT="${REPO_ROOT}/app"

# TMP_DIR=$(mktemp -d -t cf-release-$(date +%Y%m%dT%H%M%S)-XXXXXXXXXX)
# trap "rm -rf ${TMP_DIR}" EXIT
TMP_DIR="/tmp/cf-build"
rm -rf "${TMP_DIR}"
mkdir -p "${TMP_DIR}"

echo "Downloading official Crazyflie release, version ${UPSTREAM_CF_VERSION}..."
curl -sLo "${TMP_DIR}/firmware.zip" https://github.com/bitcraze/crazyflie-release/releases/download/${UPSTREAM_CF_VERSION}/firmware-cf2-${UPSTREAM_CF_VERSION}.zip

cd "${APP_ROOT}"
make clean
for VARIANT in ${VARIANTS}; do
    echo "Compiling custom firmware for ${VARIANT}..."
    make CF_MODEL=${VARIANT}
    ZIP_NAME="skybrush-cf2_${VARIANT}_${VERSION}"
    rm -f "${TMP_DIR}"/cf2-*.bin
    cp cf2-*.bin "${TMP_DIR}"
    make clean

    cp "${TMP_DIR}/firmware.zip" "${TMP_DIR}/${ZIP_NAME}.zip"
    mv ${TMP_DIR}/cf2-*.bin "${TMP_DIR}/cf2-${UPSTREAM_CF_VERSION}.bin"
    ( cd "${TMP_DIR}" && zip "${ZIP_NAME}.zip" "cf2-${UPSTREAM_CF_VERSION}.bin" )
    rm -f "${TMP_DIR}"/cf2-*.bin
done

cd "${CURRENT_DIR}"
mv "${TMP_DIR}"/skybrush-cf2*.zip .

echo ""
echo "========================================================================"
echo ""

echo "Firmware built in:"
ls skybrush-cf2*.zip

