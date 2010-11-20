#!/bin/bash
# version of me
__VERSION__="2011-11-20"

# use 'http' to download and flash images, use 'file' to flash images present in the <WORKING_DIR>
PROTOCOL="http"

# NanoNote images Version
VERSION="latest"

# working directory
WORKING_DIR="${HOME}/.qi/nanonote/ben/${VERSION}"

# where the verbose output goes to
LOG_FILE="${WORKING_DIR}/log.txt"

# URL to images ($URL/$VERSION/$[images])
BASE_URL_HTTP="http://downloads.qi-hardware.com/software/images/NanoNote/Ben"

# names of images
LOADER="openwrt-xburst-qi_lb60-u-boot.bin"
KERNEL="openwrt-xburst-qi_lb60-uImage.bin"
ROOTFS="openwrt-xburst-qi_lb60-root.ubi"

# options for reflash bootloader, kernel,  rootfs
B="TRUE"
K="TRUE"
R="TRUE"

while getopts d:l:h OPTIONS
do
    case $OPTIONS in
    d)     
        VERSION=$OPTARG # override version by first argument if passed
        ;;
    l)
        WORKING_DIR=$OPTARG
        PROTOCOL="file"
        VERSION=
        ;;
    *)
        echo "\

Usage: $0 [-d <version>] [-l <path to local images>] [-h]
     -d <>  I will download and flash a specific version of OpenWrt images

     -l <>  I will flash images present in <arg>
            (missing files will be skipped)

     -h     you already found out

without any arguments, I will download and flash the latest OpenWrt images
(includes bootloader, kernel and rootfs)

OpenWrt reflash script for qi-hardware Ben NanoNote
written by: Mirko Vogt (mirko.vogt@sharism.cc)
            Xiangfu Liu (xiangfu@sharism.cc)

                                                     version: ${__VERSION__}
Please report bugs to developer@lists.qi-hardware.com"
        exit 1
        ;;
    esac
done

# create working directory
mkdir -p ${WORKING_DIR}
# purge logfile if exists
date > "${LOG_FILE}"

log() {
    echo -e "$1"
    echo -e "$1" >> "${LOG_FILE}"
}

abort() {
    log "==="
    log "fatal error occured - ABORTED"
    log "==="
    log "$1"
    log "==="
    log "Before reporting this as a bug"
    log "please ensure you're using the latest available version of"
    log "  this reflash script"
    log "  the xburst-tools"
    exit 1
}

[ "$(whoami)" == "root" ] || abort "this script must be run as root"

log "working dir:      ${WORKING_DIR}"
log "chosen method:    ${PROTOCOL}"
test ${VERSION} && log "chosen version:   ${VERSION}"
log "==="

if [ "$PROTOCOL" == "http" ]; then

    MD5SUMS_SERVER=$(wget -O - ${BASE_URL_HTTP}/${VERSION}/md5sums 2> /dev/null | grep -E "(${LOADER}|${KERNEL}|${ROOTFS})" | sort)
    [ "${MD5SUMS_SERVER}" ] || abort "can't fetch files from server"
    
    MD5SUMS_LOCAL=$( (cd "${WORKING_DIR}" ; md5sum --binary "${LOADER}" "${KERNEL}" "${ROOTFS}" 2> /dev/null) | sort )

    if [ "${MD5SUMS_SERVER}" == "${MD5SUMS_LOCAL}" ]; then
        log "present files are identical to the ones on the server - do not download them again"
    else
        rm -f "${WORKING_DIR}/${LOADER}" "${WORKING_DIR}/${KERNEL}" "${WORKING_DIR}/${ROOTFS}"
	if [ "$B" == "TRUE" ]; then
		log "fetching bootloader..."
		wget \
		    -a "${LOG_FILE}" \
		    -P "${WORKING_DIR}" \
		    "${BASE_URL_HTTP}/${VERSION}/${LOADER}"
	fi
	if [ "$K" == "TRUE" ]; then
		log "fetching kernel..."
		wget \
		    -a "${LOG_FILE}" \
		    -P "${WORKING_DIR}" \
		    "${BASE_URL_HTTP}/${VERSION}/${KERNEL}"
	fi
	if [ "$R" == "TRUE" ]; then
		log "try fetching .ubi.bz2 rootfs..."
		wget \
		    -a "${LOG_FILE}" \
		    -P "${WORKING_DIR}" \
		    "${BASE_URL_HTTP}/${VERSION}/${ROOTFS}.bz2" && \
		    (cd ${WORKING_DIR}; tar xf ${ROOTFS}.bz2)

		if [ "$?" == "8" ]; then
		    log "fetching .ubi rootfs..."
		    wget \
			-a "${LOG_FILE}" \
			-P "${WORKING_DIR}" \
			"${BASE_URL_HTTP}/${VERSION}/${ROOTFS}"
		fi
	fi
    fi
fi

log "booting device..."
usbboot -c "boot" >> "${LOG_FILE}" || abort "can't boot device - xburst-tools setup correctly? device in boot-mode? device connected?"

if [ "$B" == "TRUE" ]; then
	log "flashing bootloader..."
	tmp=$(usbboot -c "nprog 0 ${WORKING_DIR}/${LOADER} 0 0 -n" 3>> "${LOG_FILE}" 2>&1 >&3)
	test "${tmp}" && abort "error while flashing bootloader:\n${tmp}"
fi
if [ "$K" == "TRUE" ]; then
	log "flashing kernel..."
	tmp=$(usbboot -c "nprog 1024 ${WORKING_DIR}/${KERNEL} 0 0 -n" 3>> "${LOG_FILE}" 2>&1 >&3)
	test "${tmp}" && abort "error while flashing kernel:\n${tmp}"
fi
if [ "$R" == "TRUE" ]; then
	log "erase nand rootfs partition..."
	usbboot -c "boot;nerase 16 1024 0 0" >> "${LOG_FILE}" 2>&1
	log "flashing rootfs..."
	tmp=$(usbboot -c "nprog 2048 ${WORKING_DIR}/${ROOTFS} 0 0 -n" 3>> "${LOG_FILE}" 2>&1 >&3)
	test "${tmp}" && abort "error while flashing rootfs:\n${tmp}"
fi
log "done"
