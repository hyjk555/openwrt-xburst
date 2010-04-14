#!/bin/bash
VERSION="latest"

# use 'http' to download and flash images, use 'file' to flash images present in the <WORKING_DIR>
PROTOCOL="http"

# working directory
#WORKING_DIR="/tmp/NanoNote/${VERSION}"
WORKING_DIR="${HOME}/.qi/nanonote/ben/${VERSION}"

# where the verbose output goes to
LOG_FILE="${WORKING_DIR}/log.txt"

# URL to images ($URL/$VERSION/$[images])
BASE_URL_HTTP="http://downloads.qi-hardware.com/software/images/Ben_NanoNote_2GB_NAND"

# names of images
LOADER="openwrt-xburst-u-boot.bin"
KERNEL="openwrt-xburst-uImage.bin"
#ROOTFS="openwrt-xburst-rootfs.ubi"
ROOTFS="openwrt-xburst-qi_lb60-root.ubi"

###############

# version of script
__VERSION__="2.1.1"


# options for reflash bootloader, kernel or rootfs
B="TRUE"
K="TRUE"
R="TRUE"

while getopts d:l:bkr OPTIONS
do
    case $OPTIONS in
    d)     
        VERSION=$OPTARG # override version by first argument if passed
        B="TRUE"
        K="TRUE"
        R="TRUE"
        ;;
    l)
        WORKING_DIR=$OPTARG
        VERSION=
        PROTOCOL="file"
        B="TRUE"
        K="TRUE"
        R="TRUE"
        ;;
    *)
        echo "\
Usage: $0 [-d <version>] [-l <path to local images>] [-b] [-k] [-r] [-h] [-v]

without any arguments, I will download and flash the latest official images
(includes bootloader, kernel and rootfs)

     -d <>  I will download and flash a specific version of official images
            (includes bootloader, kernel and rootfs)

     -l <>  I will flash images present in <arg>
            (includes bootloader, kernel and rootfs -
             missing files will be skipped)

     -h     you already found out

 
reflash script for qi-hardware Ben NanoNote

written by: Mirko Vogt (mirko.vogt@sharism.cc)
            Xiangfu Liu (xiangfu@sharism.cc)

version: ${__VERSION__}

Please report bugs to developer@lists.qi-hardware.com"
        exit 1
        ;;
    esac
done

# if no arguments were given
if [ "$#" == "0" ]; then
    B="TRUE"
    K="TRUE"
    R="TRUE"
fi

# create working directory
mkdir -p ${WORKING_DIR}
date > "${LOG_FILE}" # purge logfile if exists

function abort() {
    log "$1"
    log "==="
    log "fatal error occured - ABORT"
    exit 1
}

function log() {
    echo "$1"
    echo "$1" >> "${LOG_FILE}"
}

[ "$(whoami)" == "root" ] || abort "this script must be run as root"

log "working dir:      ${WORKING_DIR}"
log "chosed method:    ${PROTOCOL}"
test ${VERSION} && log "chosed version:   ${VERSION}"
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
		    -o "${LOG_FILE}" \
		    -P "${WORKING_DIR}" \
		    "${BASE_URL_HTTP}/${VERSION}/${LOADER}"
	fi
	if [ "$K" == "TRUE" ]; then
		log "fetching kernel..."
		wget \
		    -o "${LOG_FILE}" \
		    -P "${WORKING_DIR}" \
		    "${BASE_URL_HTTP}/${VERSION}/${KERNEL}"
	fi
	if [ "$R" == "TRUE" ]; then
		log "fetching rootfs..."
		wget \
		    -o "${LOG_FILE}" \
		    -P "${WORKING_DIR}" \
		    "${BASE_URL_HTTP}/${VERSION}/${ROOTFS}"
	fi
    fi
fi

log "booting device..."
usbboot -c "boot" > "${LOG_FILE}" || abort "can't boot device - xburst-tools setup correctly? device in boot-mode? device connected?"

if [ "$B" == "TRUE" ]; then
	log "flashing bootloader..."
	usbboot -c "nprog 0 ${WORKING_DIR}/${LOADER} 0 0 -n" >> "${LOG_FILE}"
fi
if [ "$K" == "TRUE" ]; then
	log "flashing kernel..."
	usbboot -c "nprog 1024 ${WORKING_DIR}/${KERNEL} 0 0 -n" >> "${LOG_FILE}"
fi
if [ "$R" == "TRUE" ]; then
	log "erase nand rootfs partition..."
	usbboot -c "boot;nerase 16 4079 0 0" >> "${LOG_FILE}"
	log "flashing rootfs..."
	usbboot -c "nprog 2048 ${WORKING_DIR}/${ROOTFS} 0 0 -n" >> "${LOG_FILE}"
fi
log "done"
