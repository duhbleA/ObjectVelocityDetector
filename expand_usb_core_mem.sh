#!/bin/bash
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" 
   exit 1
fi
echo "Adding more usbcore buffer memory..."
echo 12096 > /sys/module/usbcore/parameters/usbfs_memory_mb
