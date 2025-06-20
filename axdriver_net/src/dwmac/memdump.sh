#!/bin/bash
# save as memdump.sh
addr=$1
count=$2

for ((i=0; i<count; i++)); do
    raddr=$((addr + i*4))
    printf "0x%08X: " $raddr
    devmem $raddr 32
done

# ./memdump.sh 0x17000000 128

