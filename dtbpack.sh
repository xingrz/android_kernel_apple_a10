#!/bin/bash

DTBPATH='arch/arm64/boot/dts/hx/hx-h9p-%s.dtb'
DEVICES='D10 D101 D11 D111 N112'
OUTPUT='dtbpack'

echo -n 'Cows' > dtbpack
for DEV in $DEVICES; do
    DEV_LC=`echo -n $DEV | tr '[:upper:]' '[:lower:]'`
    DTB=`printf "$DTBPATH" $DEV_LC`
    echo -ne $DEV'\0' >> dtbpack
    SIZE=`stat -c'%s' $DTB`
    printf '%08x' $SIZE | xxd -r -p >> dtbpack
    cat $DTB >> dtbpack
done
echo -ne '\0\0\0\0\0' >> dtbpack
