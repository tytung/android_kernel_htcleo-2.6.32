#!/bin/sh

KERNELBASEDIR=/ics/kernel/out

make htcleo_defconfig
make ARCH=arm CROSS_COMPILE=/home/securecrt/tools/arm-2010q1/bin/arm-none-eabi- zImage -j8 && make ARCH=arm CROSS_COMPILE=/home/securecrt/tools/arm-2010q1/bin/arm-none-eabi- modules -j8

if [ -f arch/arm/boot/zImage ]; then

mkdir -p $KERNELBASEDIR/
rm -rf $KERNELBASEDIR/boot/*
rm -rf $KERNELBASEDIR/system/*
mkdir -p $KERNELBASEDIR/boot
mkdir -p $KERNELBASEDIR/system/
mkdir -p $KERNELBASEDIR/system/lib/
mkdir -p $KERNELBASEDIR/system/lib/modules

cp arch/arm/boot/zImage $KERNELBASEDIR/boot/zImage

make ARCH=arm CROSS_COMPILE=/home/securecrt/tools/arm-2010q1/bin/arm-none-eabi- INSTALL_MOD_PATH=$KERNELBASEDIR/system/lib/modules modules_install -j8

cd $KERNELBASEDIR/system/lib/modules
find -iname *.ko | xargs -i -t cp {} .
rm -rf $KERNELBASEDIR/system/lib/modules/lib
stat $KERNELBASEDIR/boot/zImage
cd ../../../
zip -r tytung_HWA_kernel.`date +"%Y%m%d_%H_%M"`.zip boot system
else
echo "Kernel STUCK in BUILD! no zImage exist"
fi

