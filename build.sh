#!/bin/sh

export KERNELBASEDIR=$PWD/../KK_Kernel_update-zip-files
#export TOOLCHAIN=$HOME/CodeSourcery/Sourcery_G++_Lite/bin/arm-none-eabi-
export TOOLCHAIN=$HOME/arm-2010q1/bin/arm-none-eabi-

export KERNEL_FILE=HTCLEO-Kernel_2.6.32_tytung_kitkat

rm arch/arm/boot/zImage
make htcleo_defconfig
make ARCH=arm CROSS_COMPILE=$TOOLCHAIN zImage -j8 && make ARCH=arm CROSS_COMPILE=$TOOLCHAIN modules -j8

if [ -f arch/arm/boot/zImage ]; then

mkdir -p $KERNELBASEDIR/
rm -rf $KERNELBASEDIR/boot/*
rm -rf $KERNELBASEDIR/system/lib/modules/*
mkdir -p $KERNELBASEDIR/boot
mkdir -p $KERNELBASEDIR/system/
mkdir -p $KERNELBASEDIR/system/lib/
mkdir -p $KERNELBASEDIR/system/lib/modules

cp -a arch/arm/boot/zImage $KERNELBASEDIR/boot/zImage

make ARCH=arm CROSS_COMPILE=$TOOLCHAIN INSTALL_MOD_PATH=$KERNELBASEDIR/system/lib/modules modules_install -j8

cd $KERNELBASEDIR/system/lib/modules
find -iname *.ko | xargs -i -t cp {} .
rm -rf $KERNELBASEDIR/system/lib/modules/lib
stat $KERNELBASEDIR/boot/zImage
cd ../../../
zip -r ${KERNEL_FILE}_`date +"%Y%m%d_%H_%M"`.zip boot system META-INF work
else
echo "Kernel STUCK in BUILD! no zImage exist"
fi

