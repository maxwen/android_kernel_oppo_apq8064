#!/bin/bash
export USE_CCACHE=1
export ARCH=arm
export PATH=${PATH}:~/toolchain/android-4.6/bin
export CROSS_COMPILE=arm-eabi-
config=color_rom_defconfig

if [ ! -f out/zImage ]
then
    if [ ! -f out/kernel/noclean ]
    then
	echo "--- Cleaning up ---"
	rm -rf out/kernel
	rm -rf out/system
	make mrproper
    fi

	mkdir -p out/kernel
	echo "--- Making defconfig ---"
	make O=out/kernel $config
	echo "--- Building kernel ---"
	make -j4 O=out/kernel
	touch out/kernel/noclean

  if [ -f out/kernel/arch/arm/boot/zImage ]
  then
	echo "--- Installing modules ---"
	make -C out/kernel INSTALL_MOD_PATH=.. modules_install
	mdpath=`find out/lib/modules -type f -name modules.order`

	  if [ "$mdpath" != "" ]
	  then
		mpath=`dirname $mdpath`
		ko=`find $mpath/kernel -type f -name *.ko`
		for i in $ko
		do "$CROSS_COMPILE"strip --strip-unneeded $i
		mkdir -p out/system/lib/modules
		mv $i out/system/lib/modules
		done
	  else
	  echo "--- No modules found ---"
	  fi

	cp out/kernel/arch/arm/boot/zImage out
	rm -f out/kernel/noclean
	rm -rf out/lib
  else
	exit 0
  fi
fi

if [ -f out/boot.img ]
then
  if [ ! -d out/boot/boot.img-ramdisk ] && [ ! -f out/ramdisk.gz ]
  then
	mkdir -p out/boot
	cp out/boot.img out/boot
	cp scripts/mkbootfs out/boot
	cp scripts/extract-ramdisk.pl out/boot
	cd out/boot
	echo "--- Extracting ramdisk ---"
	./extract-ramdisk.pl boot.img
	./mkbootfs boot.img-ramdisk | gzip > ramdisk.gz
	rm -f boot.img
	rm -f mkbootfs
	rm -f extract-ramdisk.pl
	cd ../..
	mv out/boot/ramdisk.gz out
	exit 0
  fi

  if [ -d out/boot/boot.img-ramdisk ] || [ -f out/ramdisk.gz ]
  then
	mkdir -p out/boot
	mv out/zImage out/boot
	mv out/boot.img out/boot
	cp scripts/mkbootfs out/boot
	cp scripts/mkbootimg out/boot

	  if [ -d out/boot/boot.img-ramdisk ]
	  then
		rm -f out/ramdisk.gz
		cd out/boot
		./mkbootfs boot.img-ramdisk | gzip > ramdisk.gz
	  fi

	  if [ -f out/ramdisk.gz ]
	  then
		mv out/ramdisk.gz out/boot
		cd out/boot
	  fi

	cmd_line=`od -A n --strings -j 64 -N 512 boot.img`
	temp=`od -A n -H -j 20 -N 4 boot.img | sed 's/ //g'`
	ramdisk=0x$temp
	base_temp=`od -A n -h -j 14 -N 2 boot.img | sed 's/ //g'`
	zeros=0000
	base=0x$base_temp$zeros
	echo "--- Creating boot.img ---"
	./mkbootimg --kernel zImage --ramdisk ramdisk.gz --cmdline "$cmd_line" -o newboot.img --base $base --ramdiskaddr $ramdisk
	cd ../..
	mv out/boot/ramdisk.gz out
	mv out/boot/newboot.img out/boot.img
	rm -rf out/boot
  fi

else
	echo "--- No boot.img found ---"
	exit 0
fi
