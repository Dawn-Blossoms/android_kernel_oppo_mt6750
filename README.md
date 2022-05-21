## Branch Selection
[**Global Edition**](https://github.com/Dawn-Blossoms/android_kernel_oppo_mt6750/tree/Lollipop-Global) / [**CHN Edition**](https://github.com/Dawn-Blossoms/android_kernel_oppo_mt6750/tree/Lollipop-CHN) 

## Chart
| Codename | Device |
| :-: | :-: |
| oppo6750_15127 | A37M/T/TM |
| oppo6750_15131 | A59M/S F1S |
| oppo6750_16021 | A39 |
| oppo6755_15111 | R9M/TM/KM F1Plus|


### How to Build
```bash
$ apt install -yq make bc gcc lib32z1
$ git clone https://android.googlesource.com/platform/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9 -b android-6.0.1_r32 --depth=1
$ export Codename=(codename of your device)
$ export ARCH=arm64
$ export CROSS_COMPILE=$(pwd)/aarch64-linux-android-4.9/bin/aarch64-linux-android-
$ cd android_kernel_oppo_mt6750
$ make ${Codename}_debug_defconfig
# Optional Marcos can be found at Makefile
$ make -j$(nproc --all) 2>&1 | tee build.log
# Kernel: arch/arm64/boot/Image.gz-dtb
# Module: "hypnus" (Optional)
```
