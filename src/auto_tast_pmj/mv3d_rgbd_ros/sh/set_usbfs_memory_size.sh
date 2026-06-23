#!/bin/sh 
# *************************************************************************************** 
# 
# 
# *************************************************************************************** 
 
DISTRIBUTION="unknown" 
 
if [ -f /etc/lsb-release ]; then 
    if [ -f /lib/lsb/init-functions ]; then 
        DISTRIBUTION=`grep DISTRIB_ID /etc/lsb-release | sed -e 's/DISTRIB_ID=//'` 
    fi 
elif [ -f /etc/os-release ]; then 
    if [ -f /lib/lsb/init-functions ]; then 
        DISTRIBUTION=`grep -w NAME /etc/os-release | sed 's/[^"]*"\([^"]*\)"/\1/'` 
        if [ "$DISTRIBUTION" == "Debian GNU/Linux" ] ; then 
            # Debian is close enough for the purposes of this script 
            DISTRIBUTION="Ubuntu" 
        fi 
    fi 
fi 
 
if [ "$DISTRIBUTION" != "Ubuntu" ]; then 
    echo "" 
    echo "Unsupported distribution." 
    echo "" 
fi 
 
 
if [ $(id -u) -ne 0 ] ; then 
    echo "User has insufficient privileges. Please try sudo command." 
    exit 1 
fi 
 
echo "Setting usbfs memory size to 2000" 
sh -c 'echo 2000 > /sys/module/usbcore/parameters/usbfs_memory_mb' 
# 设置 usbfs 内存大小（单位：MB，可修改为所需值） 
USBFS_MEMORY_MB=2000 
GRUB_CFG="/etc/default/grub" 
 
# 检查是否已存在 usbcore.usbfs_memory_mb 参数 
if grep -q "usbcore\.usbfs_memory_mb=" "$GRUB_CFG"; then 
    # 存在则替换值 
    sed -i "s/usbcore\.usbfs_memory_mb=[0-9]*/usbcore\.usbfs_memory_mb=$USBFS_MEMORY_MB/g" "$GRUB_CFG" 
else 
    # 不存在则添加到启动参数 
    # 备份原 grub 配置 
    cp /etc/default/grub /etc/default/grub.bak 
    if grep -q "^GRUB_CMDLINE_LINUX_DEFAULT" "$GRUB_CFG"; then 
        sed -i "s/\(GRUB_CMDLINE_LINUX_DEFAULT=\"[^\"]*\)\"/\1 usbcore.usbfs_memory_mb=$USBFS_MEMORY_MB\"/" "$GRUB_CFG" 
    else 
        echo "GRUB_CMDLINE_LINUX_DEFAULT=\"usbcore.usbfs_memory_mb=$USBFS_MEMORY_MB\"" >> "$GRUB_CFG" 
    fi 
fi 
# 更新 grub 配置 
update-grub 
 
echo "usbfs_memory_mb=$USBFS_MEMORY_MB"