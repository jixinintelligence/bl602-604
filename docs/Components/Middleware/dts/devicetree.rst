device tree
===========

一、 介绍
---------

-  调试平台

   ``ubuntu18.04``

二、 dts 和 dtb 相互转换
------------------------

-  device tree 调试工具

   ``sudo sudo apt-get install device-tree-compiler``

-  dts 转 dtb

   ``dtc -I dts -O dtb -o *.dtb *.dts``

-  dtb 转 dts

   ``dtc -I dtb -O dts *.dtb -o *.dts``

-  dtb 转数组到数组 \*.c

   ``xxd -i *.dtb ./*.c``

三、 device tree 语法格式
-------------------------

-  \*.dts 文件开头必须采用如下所示开头
   ``/dts-v1/;     // version: 17     // last_comp_version: 16     // boot_cpuid_phys: 0x0``

-  16进制数组表示方法如下(注：字节间必须有一个空格，且暂时不支持换行)

   ::

       mac {
               sta_mac_addr = [C8 43 57 82 73 40];
               ap_mac_addr = [C8 43 57 82 73 02];
           };

-  字符串或者字符串数据表示方法如下

   ::

       model = "bl bl602 AVB board";
       compatible = "bl,bl602-sample", "bl,bl602-common";

-  32bit数据表示方法(可以使用16进制ox方式，也可使用10进制方式，字节间必须有一个空格，且暂时不支持换行)

   ::

       pwr_table = <0x4 0x64>
       pwr_table = <4 100>

四、 device tree 模块配置
-------------------------

串口配置
~~~~~~~~

::

    串口暂时仅支持配置以下功能

-  使能串口

   ``status = "okay";``

-  关闭串口

   关闭串口时，余下其他引脚、波特率等配置均无效，不会初始化相关硬件

   ::

       status = "disable";

-  配置引脚

   暂时不支持配置cts和rts相关功能，故暂时\ ``feature``\ 中rts和cts均为\ ``disable``\ 。如果使用了串的tx和rx，\ ``feature``\ 中到tx和rx需要配置为\ ``okay``\ ，\ ``pin``\ 中的tx和rx需要选择相关引脚。

   ::

       pin {
           rx = <7>;
           tx = <16>;
       };
       feature {
           rts = "disable";
           cts = "disable";
           rx = "okay";
           tx = "okay";
       };

-  配置波特率

   配置波特率参考如下，最大支持 ``2000000 bps``\ 这里以\ ``9600``\ 为例

   ::

       baudrate = <9600>;

-  配置id号

   配置id参考如下，这里以\ ``<0>``\ 为例

   ::

       id = <0>;

-  配置设备名

   目前串口设备名为/dev/ttyS\ *,至于*\ 为当前串口id号，目前调试口使用
   /dev/ttyS0

   ::

       pin {
               rx = <7>;
               tx = <16>;
           };
       feature {
           rts = "disable";
           cts = "disable";
           rx = "okay";
           tx = "okay";
       };
       path = "/dev/ttyS0";

