.. _blsync-ble-index:

BLSYNC-BLE
==================

总览
------

本示例主要介绍如何使用ble进行wifi配网。

使用步骤
-----------

- 编译 ``customer_app/sdk_app_ble_sync`` 工程并下载工程固件；
- 固件上电运行会自动开启ble广播，等待手机APP连接配网，如下所示；

    .. figure:: imgs/image1.png
       :alt: 

- 打开手机APP搜索蓝牙设备，搜索到设备名“BL602-BLE-DEV”；

    .. figure:: imgs/image2.png
       :alt: 

- 点击连接设备后，点击APP中的扫描，等待数秒后APP会显示开发板扫描到的wifi设备列表；

    .. figure:: imgs/image3.png
       :alt: 

    .. figure:: imgs/image4.png
       :alt: 

- 用户可以通过扫描出来的设备列表对进行需要配网的wifi进行连接；

    .. figure:: imgs/image5.png
       :alt: 

- 当用户确定配网完成时，不需要再使用配网功能，可以使用“blsync_ble_stop”命令将其关闭。

    .. figure:: imgs/image6.png
       :alt: 