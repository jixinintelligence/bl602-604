blog
====

概述
----

blog 是一款超轻量级的日志组件，非常适合对资源敏感的软件项目.

功能介绍
--------

blog
将log简化成3类，分别是组件、文件、私有log，且分别支持软件动态修改、宏彻底关闭方案（不占用rom）。下文统称动态修改为软开关，宏彻底关闭方案为静态开关。

-  组件log管理 该log优先级最高，如下的文件log和私有log均受此开关的束缚
-  文件log管理
   该log优先级中，如下的私有log均受此开关的束缚，且文件log受到组件log的管理。
-  私有log管理 该log优先级最低，受组件log和文件log管理

log等级说明
-----------

按照等级高低分别如下，其中all最低，即所有log均输出

::

    all : 所有log均输出，其实等同于all
    debug : debug及以上
    info : info及以上
    warn : warn及以上
    error : error及以上
    assert : assert及以上
    never: 所有log均不输出，其实等同于assert

使用方法
--------

包含必要的头文件
``#include <blog.h>``\ 然后分别设置组件log、文件log、私有log。

.. code:: c

    #include <blog.h>
    BLOG_DECLARE(blog_testc2);

    void func(void)
    {
        blog_debug("blog_testc2 debug\r\n");
        blog_info("blog_testc2 info\r\n");
        blog_warn("blog_testc2 warn\r\n");
        blog_error("blog_testc2 error\r\n");
        blog_assert("blog_testc2 assert\r\n");

        blog_debug_user(blog_testc2,"blog_testc2 debug user\r\n");
        blog_info_user(blog_testc2,"blog_testc2 info user\r\n");
        blog_warn_user(blog_testc2,"blog_testc2 warn user\r\n");
        blog_error_user(blog_testc2,"blog_testc2 error user\r\n");
        blog_assert_user(blog_testc2,"blog_testc2 assert user\r\n");
    }

-  组件log开关

   -  静态开关 在相应的 proj\_config.mk
      文件目录下，LOG\_ENABLED\_COMPONENTS配置上增加对应组件的名字
      例如这里需要增加blog\_testa blog\_testb
      blog\_testc组件静态开关，其他组件默认关闭
      ``LOG_ENABLED_COMPONENTS:=blog_testa blog_testb blog_testc``
   -  软件开关 通过输入如下命令来使能log输出等级 形如，logset level
      component\_name例如： ``blogset assert blog_testc``

-  文件log管理

   -  静态开关
      在对应的\*.c文件中，加入此行代码，注意，不管是开或者关，必须选择一种。
      ``默认就是开`` ``#define BLOG_HARD_DECLARE_DISABLE 1    // 关``
   -  软件开关 通过输入如下命令来使能log输出等级 形如，logset level
      component\_name.file\_name例如：
      ``blogset info blog_testc.blog_testc2_full``

-  私有log管理

   -  静态开关 使用就增加BLOG\_DECLARE(...)，不用直接不增加此行即可。
      ``BLOG_DECLARE(blog_testc2);    // 打开，其中　"blog_testc2"为用户自定义``
   -  软件开关 通过输入如下命令来使能log输出等级 形如，logset level
      component\_name.file\_name.pri\_name例如：
      ``blogset debug blog_testc.blog_testc2_full.blog_testc2``

总结
----

