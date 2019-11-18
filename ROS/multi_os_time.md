# ros time

设定时间
首先呢，我们需要设置我们的时间，一般是不用的，但是双系统的时间经常出错，所以还是说一下吧

网络时间设置

$ sudo apt update
$ sudo apt install chrony ntpdate
$ sudo ntpdate ntp.ubuntu.com
手动时间设置

$ sudo date -s "16:47 1/23/2017"

————————————————
版权声明：本文为CSDN博主「张宙辕」的原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/zhangzhouyuan6514/article/details/79019966
