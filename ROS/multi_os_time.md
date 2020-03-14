# ros time

设定时间
首先呢，我们需要设置我们的时间，一般是不用的，但是双系统的时间经常出错，所以还是说一下吧

ERROR: `15 Dec 17:20:29 ntpdate[26610]: no server suitable for synchronization found`

```bash
sudo apt-get install ntp

sudo ntpdate ntp.ubuntu.com
[sudo] password for cong: 
 1 Mar 22:25:43 ntpdate[2182]: the NTP socket is in use, exiting

```

网络时间设置
```bash
$ sudo apt update
$ sudo apt install chrony ntpdate
$ sudo ntpdate ntp.ubuntu.com
```
手动时间设置
```bash
$ sudo date -s "16:47 1/23/2017"
```
————————————————
版权声明：本文为CSDN博主「张宙辕」的原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/zhangzhouyuan6514/article/details/79019966

## hosts

in /etc/hosts, add

```
192.168.1.11 cpr-a200-0375
```

ERROR: Couldn't find an AF_INET address for *****

ERROR:
```bash
[ INFO] [1576448995.618869857]: Didn't received robot state (joint angles) with recent timestamp within 1 seconds.
Check clock synchronization if your are running ROS across multiple machines!
[ERROR] [1576448995.618984589]: Failed to fetch current robot state
```
Solution: in husky pc
```
sudo ntpdate 192.168.1.105
```