# git tips

好，我们来总结一下。我们想要进行精准搜索，无非就是增加筛选条件。
in:name xxx // 按照项目名搜索

in:readme xxx // 按照README搜索

in:description xxx // 按照description搜索


那么在这里面呢，我们又可以增加筛选条件
stars:>xxx // stars数大于xxx

forks:>3000 // forks数大于xxx

language:xxx // 编程语言是xxx

pushed:>YYYY-MM-DD // 最后更新时间大于YYYY-MM-DD

作者：Java技术栈
链接：https://zhuanlan.zhihu.com/p/112130321
来源：知乎
著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。


git diff提示filemode发生改变（old modee 100644、new mode 10075）
```bash
git config --add core.filemode false
```