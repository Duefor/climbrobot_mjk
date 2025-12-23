# 遥操作 touch + cs66robot
## 25/11/11
小车用的机械臂sdk是老版sdk，但调试用的sdk是新版的，新旧sdk安装会覆盖对方的include文件夹中的hpp文件（/usr/local/include/Elite），但不会覆盖动态库（/usr/local/lib）
新旧sdk有些许不一样：名称；部分函数。需要改对应的cmakelist文件及其他
## 25/12/19
将该包同步到gitee上，在没有梯子的情况下可以通过gitee拉取代码