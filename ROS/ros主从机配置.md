两台设备，一个台式机（hostname是peng），另一个是虚拟机（hostname是jianwei），将台式机设成主机，虚拟机设成从机

### 1. 先确保两台设备都在同一个局域网

- 使用`ifconfig`查看两台设备在局域网下的IP地址

台式机：

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202208141704548.png" alt="image-20220814170426486" style="zoom: 67%;" />

虚拟机：

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202208141648927.png" alt="image-20220814164800869" style="zoom:50%;" />

两台设备都在192.168.3段

### 2. 配置hosts文件

- 终端使用`hostname`可以查看设备的计算机名称

- 分别在两台设备中打开hosts文件，加入对方的IP地址和对应的计算机名称

	`sudo gedit /etc/hosts`

台式机的IP地址是：192.168.3.228，计算机名称是：peng，那么在虚拟机的hosts文件中添加如下内容：

```
192.168.3.228	peng
```

虚拟机的IP地址是：192.168.3.149，计算机名称是：jianwei，那么在台式机的hosts文件中添加如下内容：

```
192.168.3.149	jianwei
```

- 配置好之后可以相互ping一下，测试网络是否联通

在台式机上：

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202208141702417.png" alt="image-20220814170224375" style="zoom:50%;" />

在虚拟机上：

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202208141703937.png" alt="image-20220814170337911" style="zoom:50%;" />

从终端消息可以看出能相互ping，即hosts配置正确

### 3. 主从机IP配置

从机（虚拟机）要知道Master的位置（即主机的位置），那么就需要在从机中配置主机的IP，告诉从机主机在什么地方

在从机的~/.bashrc文件中设置主机IP

```
export ROS_MASTER_URI=http://peng:11311
```

保存退出后，终端执行：`source ~/.bashrc`

也可以直接在终端使用指令设置主机IP

```
echo "export ROS_MASTER_URI=http://peng:11311" >> /.bashrc
source ~/.bashrc
```

**注意：修改了bashrc文件后一定要记得source一下**

### 4. 主从机测试

在主机（台式机）上开启roscore，同时运行小乌龟`rosrun turtlesim turtlesim_node`

然后在从机（虚拟机）上可以查看topic

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202208141728360.png" alt="image-20220814172816317" style="zoom:50%;" />

也可以在从机上运行键盘控制节点，实现在从机上控制主机上的小乌龟，`rosrun turtlesim turtle_teleop_key`

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202208141729171.png" alt="image-20220814172923142" style="zoom:50%;" />

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202208141746502.png" alt="image-20220814174604121" style="zoom: 33%;" />

