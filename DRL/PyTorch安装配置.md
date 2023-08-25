## 一、Anaconda安装与配置

- Anaconda提供了conda机制用于管理Python数据分析时常用的工具库/包

- 官网下载最新的（翻墙）：https://repo.anaconda.com/archive/

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271544058.png" alt="image-20230627154419917" style="zoom: 25%;" />

  - 清华大学Anaconda镜像站：https://mirrors4.tuna.tsinghua.edu.cn/help/anaconda/
  - Anaconda3对应Python3，Anaconda2对应Python2

- 安装后配置系统环境变量

  - 此电脑 —— 属性 —— 高级系统设置 —— 环境变量

  - 在系统变量中选择“Path”，进行编辑，然后选择新建

    <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271625443.png" alt="image-20230627162539336" style="zoom: 25%;" />

  - 将以下内容加入，anaconda的安装目录是：D:\01_AppSpace\anaconda\install

    ```c
    D:\01_AppSpace\anaconda\install
    D:\01_AppSpace\anaconda\install\Scripts
    D:\01_AppSpace\anaconda\install\Library\mingw-w64\bin
    D:\01_AppSpace\anaconda\install\Library\usr\bin
    D:\01_AppSpace\anaconda\install\Library\bin
    ```

  - 点击确定退出即可

- 测试Anaconda是否安装成功。win + r，输入cmd进入终端

  - 输入python，确定有无python环境
  - 输入conda –version，确定是否有conda环境
    -  如果提示conda不是内部或外部命令，一般是Anaconda的环境变量设置问题

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271630014.png" alt="image-20230627163014963" style="zoom: 33%;" />



## 二、CUDA和cuDNN安装配置

- 安装这两个是为了使用上GPU
- CUDA是NVIDIA发明的一种并行计算平台和编程模型，它通过利用图形处理器GPU的处理能力，大幅提升计算性能；cuDNN则是GPU加速使用的深度神经网络原语库，cuDNN为标准例程提供了高度优化的实现，例如向前和向后卷积，池化、规范化和激活层等

---

### 2.1 CUDA安装配置

- 在NVIDIA面板中的系统信息查看电脑支持的cuda（CUDA 12.0.81）

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271641811.png" alt="image-20230627164111689" style="zoom: 25%;" />

- 下载CUDA toolkit：https://developer.nvidia.com/cuda-toolkit-archive

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271644511.png" alt="image-20230627164456436" style="zoom:25%;" />

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271645038.png" alt="image-20230627164555901" style="zoom:20%;" />

- 安装cuda，建议直接使用默认的安装路径，安装选择自定义

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271649911.png" alt="image-20230627164820971" style="zoom:40%;" />

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271651176.png" alt="image-20230627165111090" style="zoom:33%;" />

  - 第一次安装，尽量全选；第n次安装，尽量只选择第一个，不然会出现错误

  - 记住默认安装目录

    <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271652319.png" alt="image-20230627165245231" style="zoom:40%;" />

  - 安装完成，重启电脑

  - 配置cuda的系统环境变量

    - CUDA_PATH和CUDA_PATH_V12_0是自动生成的，如果没有则要手动添加

    <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271714894.png" alt="image-20230627171400807" style="zoom:33%;" />

  - 测试cuda是否安装成功

    - nvcc –V，查看cuda版本号
    - set cuda，查看cuda的环境变量

    <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271719636.png" alt="image-20230627171925598" style="zoom: 40%;" />

### 2.2 cuDNN配置

- 下载cuDNN：https://developer.nvidia.com/rdp/cudnn-download

  - 如果是第一次则需要先进行注册，已经有了账号则直接登录即可
  - 选择和CUDA适配的cuDNN版本

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271726984.png" alt="image-20230627172640849" style="zoom:20%;" />

- cuDNN下载下来是一个压缩包，对其解压，将其中三个文件复制到CUDA的安装目录中

  - CUDA的默认安装路径：C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.0
  - CUDA 的安装目录中，有和cuDNN解压缩后的同名文件夹，但直接复制即可。cuDNN解压缩后的同名文件夹中的配置文件会添加到CUDA安装目录中的同名文件夹中。这也表明cuDNN其实就是CUDA的一个补丁而已，专为深度学习运算进行优化的。

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271729966.png" alt="image-20230627172958949" style="zoom: 50%;" />

- 配置环境变量

  - 此电脑 —— 属性 —— 高级系统设置 —— 环境变量 —— Path —— 编辑 —— 新建

  - 加入以下内容：

    ```c++
    C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.0\bin
    C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.0\include
    C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.0\lib
    C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.0\libnvvp
    ```

    <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271740052.png" alt="image-20230627173951678" style="zoom:33%;" />

- 测试cuDNN是否配置成功

  - 主要使用CUDA内置的deviceQuery.exe 和 bandwithTest.exe

  - cd到安装目录：cd C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.0\extras\demo_suite

    - 执行：.\bandwidthTest.exe

    <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271744066.png" alt="image-20230627174438985" style="zoom:30%;" />

    - 执行：.\deviceQuery.exe

      <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271745492.png" alt="image-20230627174540389" style="zoom:25%;" />

## 三、PyTorch安装

- 打开Anaconda Prompt，创建一个虚拟空间用于pytorch的安装

  - 查看系统python版本
  - 创建虚拟空间：conda create -n 虚拟空间名称 python=系统python版本号

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271828520.png" alt="image-20230627182807418" style="zoom:25%;" />

  - 创建过程需要下载安装某些东西，直接输入y确认即可

- 激活环境：activate pytorch

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271829154.png" alt="image-20230627182921093" style="zoom:50%;" />

- pytorch官网：https://pytorch.org/get-started/locally/

  - 要注意和电脑的CUDA版本对应起来
  - 此次安装时CUDA装的12.0版本，在pytorch中没有对应的，但可以选择CUDA 11.8，能够兼容高版本CUDA（https://blog.csdn.net/AiTanXiing/article/details/129933911）

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271838569.png" alt="image-20230627183842481" style="zoom: 33%;" />

- 使用conda安装pytorch：conda install pytorch torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia

  - 这步需要翻墙才会比较快

- 测试pytorch是否安装成功

  ```python
  # CUDA TEST
  import torch
  x = torch.Tensor([1.0])
  xx = x.cuda()
  print(xx)
  
  # CUDNN TEST
  from torch.backends import cudnn
  print(cudnn.is_acceptable(xx))
  ```

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271853731.png" alt="image-20230627185350665" style="zoom:33%;" />

## 四、PyCharm安装配置

- pycharm下载安装：https://www.jetbrains.com/pycharm/download/#section=windows

  - 安装community版本即可

- 配置pycharm

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271859189.png" alt="image-20230627185922094" style="zoom: 25%;" />

  - 创建新项目

    - 设置项目位置

    - 选择previously configured interpreter，再选择Add Interpreter，点击Add Local Interpreter

      <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306280958227.png" alt="image-20230628095803138" style="zoom:50%;" />

    - 选择Conda Environment，选择环境

      - 进入anaconda安装目录，选择envs文件夹，选择建立的开发环境的python.exe

    ---

    **出现问题：**

    - **选择Conda Environment后，左下角出现conda executable is not found**

    解决方法：

    - 选择base环境下的conda.exe
      - 在anaconda安装目录的scripts下

    <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271910174.png" alt="image-20230627191036120" style="zoom: 33%;" />

    <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281000449.png" alt="image-20230627191242837" style="zoom: 33%;" />

点击create即可创建成功

测试：

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271915669.png" alt="image-20230627191541518" style="zoom:25%;" />

---

在pycharm中缺少某些python库？

- 在setting中找到project，选择python interpreter，先点击anaconda小图标，再点击 + 号即可下载安装新库

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271919186.png" alt="image-20230627191941059" style="zoom: 25%;" />

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306271923590.png" alt="image-20230627192335463" style="zoom:25%;" />







