zotero管理文献，坚果云同步论文pdf文件

---

**Step 1：**

- 下载安装zotero：https://www.zotero.org/

  - 需要翻墙
  - 注册一个账号

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202307081057735.png" alt="image-20230708105734552" style="zoom: 20%;" />

  - zotero要存储的是“数据”和“附件”两部分内容
    - 数据可以理解为一个关于条目及条目关系的数据库，它不占zotero的免费300兆存储空间
    - 附件就是论文的pdf，这部分体积很大，zotero的默认设置是将其同步到云端，300兆空间很容易被占满

- 安装好后在客户端登录zotero账号

  - 同步分为数据同步和文件同步两部分
  - 数据同步不占用云端空间，要保留；文件同步要取消掉

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202307081100298.png" alt="image-20230708110021229" style="zoom:25%;" />

**Step 2：**

- 下载安装插件zotfile（专门用来管理附件的）：https://github.com/jlegewie/zotfile/releases/

  - 下载后放到zotero安装目录下的extensions目录下

    <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202307081101763.png" alt="image-20230708110146731" style="zoom:25%;" />

  - 客户端的工具选项中点击附加组件添加上即可

- 设置附件的保存位置

  - 工具 —— zotfile preference —— general settings

    - 第一个选择论文下载位置（不是条目链接的pdf所在文件夹）
    - 第二个选择条目链接的pdf所在文件夹（要同步到云端的文件夹）
    - zotero进行链接的过程，可以理解为将原始文件夹（文件夹1）里的pdf复制到真正的pdf文件夹（文件夹2）中，将其链接到特定条目

    <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202307081105290.png" alt="image-20230708110508208" style="zoom:25%;" />

**Step 3：**

- 设置根目录和数据存储位置：编辑——首选项——高级——文件和文件夹

  - 根目录也放在文件夹2中，和pdf一起同步
  - 数据存储选择一个新建的目录

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202307081110602.png" alt="image-20230708111051562" style="zoom:33%;" />

- 将zotero_paper文件同步到坚果云即可





