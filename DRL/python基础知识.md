Python笔记，参考链接：

https://www.liaoxuefeng.com/wiki/1016959663602400/1017063826246112

https://github.com/datawhalechina/learn-python-the-smart-way

---

## 一、Python基础

### 1.1 变量与注释

1. **常量**

   - 固定的量，值不可以改变，例如数字2

   - 包括：**数字、字符串、逻辑值**

   - 函数type(*)可查看*的类型

     <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281430909.png" alt="image-20230628143050863" style="zoom:33%;" />

2. **变量**

   - 可存储不同的值，表示不同的内容
   - 通过赋值符号 “ = ” 创建，例如var = 1
   - **标识符命名规范：**
     - **只能由字母、下划线或者数字组成**
     - **首字符必须是字母或者下划线**
     - 区分大小写

3. **注释**

   - 使用 **#** 表示注释，用于单行
   - 使用 **'''  '''** 或者 **''''''  ''''''** 表示区间注释

### 1.2 运算符

1. 算数运算符

   <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281435990.png" alt="image-20230628143516966" style="zoom: 50%;" />

2. 关系运算符和逻辑运算符

   <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281436290.png" alt="image-20230628143544135" style="zoom:50%;" />

3. 位运算符

   <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281436689.png" alt="image-20230628143609666" style="zoom: 50%;" />

4. 其他运算符

   - is，is not：对比两个变量的内存地址

   - ==，!=：对比两个变量的值

   <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281436378.png" alt="image-20230628143644358" style="zoom:50%;" />

### 1.3 数据类型与转换

1. 整型：**int**

2. 浮点型：**float**

3. 布尔型：**bool**

   - 只有True和False两个值

4. **类型转换：**

   - 转换为整型：int( )

   - 转换为字符串：str( )

   - 转换为浮点型：float( )

### 1.4 函数

#### 1.4.1 函数的定义

- 一种**可复用的部件**，用于定义更加复杂的操作以减少代码冗余
- 定义：
  - 关键字：**def**
  - 函数执行代码以冒号 **：**起始，并且缩进
  - **return** 函数结束函数，并选择性返回一个值给调用方；如果不带表达式则不返回值

```python
def function_name(parameters):
    ''函数-文档字符串''
    
    function_body

    return expression
```

- 实例

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281441078.png" alt="image-20230628144106056" style="zoom: 67%;" />

#### 1.4.2 函数参数

1. 形参与实参

   - **形参**：定义函数时的参数
   - **实参**：调用函数时输入的参数

2. **位置参数**

   ```python
   '''arg1为位置参数，在调用函数时位置必须固定''' 
   def function_name(arg1):
       
       return 
   ```

3. **默认参数**

   ```python
   '''arg2是默认参数，v是默认参数值。调用函数时，若无参数值传入则使用默认值'''
   '''默认参数必须放在位置参数后面''' 
   def function_name(arg1，arg2 = v):
       
       return   
   ```

4. **可变参数**

   ```python
   '''参数名：args'''
   '''传入的参数个数是可变化的，args是一个不定长的参数'''
   '''自动组装成...元组...'''
   '''加了星号（*）的变量名会存放所有未命名的变量参数'''
   def function_name(arg1，arg2 = v，*args):
       
       return 
   ```

5. **关键字参数**

   - 「可变参数」和「关键字参数」的同异总结如下：
     - **可变参数允许传入零个到任意个参数**，它们在**函数调用时自动组装为一个元组** 
     - **关键字参数允许传入零个到任意个参数**，它们**在函数内部自动组装为一个字典**

   ```python
   '''参数名：kw'''
   '''传入的参数个数是可变化的，kw是一个不定长的参数'''
   '''自动组装成...字典...'''
   def function_name(arg1，arg2 = v，*args，**kw):
       
       return 
   ```

6. **命名关键字参数**

   ```python
   '''参数名：nkw'''
   '''*,nkw --- 命名关键字参数，在nkw前加上分隔符*'''
   '''如果要限制关键字参数的名称，即可使用命名关键字参数'''
   def function_name(arg1，arg2 = v，*args，**kw):
       
       return 
   '''使用命名关键字参数时必须要有函数中指定的参数名''' 
   ```

7. **参数组合顺序**

   - 位置参数、默认参数、可变参数、关键字参数
   - 位置参数、默认参数、命名关键字参数、关键字参数

#### 1.4.3 全局与局部变量

- 局部变量：只在函数内部生效的变量，在函数外部无法使用
- 全局变量：在整个代码中都生效的变量，在函数内/外部都可使用

### 1.5 控制流

#### 1.5.1 while循环

- 基本结构：

  - 根据某一条件重复执行某个语句块
  - 判断条件后面加上冒号：

  ```python
  '''布尔表达式值为真则循环，假则停止'''
  while 布尔表达式：
  
      代码块 
  ```

- 实例

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281454325.png" alt="image-20230628145451301" style="zoom: 67%;" />

#### 1.5.2 for循环

- 迭代循环。**可根据某一序列进行循环迭代，直到迭代完整个序列**

  - 序列只是一个有序的项的集合，例如方括号括起来的一组常量或变量[0,1,2,3]

- 基本结构

  ```python
  for 迭代变量 in 可迭代对象：
  
      代码块
  ```

- 实例

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281456765.png" alt="image-20230628145609732" style="zoom: 67%;" />

#### 1.5.3 条件语句

- if语句

  ```python
  if expression:
      
      expr_true_suite
  ```

- if-else语句

  ```python
  if expression:
      expr_true_suite
  else:
      expr_false_suite
  ```

- if - elif - else语句

  ```c++
  if expression1:
      
      expr1_true_suite
      
  elif expression2:
      
      expr2_true_suite
      .
      .
  elif expressionN:
      
      exprN_true_suite
      
  else:
      
      expr_false_suite  
  ```

- 实例

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281459025.png" alt="image-20230628145910564" style="zoom: 67%;" />

#### 1.5.4 break语句

- 用于停止当前循环

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281500089.png" alt="image-20230628150009064" style="zoom: 67%;" />

#### 1.5.5 continue语句

- 用于停止当前循环，继续执行下一次循环

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281500385.png" alt="image-20230628150044362" style="zoom:67%;" />

#### 1.5.6 pass语句

- pass是**空语句，不做任何操作**，只起占位作用，目的是为了**保持程序结构的完整性**
- 如果在需要有语句的地方不写任何语句，那么解释器会提示错误，而pass可用于解决该问题

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281501360.png" alt="image-20230628150122331" style="zoom: 67%;" />

### 1.6 综合实例

![image-20230628150308512](https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281503538.png)

```python
# 定义阶乘函数，用于排列组合中
def multiple(x):
    result = 1
    while x != 0:
        result = result * x
        x = x - 1
    return result

# 定义二项分布计算函数
def p_xk(k):
    # 计算排列组合
    temp = multiple(20) / (multiple(k) * multiple(20 - k))
    # 计算概率
    p = (0.2 ** k) * (0.8 ** (20 - k))
    return temp * p

k = 0
while k != 21:
    print('P{ X =', k, ' } = ', p_xk(k))
    k = k + 1
```

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281509641.png" alt="image-20230628150923594" style="zoom:50%;" />

## 二、Python数据结构

### 2.1 序列

#### 2.1.1 序列定义

- python支持一种数据结构的基本概念，**容器（container）**
  - 可包含其他对象的对象
  - **两种主要的容器是序列（如列表和元组）和映射（如字典）**，在序列中每个元素都有编号，在映射中每个元素都有名称（也称为键）
  - **集合（set）**是一种既不是序列也不是映射的容器
- 序列中每个元素都有编号，即其位置或索引，**第一个元素的索引为0，用负索引表示末尾元素位置**
- 最常用的序列：**列表**和**元组**
  - **列表可以修改，元组不可以修改**

#### 2.1.2 常用序列操作

1. **索引**

   - 元素的编号，通过索引可获取元素
   - 首元素索引为0，-1可表示最后一个元素索引

   <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281531636.png" alt="image-20230628153152610" style="zoom:50%;" />

2. **切片**

   - 用于**访问特定范围内的元素**
   - 提供**两个索引指定切片的边界，中间用冒号隔开**
     - **第一个索引是包含的第一个元素的编号**
     - **第二个索引是切片后剩下的第一个元素的编号**

   <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281532882.png" alt="image-20230628153252860" style="zoom:50%;" />

   - 简写：

     - 切片开始于序列开头，可省略第一个索引

       <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281534154.png" alt="image-20230628153459132" style="zoom:67%;" />

     - 切片结束于序列末尾，可省略第二个索引

       <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281535176.png" alt="image-20230628153522154" style="zoom:67%;" />

     - 复制整个序列，两个索引都可省略

       <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281535162.png" alt="image-20230628153538139" style="zoom: 67%;" />

   - 设置步长

     - 步长默认为1，可以设为负数（第一个索引必须大于第二个），即从右向左提取元素
     - 步长不可以设置为0，否则无法向前移动
     - 显示指定某一步长，提取元素时将跳跃步长值个数个元素
     - **步长在末尾设置**
       - 2是步长

     <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281534738.png" alt="image-20230628153437713" style="zoom:67%;" />

3. **相加**

   - 通过加法拼接两个序列
   - 不同类型的序列不可以相加

   <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281536145.png" alt="image-20230628153633121" style="zoom:67%;" />

4. **相乘**

   - 序列与数a相乘，将重复这个序列a次来创建一个新序列

     <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281537597.png" alt="image-20230628153749575" style="zoom:67%;" />

   - 空列表：使用不包含任何内容的两个方括号（[ ]）表示

   - None表示什么都没有

     <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281538894.png" alt="image-20230628153813874" style="zoom:67%;" />

5. **成员资格检查**

   - 检查特定的值是否包含在序列中，可使用运算符 **in**
   - 检查是否满足指定的条件，并返回相应的值：**满足时返回True，否则返回False**

   <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281538950.png" alt="image-20230628153858926" style="zoom:67%;" />

### 2.2 列表

#### 2.2.1 函数list

- 有序序列，可以修改
- 可以使用任何序列作为list的参数

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281540687.png" alt="image-20230628154000665" style="zoom:67%;" />

#### 2.2.2 基本列表操作 — 增加

1. **append**：用于将一个对象附加到列表末尾

   ![image-20230628154152488](https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281541513.png)

   ![image-20230628154210232](https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281542253.png)

2. **extend**：能同时将多个值附加到列表末尾

   ![image-20230628154237240](https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202306281542263.png)

3. **insert**：将一个对象插入列表

   ![image-20230628154259078](C:\Users\pjw\AppData\Roaming\Typora\typora-user-images\image-20230628154259078.png)

4. **切片插入**：

   ![image-20230628154407491](C:\Users\pjw\AppData\Roaming\Typora\typora-user-images\image-20230628154407491.png)

#### 2.2.3 基本列表操作 — 删除

1. **del**：删除元素
2. **clear**：清空列表
3. **remove**：用于删除第一个指定值的元素
4. **pop**：从列表删除一个元素（默认最后一个），并返回这一元素
   - pop是唯一既修改列表又返回一个非None值的列表方法

#### 2.2.4 基本列表操作 — 查找

- **index**：在列表中查找指定值第一次出现的索引

![image-20230628154746927](C:\Users\pjw\AppData\Roaming\Typora\typora-user-images\image-20230628154746927.png)

#### 2.2.5 基本列表操作 — 修改

- 直接指定赋值