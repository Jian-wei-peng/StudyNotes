## 一、面向对象编程

### 1.1 面向过程与面向对象

- 区别：
  - 面向过程的核心是一系列函数，执行过程是依次使用每个函数
  - 面向对象的核心是**对象（类）**及其**属性、方法**，每个对象根据需求执行自己的方法以解决问题

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202304251558434.png" alt="image-20230425155810396" style="zoom:33%;" />

- **对象**：单个事务的抽象。也可理解为**函数**和**数据**的**封装**

- 面向对象的必要性：

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202304251559978.png" alt="image-20230425155939919" style="zoom:40%;" />

### 1.2 对象的基本概念

1. **类**：一组相关的**属性**和**行为**的集合，是一个抽象的概念
   - **抽象化**：抽象代表现实世界中实体的行为
   - **属性**：存储在类中的**数据**和**变量**
   - **方法**：事物可以执行的操作或行为
   - **初始化**：构造一个类并指明固有属性的过程
2. **对象**：该类事物的具体表现形式，具体存在的个体

### 1.3 面向对象的三个特征

- **封装**
  - 一个对象就是一个封装了**数据**和**操作这些数据的代码**的实体
  - 部分数据和方法只能通过内部访问，避免意外的修改
- **继承**
  - 可以复用现有类的所有功能，并在无需重新编写原来的类的情况下对这些功能进行拓展
  - 提供了从特殊到一般的方法
- **多态**
  - 一个类实例的相同方法在不同情形有不同表现形式
  - 不同的对象可以通过相同的操作来调用

### 1.4 MATLAB面向对象编程的用法

新建文件的时候直接新建一个类文件，会自动生成一个标准类模板：

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202308221146481.png" alt="image-20230822114650351" style="zoom:25%;" />

```matlab
classdef untitled
    %UNTITLED2 此处提供此类的摘要
    %   此处提供详细说明

    properties
        Property1
    end

    methods
        function obj = untitled(inputArg1, inputArg2)
            % 构造函数本身
            obj.Property1 = inputArg1 + inputArg2;
        end

        function outputArg = method1(obj, inputArg)
            % 方法
            outputArg = obj.Property1 + inputArg;
        end
    end
end
```

注意：

- 类名称要和文件名保持一致
- **properties**关键字：类属性
- **methods**关键字：方法
  - **在methods里必须存在一个与类名相同的函数**，这个函数的输出变量就是类产生的对象

### 1.5 实例

定义一只小狗；属性：名字、年龄；初始化：输入名字；方法1：让它坐下；方法2：询问年龄

```matlab
classdef dog
    properties
        % 小狗的属性
        name
        age
    end 
    
    methods
        function obj = dog(name, age)
            % 构造函数
            obj.name = name;
            obj.age = age;
        end
        
        function sit(obj)
            % 让小狗坐下
            fprintf('%s is now sitting.\n', obj.name);
        end
    end
end
```

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202308221448943.png" alt="image-20230822144800834" style="zoom:33%;" />

## 二、面向对象的基本操作

### 2.1 类的属性和方法

1. 属性

   - 属性的默认值 —— 在属性列表中指定
   - **常量属性**：用**Constant**声明
   - **非独立属性**：用**Dependent**声明，属性值是否取决于其他值
   - **隐藏的属性**：通过**Hidden**声明，**用于隐藏内部细节**，使对象更简洁
2. 属性的访问权限

   - 语法：（Set / Get）Access = private / protected / public
   - **private**：只有该类的成员方法可以访问
   - **protected**：只有该类的成员方法和该类的子类可以访问
   - **public**：除该类和子类外，在类之外的函数或脚本中也可以访问
3. 方法
   - **构造函数Constructor**
     - 构造函数和类的名称相同，用来创建类的实例
     - 一个类的定义中只能有一个构造函数
     - 构造函数只能有一个返回值，即新创建的对象

### 2.2 类的组合和继承

- **继承是为了提高代码复用性**，但也要注重逻辑性

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/image-20230426112910880.png" alt="image-20230426112910880" style="zoom:50%;" />

- 实例：有一个车辆类vehicle，现在要创建一个电动车类e_vehicle

  - vehicle.m

    ```matlab
    classdef vehicle < handle  
        properties
            make
        end
        properties(SetAccess=protected)
            car_miles = 0;
        end
        properties(Hidden)
            year
        end
        properties(Dependent)
            car_km
        end
        
        methods
            function obj = vehicle(year, make)
                obj.make = make;
                obj.year = year;
            end
            function descripe(obj)
                fprintf('%s made in %d.\n', obj.make, obj.year);
            end
            function run(obj, miles)
                obj.car_miles = obj.car_miles + miles;
            end
        end
    end
    ```

  - e_vehicle.m

    ```matlab
    classdef e_vehicle < vehicle
        properties
            soc
        end
        properties(Constant)
            soc_per_mile = 1;   % 常量属性
        end
        
        methods
            function obj = e_vehicle(soc, year, make)
                obj = obj@vehicle(year, make); % 用@调用父类方法
                obj.soc = soc;
            end
            function run(obj, miles)
                run@vehicle(obj, miles);
                obj.soc = obj.soc - miles*obj.soc_per_mile;
            end
        end
    end
    ```

  - 实例化一个e_vehicle对象ev

    ```matlab
    ev=e_vehicle(95,2020,'yiqi')
    ```

    <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202308221512339.png" alt="image-20230822151246300" style="zoom: 50%;" />

  - 假设跑了10英里（调用ev的方法run），查看ev的里程

    ```matlab
    ev.run(10)
    ev.car_miles
    ```

    <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202308221514847.png" alt="image-20230822151443823" style="zoom:67%;" />

  - 跑了10英里后，电量减少了，ev.soc查看从95变成了85

### 2.3 句柄类和实体类

- **handle类**是遵循句柄语义的所有类的超类
  
  - 句柄是引用handle类的对象的变量
  - 多个变量可以引用同一个对象
  
- **handle类是抽象类，无法直接创建该类的实例**。可使用handle类派生其他类，可以使其实例为句柄对象的具体类

  ```matlab
  classdef MyHandleClass < handle		% 表示MyHandleClass类继承于handle类
   	...
  end
  ```

- 值类（实体类，Value）：

  - 构造函数返回一个与其赋值变量相关联的对象
  - 复制行为会创建一个独立的副本
  - 值类对象在方法中修改后需要返回对象本身

- 句柄类：

  - 构造函数返回一个句柄对象，该对象是对所创建对象的引用
  - 复制的句柄指向同一块内存，不会创建副本
  - 函数对作为输入参数传递的句柄对象进行修改后，不必返回该对象

例子：

值类dog_value.m

```matlab
classdef dog_value
    properties
        % 小狗的属性
        name
        age
    end 
    
    methods
        function obj = dog_value(name, age)
            % 构造函数
            obj.name = name;
            obj.age = age;
        end
        
        function sit(obj)
            % 让小狗坐下
            fprintf('%s is now sitting.\n', obj.name);
        end
    end
end
```

句柄类dog_handle.m（代码与值类一样，名称不同而已，定义时加上handle）

```matlab
classdef dog_handle < handle
    properties
        % 小狗的属性
        name
        age
    end 
    
    methods
        function obj = dog_handle(name, age)
            % 构造函数
            obj.name = name;
            obj.age = age;
        end
        
        function sit(obj)
            % 让小狗坐下
            fprintf('%s is now sitting.\n', obj.name);
        end
    end
end
```

- 分别创建两种类的实例

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202308221529439.png" alt="image-20230822152913396" style="zoom:50%;" />

- 复制值类dv，会创建一个独立的副本，即dv_copy与dv是相互独立的，互不影响

  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202308221533905.png" alt="image-20230822153300867" style="zoom:50%;" />

  - 将dv的age修改成10，但是dv_copy的age没改变

- 复制句柄类dh，句柄指向同一块内存，不会创建副本，即修改其中一个的属性就会影响到另外一个

  - 修改dh的年龄，dh_copy的年龄也被修改了
  
  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202308221535601.png" alt="image-20230822153530554" style="zoom:50%;" />

- 值类对象在方法中修改后需要返回对象本身

  - 在值类dog_value.m中添加一个方法set_age用于修改年龄

    - 如下图这样添加是无法修改的，因为值类对象在方法中修改后需要返回对象本身

      ```matlab
      function set_age(obj, age)
      	% 修改年龄
      	obj.age = age;
      end
      ```

    <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202308221601858.png" alt="image-20230822160127744" style="zoom: 33%;" />

    - 正确的是：

      - 调用的时候如果只是执行dv.set_age这个方法，dv的age并不会被修改，因为没有返回对象本身
  - 正确的是`dv = dv.set_age(2)`，这样才能修改dv的age
    
      ```matlab
      function obj = set_age(obj, age) % 修改后需要返回对象本身
      	% 修改年龄
      	obj.age = age;
      end
      ```
  
  <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202308221605885.png" alt="image-20230822160524761" style="zoom:33%;" />
  
- 相比之下，句柄类就会更简单一些

  - 直接添加如下方法，不需要返回对象本身

    ```matlab
    function set_age(obj, age)
    	% 修改年龄
    	obj.age = age;
    end
    ```

    <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202308221610388.png" alt="image-20230822161018269" style="zoom:33%;" />

  - 调用方法set_age后，dh对象的age即被修改

- **判断对象是否相等**
  - 对于handle类
    - 可以用**==**和**isequal**判断是否指向同一块内存（句柄复制）
    - 可以用**isequal**判断指向不同内存的对象是否属性一致
  - 对于value类
    - 不支持使用==判断相等关系，但可以进行运算符重载
    - 可以用isequal判断相等关系









