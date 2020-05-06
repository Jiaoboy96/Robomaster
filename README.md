# 2019RM视觉相关算法

>## 前言
>1. 这是2019去总决赛的视觉算法，其中融合了装甲识别+能量机关的相关代码
>2. 为了调试方便，这里环境采用win10+vs2017+opencv3.4.2（版本对>应直接运行）
>3. 硬件： 免驱摄像头+PC
>4. 检测对象![](https://gitee.com/Mr-Shi-San/images/raw/master/img/20200421102130.png)![](https://gitee.com/Mr-Shi-San/images/raw/master/img/20200421102428.png)


| 作者   | 负责部分     | 联系方式          |
| ------ | ------------ | ----------------- |
| 张金峰 | 自瞄装甲识别 | 2607293749@qq.com |
| 焦晋文 | 自瞄装甲识别 | 1797254462@qq.com |
| 安岳凡 | 能量机关     |                   |


>## 文件结构和说明
>>1. 上场程序
>>> ```
>>> A:.
>>> │   FinalArrorDection.sln
>>> │   list.txt
>>> │   
>>> ├───FinalArrorDection
>>> │   │   ArrorAttitudeAlgorithm.cpp              目标测距 
>>> │   │   ArrorAttitudeAlgorithm.h
>>> │   │   ArrorRecognition.cpp                    装甲识别
>>> │   │   ArrorRecognition.h
>>> │   │   BigPinwheel.cpp                         能量机关相关代码
>>> │   │   BigPinwheel.h
>>> │   │   FinalArrorDection.vcxproj
>>> │   │   FinalArrorDection.vcxproj.filters
>>> │   │   FinalArrorDection.vcxproj.user
>>> │   │   Graph.cpp                               显示数据波形
>>> │   │   Graph.h
>>> │   │   main.cpp                                协调装甲识别+能量机关
>>> │   │   main.h
>>> │   │   README.md
>>> │   │   
>>> │   ├───sammple                                 程序运行的同时采集到的装甲板样本，方便机器学习
>>> │   │   ├───1
>>> │   │   ├───2
>>> │   │   ├───3
>>> │   │   ├───4
>>> │   │   └───5
>>> │   ├───video                       测试视频（比赛时+在家录不同分辨率工业相机，不同场景）
>>> │   │       bBigPinwheel0.avi
>>> │   │       bBigPinwheel1.mov
>>> │   │       bisai0(800).avi
>>> │   │       bisai1(728).avi
>>> │   │       rBigPinwheel0.avi
>>> │   │       rBigPinwheel1.mov
>>> │   │       video0(800).avi
>>> │   │       video1(800).avi
>>> │   │       video10(800)6500.avi
>>> │   │       video11(800)6000.avi
>>> │   │       
>>> │   ├───x64
>>> │   │   └───Debug
>>> │   │       │   
>>> │   │       └───FinalArr.1260F331.tlog
>>> │   │               
>>> │   └───xml                         调试参数的xml+机器学习>>> 的xml                         
>>> │           debugging.xml
>>> │           svm.xml
>>> │           
>>> └───x64
>>>     └───Debug>
>>>             FinalArrorDection.exe
>>> ```

>## 原理介绍与理论支持分析
>>### 自瞄
>>1. 算法流程![](https://gitee.com/Mr-Shi-San/images/raw/master/img/20200419224159.png)
>>2. 装甲识别（最终方案）
>>    1) 灯条检测
>>       - 灰度亮度检测：主要根据灯条的亮度和颜色的特点，先以灰度图>>检测出发光物体的位置（特点是能够将灯条形状完好的保留），再>>以彩色图R-G，G-R检测颜色（膨胀多一点，用来区分颜色），最>>后将两者二值化后的图进行判等，保留下来的即为（r/b）颜色和>>形状完好的灯条。
>>    2) 灯条筛选
>>       - 根据灯条的形状进行筛选（长宽比、角度等等。
>>    3) 拟合矩形
>>       - 根据装甲班两边灯条位置进行拟合（长宽比、角度等等）。 
>>    4) 条件限制、机器学习装甲贴纸判定
>>       - 经过上述形态学筛选，最后确定可能是装甲板的位置，将图像扣>>出，进一步用机器学习对该图像判断看是否位装甲班贴纸。 
>>    5) 找出装甲中心，进行小孔成像测距，测角度，最后将数据发送控制>>芯片
>>    6) 控制组开始进行角度拟合卡尔曼预测
>>3. 测距相关算法
>>   - PNP姿态解算（opencv官方库中方法）
>>     - 在识别到装甲板后就是对装甲板进行距离测量，方便对弹丸的下坠>>进行补偿。这里开始采用的pnp测距，该算法主要是通过将世界坐标>>系，像素平面坐标系，像平面坐标系以及相机坐标系进行相应的转>>换，包括![](https://gitee.com/Mr-Shi-San/images/raw/master/img/20200421104319.png)![](https://gitee.com/Mr-Shi-San/images/raw/master/img/20200421104351.png)![](https://gitee.com/Mr-Shi-San/images/raw/master/img/20200421104351.>>png)通过一系列的推导，就可以将二维坐标系与三维坐标系联系起>>来，从而得到目标物体的实际距离。
>>   -  小孔成像姿态解算（最终采用）
>>      -  ![](https://gitee.com/Mr-Shi-San/images/raw/master/img/20200421104531.png)在对装甲进行举例测量，>>他与图像上的二维坐标尺寸存在一定的比例关系（线性关系），通过>>成像尺寸和焦距得到距离，在装甲板的物理尺寸中，装甲的高是始终>>不变的，可以根据这个来计算，计算过程中需要对相机的感光芯片的>>尺寸进行匹配，换算成物理尺寸。以下是部分尺寸换算：
>>            ```
>>            Type Aspect Ratio Dia. (mm) Diagonal Width >>Height 
>>            1/3.6" 4:3 7.056 5.000 4.000 3.000 
>>            1/3.2" 4:3 7.938 5.680 4.536 3.416 
>>            1/3" 4:3 8.467 6.000 4.800 3.600 
>>            1/2.7" 4:3 9.407 6.592 5.270 3.960 
>>            1/2" 4:3 12.700 8.000 6.400 4.800 
>>            1/1.8" 4:3 14.111 8.933 7.176 5.319 
>>            2/3" 4:3 16.933 11.000 8.800 6.600 
>>            1" 4:3 25.400 16.000 12.800 9.600
>>            ```
>>   - 双目测距
>>     - 双目测距：双目测距主要是得到两张图像的视差。然后就可以根据>>摄像头的安装距离差得到实际距离。
>>4.  运行效果![](https://gitee.com/Mr-Shi-San/images/raw/master/img/20200421110901.png)