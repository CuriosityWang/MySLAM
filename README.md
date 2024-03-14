# MySLAM
 Linux 下使用 C++ 开发的轻量级 SLAM 系统，包含前端，后端，回环检测模块，支持基本的定位，BA 优化，回环优化算法。
这个学期看完了高翔老师的《视觉SLAM十四讲》，学到了很多，首先是对计算机视觉的基本知识有了一个更加全面系统的理解，其次是动手去做实验的过程中，也更加理解了很多有关g2o,opencv,sophus等等工具的使用。

这个学期看完了高翔老师的《视觉SLAM十四讲》，学到了很多，首先是对计算机视觉的基本知识有了一个更加全面系统的理解，其次是动手去做实验的过程中，也更加理解了很多有关`g2o`,`opencv`,`sophus`等等工具的使用。

在第**13讲的实践部分**，高翔老师已经写好了一个基本SLAM框架的前端，后端部分，因此本篇博文主要记录第三个线程**回环检测**的实现。

先简单介绍一下三个线程的功能：

1. 前端（Frontend）线程： 前端线程负责处理相机图像流并提取特征点对应关系，然后使用这些对应关系来跟踪相机的运动。该线程负责实时地定位相机并估计其在三维空间中的轨迹。前端的主要任务包括：

   - 特征提取和匹配：从相机图像中提取特征点，通常使用角点检测器或特征描述子来找到关键点，并将它们与先前帧中的特征点进行匹配。

   - 运动估计：使用特征点的匹配信息来估计相机的运动，例如使用视觉里程计（Visual Odometry）技术来估计相机的位姿变化。

   - 姿态跟踪：将估计的相机运动与地图中的特征点进行匹配，从而跟踪相机的姿态并实时更新相机的位姿。

2. 后端（Backend）线程： 后端线程负责对前端估计的相机轨迹和地图进行优化，以获得更准确和一致的估计。该线程处理由前端产生的轨迹误差，并通过最小化重投影误差或其他优化准则来优化相机的位姿和地图点。后端的主要任务包括：

    - 图优化：建立一个图结构，其中相机位姿和地图点是图的节点，一般优化loss使用重投影误差是，然后通过图优化算法（例如g2o）来最小化误差并优化位姿和地图点。

3. **回环检测（Loop Closure）线程： 回环检测线程负责检测相机是否再次经过先前经过的地点。当检测到回环时，回环检测会触发回环优化过程，以修正先前轨迹的漂移和误差，从而提高整个系统的一致性和精度。回环检测的主要任务包括：**

    - **回环检测：使用特征点的描述子或其他特征匹配技术来检测先前访问过的地标或地图点。**

    - **回环优化：当检测到回环时，执行回环优化算法来修正先前轨迹的误差，以获得一致和精确的轨迹和地图。**
  


![image](https://github.com/CuriosityWang/MySLAM/assets/50990182/a999ca76-a59b-499f-9e72-d22b7c9421d1)

 
  更多内容可以查阅博客:https://www.cnblogs.com/curiositywang/p/17585372.html
