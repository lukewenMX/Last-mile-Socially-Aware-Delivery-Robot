
# Last-mile-Socially-Aware-Delivery-Robot
<figure style="text-align:center;">
  <img src="imgs/ICRA24_MS3225_Accompany_video%2000_00_07-00_00_14.gif" alt="Image Description" width="800">
  <figcaption>Three main challenges in last-mile delivery robot navigation</figcaption>
</figure>

## 1. Introduction
<figure style="text-align:center;">
  <img src="imgs/framework.png" alt="Image Description" width="800">
  <figcaption>Framework</figcaption>
</figure>

**Safe and efficient navigation of delivery robots** in pedestrian-crowded environments is challenging and demanding since it requires robots to analyze where the traversable area is, understand the social intention of pedestrians, and plan motion with social awareness. To address this, we proposed a multi-modal socially aware navigation framework for last-mile delivery robots on pedestrian-crowded sidewalks. This framework comprises three key aspects: 
1) We introduced the **Nanyang Sidewalk dataset**, designed explicitly for class segmentation tasks on sidewalks. 
2) A multi-modal 3D detection and motion prediction integrated with the **social force model** has been introduced to perceive the intention of pedestrians. 
3) A socially aware motion planner for the delivery robot is demonstrated by following pedestrian etiquette. Extensive experiments have been conducted to verify and evaluate the performance of the proposed algorithm.

### 1.1 Nanyang Sidewalk dataset
#### **Download Nanyang Sidewalk Dataset** [here](https://entuedu-my.sharepoint.com/:f:/g/personal/yichen_zhou_staff_main_ntu_edu_sg/EiWSMUAhh6dIukF7rnQqdygBmHAQB__wPmX1BCeHz69IAA?e=60Lchq)
<figure style="text-align:center;">
  <img src="imgs/ICRA24_MS3225_Accompany_video%2000_00_25-00_00_30.gif" alt="Image Description" width="500">
</figure>

The dataset comes from a 1,150-second video recorded by a husky robot with a realsense RGB-D camera. The robot traveled about 1 km along the Nanyang Link sidewalk. We took one image every 11 frames, resulting in an initial collection of 3,000 images. We then removed similar images to increase diversity, resulting in a final dataset of **1,096** images, each with a resolution of 480x848 pixels. The dataset in **Pascal VOC 2012** format can be downloaded [here](https://entuedu-my.sharepoint.com/:f:/g/personal/yichen_zhou_staff_main_ntu_edu_sg/EiWSMUAhh6dIukF7rnQqdygBmHAQB__wPmX1BCeHz69IAA?e=60Lchq).

<figure style="text-align:center;">
  <img src="imgs/dataset.png" alt="Image Description" width="800">
  <figcaption>Visualization of segmentation results (Mask2Former)</figcaption>
</figure>

<figure style="text-align:center;">
  <img src="imgs/Weixin%20Screenshot_20231008205332.png" alt="Image Description" width="800">
  <figcaption>Performance Comparsion</figcaption>
</figure>

### 1.2. Pedestrian Intention Prediction

#### a. Pedsim Scenarios
<figure style="text-align:center;">
  <img src="imgs/ICRA24_MS3225_Accompany_video%2000_01_44-00_01_50.gif" alt="Image Description" width="800">
</figure>

#### b. Repulsive-Boundary-based Trajectory prediction

<figure style="text-align:center;">
  <img src="imgs/ICRA24_MS3225_Accompany_video%2000_01_44-00_01_50~1.gif" alt="Image Description" width="800">
</figure>

### 1.3. Socially Aware Motion Planner
#### a. Overtaking from the right side
<figure style="text-align:center;">
  <img src="imgs/ICRA24_MS3225_Accompany_video%2000_01_44-00_01_50~2.gif" alt="Image Description" width="800">
</figure>

#### b. Running along from the left side
<figure style="text-align:center;">
  <img src="imgs/ICRA24_MS3225_Accompany_video%2000_01_44-00_01_50~4.gif" alt="Image Description" width="800">
</figure>

## acknowledgements
We thank the authors of [Pedsim](https://github.com/srl-freiburg/pedsim_ros) for providing the pedestrian simulation environment. We also thank the authors of [MMSegmentation](https://github.com/open-mmlab/mmsegmentation) for providing the semantic segmentation framework. We also thank the authors of [Agentformer](https://ye-yuan.com/agentformer/) for providing the pedestrian trajectory prediction framework.
