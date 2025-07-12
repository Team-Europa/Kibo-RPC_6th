# Europa @ 6th Kibo RPC -- Taiwanese Team

<p align="center">
  <img width="4167" height="1250" alt="Logo_With_Kibo-01" src="https://github.com/user-attachments/assets/a2fad2de-89e9-48a3-967e-34bbfd996d8f" />
</p>

We are team **Europa** came from Taiwan, one of participating team of **the 6th Kibo Robot Programming Challenge**.

Taiwan Prelimary Round team #: **K15**

## Technology

### Routine Strategy

To be finished.

### Object Detection for Board Scanning

#### The "Sub-Thread" Strategy

We execute a `Thread` object to run our vision model detecting separately from the main thread for:

- **Avoiding thread-blocking situations** caused by the high load model processing, which might cause the reduce of efficiency.
- Making sure **we have more than 1 image for object detection** to prevent potential miss-detecting by accepting error images

#### Object Detection Model Implement

Final solution for us is **YOLO v11 x with NCNN framework** introduced in [1], [Tencent/ncnn](https://github.com/Tencent/ncnn) C++ package and [nihui/ncnn_android_yolov8](https://github.com/Tencent/ncnn) YOLO implement in Android sample repository.

NCNN’s benefit:
- Optimized for smartphones – better for platform which have limited hardware like Astrobee.
- Faster than all the other known open-source frameworks. (README.md in NCNN’s repo)

Learned the implementation techniques from nihui’s repository:

- Use Java Native Interface (JNI) to interact NCNN’s C++ package.
- Convert our trained weight: `*.pt` → torchscript → PNNX (with modified script for dynamic input) → torchscript → NCNN

This technique worked smooth for YOLO v11 models, too.

### SIFT Estimation for Target Alignment

For target alignment, we used the **Scale-invariant Feature Transformation** (SIFT) technique introduced by [4, 5]:
1. Estimate the feature points of ArUco by background reference in assets
2. Translate to the center of the board
3. Project the reference coordinates of image to the world frame.

Prototype on Colab:

![image](https://github.com/user-attachments/assets/cca3f1ec-542d-4d4b-b626-16291eda4749)

## Simulating Experiment Result

To be experimented.

## References
[1] Jocher, G., Munawar, M. R., Derrenger, P., Yasin, M., Laughing, and Mattioli, F. Ultralytics YOLO11. _Ultralytics_. https://docs.ultralytics.com/models/yolo11/.

[2] Jocher, G., Munawar, M. R., Yasin, M., Laughing, LexBarou, Derrenger, P., MatthewNoyce, Ultralytics Assitant, Burhan-Q, Chaurasia, A., and Akyon, F. Explore Ultralytics YOLOv8. _Ultralytics_. https://docs.ultralytics.com/models/yolov8/.

[3] Huang, S., Lu, Z., Cun, X., Yu, Y., Zhou, X., and Shen, X. “DEIM: DETR with Improved Matching for Fast Convergence,” _2025 IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)_ (not yet published), 2025, pp. 15162-15171. 

[4] Lowe, D. G. “Distinctive Image Features from Scale-Invariant Keypoints,” _International Journal of Computer Vision_, Vol. 60, No. 2, 2004, pp. 91–110. https://doi.org/10.1023/b:visi.0000029664.99615.94.

[5] Lowe, D. G. “Object Recognition from Local Scale-Invariant Features,” _Proceedings of the Seventh IEEE International Conference on Computer Vision_, Institute of Electrical and Electronics Engineers, Kerkyra, Vol. 2, Sep. 1999, pp. 1150-1157.

## Statement of Project Originality and Use of Generative Artificial Intelligence Tools

In order to **uphold the fairness of international competitions** and to **comply with the principles of modern academic ethics**, the **Europa Team** (hereinafter referred to as “the Team”) hereby declares the following:

1. During the development period of the project for the 6th Kibo Robot Programming Challenge (hereinafter referred to as “Kibo-RPC”), **the Team did NOT directly use any sample code specifically prepared for this year’s competition that was NOT officially provided by the organizing body, the Japan Aerospace Exploration Agency (JAXA)**. All APK source code submitted by the Team was independently researched and written by its members, or derived and adapted from other sources through proper study for inclusion in the project.
2. During the development process, **the Team utilized ChatGPT and/or other related generative artificial intelligence language models** for purposes such as literature review, technical research, source code prototyping, debugging, logical structuring, and performance optimization. **The Team assumes full responsibility for the content of the submitted project.**
