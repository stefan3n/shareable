### [< back](SetupGuide.md)

# **GStreamer**

GStreamer with **hardware acceleration** is preferred over standard OpenCV for video capture on Jetson devices because:

* It uses hardware acceleration (GPU/ISP), **significantly reducing CPU load** and improving performance.

* The GStreamer **pipeline** optimizes image transfer and processing, resulting in lower latency and higher FPS.

* Standard OpenCV processes frames on the **CPU**, which can cause lag and poor performance on embedded devices.

You can check if your OpenCV build has GStreamer support with:

```
import cv2
print(cv2.getBuildInformation())
```

If GStreamer is not enabled, follow this **[guide to compile OpenCV with GStreamer support](https://galaktyk.medium.com/how-to-build-opencv-with-gstreamer-b11668fa09c)**.