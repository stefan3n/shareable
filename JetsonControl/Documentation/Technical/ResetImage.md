### [< back](Technical.md)

# **Jetpack - image reset guide**

**Note:** Use a **minimum 64GB SD card** !!! 32GB is not enough, and even 64GB cuts it close once you install everything.

Install **Balena Etcher** or **Imager**.

### **Jetson Orin Nano - main board:** 

1. Download JetPack 6.2.1 (latest version)

2. Flash it to the sd card using Balena Etcher or Imager


### **Jetson Nano - secondary board:**

1. Download [Ubuntu 20.04 Image](https://github-playground.int.automotive-wan.com/uig90963/smartCar2024/blob/main/jetsonPython/doc/Misc/Flashing.md) - This is not an official image provided by NVIDIA.
We also tested with JetPack 4.6.2 — the latest version supported by the Jetson Nano — but it comes with Ubuntu 18.04 and Python 3.6, which imposes significant limitations for our development.

2. Flash it to the sd card using Balena Etcher or Imager

3. Follow this [guide](https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html) after starting Ubuntu 20.04.



