# Unity Hub Installation

Reference: [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector)

## 1. Add the public signing key
Run the following command to add the Unity public signing key:

```bash
wget -qO - https://hub.unity3d.com/linux/keys/public | gpg --dearmor | sudo tee /usr/share/keyrings/Unity_Technologies_ApS.gpg > /dev/null
```

## 2. Add the Unity Hub repository
Add the Unity Hub repository to `/etc/apt/sources.list.d`:

```bash
sudo sh -c 'echo "deb [signed-by=/usr/share/keyrings/Unity_Technologies_ApS.gpg] https://hub.unity3d.com/linux/repos/deb stable main" > /etc/apt/sources.list.d/unityhub.list'
```

## 3. Install Unity Hub
Update the package cache and install the package:

```bash
sudo apt update
sudo apt-get install unityhub
```

## 4. Additional Dependencies
Install `libc6-dev` and create a symlink for `libdl.so` if necessary:

```bash
sudo apt update
sudo apt install -y libc6-dev
sudo ln -s /lib/x86_64-linux-gnu/libdl.so.2 /lib/x86_64-linux-gnu/libdl.so
```
