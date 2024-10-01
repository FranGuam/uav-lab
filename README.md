# uav-lab

Class homeworks & projects for Intelligent UAV Design and Implementation 2024 Fall

# 实用命令

### 修复Python3环境链接

```bash
ln -s /usr/bin/python3 /usr/bin/python
```

### 安装ROS依赖

```bash
rosdep install --from-paths <directory> --ignore-src
```

### 批量修改文件权限

```bash
find -name *.py -print -exec chmod +x {} \;
```
