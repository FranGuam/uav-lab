# uav-lab

Class homeworks & projects for Intelligent UAV Design and Implementation 2024 Fall

## 开发环境

### 使用DevContainer

根据操作系统，选择对应的`docker-compose.yml`文件填入`devcontainer.json`中，然后使用VSCode打开项目文件夹，点击`Reopen in Container`即可。

### 手动启动Docker容器

修改`docker-compose.{OS}.yml`文件中的`volumes`字段，添加条目将项目文件夹映射到容器中，然后执行以下命令启动容器：

```bash
docker compose -f docker-compose.{OS}.yml up -d
```

随后可以使用以下命令进入容器：

```bash
docker exec -it <container> bash
```

也可以使用VSCode的Attach to Running Container功能连接到容器。

## 实用命令

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

### [Linux] 允许GUI应用

```bash
xhost +
```

### 后台运行

```bash
nohup <command> &
```
