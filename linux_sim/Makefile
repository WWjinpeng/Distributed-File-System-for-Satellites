# linux_sim/Makefile

CC = gcc
CFLAGS = -I../main -D__linux__  # 关键：定义 __linux__ 宏

SRCS = ../main/main.c ../main/dist_vfs.c
TARGET = sate_dfs_sim

all:
	$(CC) $(CFLAGS) $(SRCS) -o $(TARGET)
	mkdir -p sim_storage
	@echo "编译完成，运行 ./sate_dfs_sim 即可启动仿真"

clean:
	rm -f $(TARGET)
	rm -rf sim_storage