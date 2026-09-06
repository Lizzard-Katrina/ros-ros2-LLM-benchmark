# 跑打分说明

打分脚本已在仓库内，clone 下来即可。

## 一次性准备

```bash
git clone https://github.com/Lizzard-Katrina/ros-ros2-LLM-benchmark.git
cd ros-ros2-LLM-benchmark

docker build -t groundtruth-base:humble \
  -f groundtruth_pipeline/docker/Dockerfile.base \
  groundtruth_pipeline/docker
```

首次构建要拉 osrf/ros:humble-desktop（数 GB），耗时较长。

## 跑一轮（把 resultsN 换成你负责的那轮）

```bash
export GROUNDTRUTH_BASE_IMAGE=groundtruth-base:humble

nohup python3 -u groundtruth_pipeline/score_benchmark.py \
  --results-run results3 --with-function-diagnostic \
  --out score_results3.json --resume > score_results3.log 2>&1 &
```

看进度：

```bash
tail -f score_results3.log
```

- 每个新终端都要重新 export，忘了会用错基础镜像
- 不需要 OPENROUTER_API_KEY，打分不调 LLM
- 中断了重跑同一条命令，--resume 会续
- 约 600 条，每条一次 docker build+run，耗时很长
- 跑完把 score_resultsN.json 提交回仓库

## 分工

- results2: 已完成
- results1:
- results3:
- results4:
- results5:

不带数字的 results/ 是废弃数据，不用管。

## 已知

7 个 task 会显示 NO_CANDIDATE_FILES，是候选生成阶段的已知限制，不是跑错了。
每轮实际评测规模 n = 101/108。
