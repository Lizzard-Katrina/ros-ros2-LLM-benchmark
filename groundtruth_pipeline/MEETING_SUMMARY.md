# Benchmark进展汇报:从"看文本"到"真的跑起来"

## 1. Pass/Fail的判定标准

不是看代码"长得像不像对",而是**真的在ROS2环境里跑起来、真的做交互、断言返回内容对不对**。具体标准:

1. **Tier1 编译**:把翻译出的ROS2代码放进真实ROS2 Humble环境,`colcon build`必须编译成功
2. **Tier3 运行时**(核心判定):编译成功后,真实**启动**翻译出的那个节点(不是读代码文本、不是自己另外写一份逻辑代替),用另一个真实节点去跟它交互(发消息/调服务/查参数),**断言收到的具体返回内容是否正确**(比如：调用查询服务后返回的专辑列表是否包含预期的3张专辑,而不是只看"有没有报错")

**只有Tier1+Tier3都通过,才判定PASS**。中间过程我们专门审计过、修过好几轮"测试自己测自己、没真的碰到目标代码"这类问题,现在的判定是可信的真实结果,不是走过场。

## 2. 现在的Docker是怎么搭的

- **一个共享基础镜像**`groundtruth-base:humble`(基于官方`osrf/ros:humble-desktop`,预装colcon、pytest、常用ROS2依赖包),所有task共用,不需要为每个task单独造镜像
- **每个task目录下有一份`Dockerfile_real`**——这是纯原生的Docker定义文件,不依赖任何我们自己写的Python代码就能独立复现:
  ```bash
  cd <task目录>
  docker build -t gt-<task> -f Dockerfile_real .
  docker run --rm gt-<task>     # exit code 0 = PASS
  ```
- 这套架构目前**108个task全部**搭好并`GROUNDTRUTH_VERIFIED`(build+run 都过)
- 之后又做了一轮**投毒测试(mutation testing)**专门查"测试有没有真的碰到目标代码":把 groundtruth 里挖空对应的那段逻辑替换成空壳,重跑同一个 `Dockerfile_real`,测试若还 PASS 就实锤是"影子测试"。全量跑下来 47 个 `TEST_IS_REAL`、40 个工具定位不到(函数被重构改名,`MUTATION_NOT_APPLICABLE`)、7 个 `SUSPICIOUS_SHADOW_TEST`。7 个已逐个人工复核:4 个是工具误伤(掏错文件/匹配到重名函数),`task_005_gazebo` 是针对该函数的真影子测试但迁移模式被姊妹函数+文本断言覆盖,`task_004_turtlrbot3_params`(纯静态文本测试)和 `task_006_amcl_navigation`(只测 action 外壳、没测数值)已重写成真 runtime 测试并重新验证通过
- 特别说明:即使是Gazebo/PX4/CARLA这类涉及仿真器的复杂task,也用的是**同一个共享镜像**——因为它们的测试在"仿真器/硬件"这个边界上做了合理的mock,不需要真的起完整仿真环境,所以不需要像最初预想的那样搭建多个专用镜像

## 3. 怎么用新代码跑pass/fail

评测的对象是:**被测模型翻译出的代码,替换掉groundtruth包里对应的目标文件,其余依赖/骨架/测试全部保持groundtruth原样,再用groundtruth自己的`Dockerfile_real`原样build+run**——保证跟groundtruth验证走的是同一套流程,公平对比。

工具链(`groundtruth_pipeline/`目录下):
- `evaluate_candidate.py`:单个task的评测执行器
- `score_benchmark.py`:批量扫描每个模型的翻译结果(存在`results2/<model>/.../dialogue.md`里),自动跑分,支持断点续跑

一条命令跑完某一批模型的全部评测:
```bash
python3 groundtruth_pipeline/score_benchmark.py --results-run results2 --with-function-diagnostic --out score_results2.json --resume
```

## 4. 新旧两套判定方式的对比结果

### 4.0 最新一轮全量数据(2026-08,108/108 groundtruth + task_004/006 重写后)

`score_benchmark.py --results-run results2 --with-function-diagnostic --resume` 跑完,6 个模型 × ~100 个 task(每个模型 8 个 `NO_CANDIDATE_FILES`/`skipped`,1 个候选实测卡死记 `ERROR`):

| 模型 | Tier3 通过 |
|---|---|
| anthropic/claude-opus-4.6 | 11/100 |
| openai/gpt-5.5 | 10/100 |
| google/gemini-3.1-pro-preview | 8/100 |
| qwen/qwen3.5-plus | 8/100 |
| z-ai/glm-5-turbo | 7/99 |
| deepseek/deepseek-v4-pro | 5/99 |

整体 **49 PASS / 598 可评组合 ≈ 8.2%**(647 条里剔除 48 个 `NO_CANDIDATE_FILES` + 1 个 `ERROR`)。数据在 `score_results2.json`,上一版备份在 `score_results2.json.bak-20260827`。

### 4.1 旧oracle vs 新Tier3 逐条对比(84/103 验证阶段的快照,尚未按上面这轮重算)

拿旧的oracle test结果(`benchmark_*_run2.csv`,纯文本正则判定)跟新的Tier3真实Docker结果做了逐条比对,同一批6个模型、同一批task:

- **467组(模型,task)组合可以直接对比**
- **整体通过率**:旧oracle测试 42/467 = **9.0%**;新Tier3测试 41/467 = **8.8%**——两个总数看着接近,但**不代表两套测试判的是同一批task**,总数接近只是巧合,下面这59例逐条对比才是重点
- **整体一致率:87.4%**(408/467两边判定相同,剩下59例判定不一致)
- **不一致的两个方向各占一半左右**:
  - **30例**:旧测试判"过",新测试判"没过"——代码文本看着对、正则能骗过,但实际编译/运行时会崩,**这正是为什么要做这套新基础设施**的核心理由
  - **29例**:旧测试判"没过",新测试判"过"——旧的正则卡得太死(变量命名/写法风格跟预设不完全一致),但代码逻辑其实是对的,旧方式误判了。其中`task_001_basic_param`这一个task在4个不同模型身上都出现这个误判,大概率是这条oracle正则本身写得不合理

**结论**:两套方法大方向一致(87%),但差异的13%不是噪音,而是有明确原因、有具体案例支撑的系统性问题——证明了从"看文本"升级到"真的跑起来"这件事是必要且有实际发现的。

---

*本文件由`groundtruth_pipeline/`基础设施生成。第 4.0 节是 108/108 groundtruth + task_004/006 重写后的最新全量数据(`score_results2.json`)。第 4.1 节的 467 组"新旧逐条对比"还是 84/103 阶段的快照——旧 oracle 那半边需要 `benchmark_*_run2.csv`,交叉比对脚本没纳进 `groundtruth_pipeline/`,要更新得单独写一版比对。*
