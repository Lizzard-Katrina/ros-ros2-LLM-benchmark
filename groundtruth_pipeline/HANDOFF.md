# 交接说明:ROS1→ROS2翻译Benchmark的Groundtruth+Docker基础设施(最新版)

这是给新窗口/新session的完整背景说明。先读这份文件,再继续手头的工作。

## 项目背景

ROS1→ROS2翻译能力benchmark。每个task目录下有`ros1_code/`(挖空了部分逻辑、留了TODO标记的ROS1代码),多个LLM模型被要求填空翻译成ROS2,翻译结果存在`results/`、`results1`~`results5`(每个模型一个子目录,原始prompt+回复存在`dialogue.md`里)。

**最初的问题**:原有`tests/test_oracle_ros2.py`只是对翻译代码文本做正则匹配,从没真正编译、运行过。`groundtruth_pipeline/`这整套是为了造一份**真正在Docker里build成功、真正跑起来产生正确行为**的groundtruth,再拿它去评测每个模型的翻译。

## 当前状态(重要,截至目前最新)

- **108/108个task全部`GROUNDTRUTH_VERIFIED`**,每个都有`Dockerfile_real`(可以直接`docker build -f Dockerfile_real .` + `docker run`独立复现,不依赖任何Python胶水代码)
- 共享基础镜像`groundtruth-base:humble`(基于`osrf/ros:humble-desktop`,预装colcon/pytest/常用ROS2包/socat),所有task共用,连Gazebo/PX4/CARLA相关的task都不需要额外专用镜像(它们的测试在仿真器边界做了mock)
- 已经用`results2`(6个模型)跑过一轮完整候选评测,结果在`score_results2.json`,通过率5%~13%
- 跟旧oracle CSV(`benchmark_*_run2.csv`)做过逐条对比:467组可比,一致率87.4%,不一致的59例都有具体案例可查(`MEETING_SUMMARY.md`里有完整数据)
- **做了"投毒测试"(mutation testing)专门检测"测试有没有真的在测目标代码"这个风险**:把groundtruth里挖空对应的那段逻辑替换成空壳(编译能过但什么都不做),重跑同一个`Dockerfile_real`,如果测试还是PASS就实锤证明测试是"影子测试"。跑了全部103个当时的task,结果:
  - 47个 `TEST_IS_REAL`(挖空后测试正确FAIL,证明测试真实有效)
  - **7个 `SUSPICIOUS_SHADOW_TEST`**(挖空后测试还是PASS,需要人工确认)
  - 40个 `MUTATION_NOT_APPLICABLE`(翻译时函数被重构改名,这个工具定位不到,不代表有问题,只是测不了)
  - 其余是编译失败/超时等不算数的情况

## 那7个可疑task的处理进度(2026-08 更新)

| Task | 处理结果 |
|---|---|
| `interface_level/parameter_server/task_004_turtlrbot3_params` | ✅ **已重写**。`turtlebot3.cpp` 真跑不了(依赖 turtlebot3_node 包 + DynamixelSDK + OpenCR 硬件,Dockerfile 里没编译任何东西)。做法:新增 `ros2_code/source/src/profile_accel_param_node.cpp`(搬运 `init_dynamixel_sdk_wrapper`/`parameter_event_callback` 里一模一样的 `AsyncParametersClient`+`on_parameter_event`+`value/constant` 逻辑,转换结果发到 `~/profile_acceleration_converted`),`test_runtime_ros2.py` 删掉两个"自己重写一遍逻辑"的假测试,换成真 `ros2 run` 起节点+改参数+断言除法转换(429.154→2.0 / 1072.885→5.0 / 非乘法)+订阅 `/parameter_events`;静态 oracle 检查保留。`metadata.json` 写清逐字 runtime 不可行、投毒对本 task 天然 N/A。已 `GROUNDTRUTH_VERIFIED`。 |
| `interface_level/action_server/task_006_amcl_navigation` | ✅ **已补断言**。投毒掏的是 `laserReceived()`(ROS1 TODO 在这函数里),旧测试从不发 LaserScan 所以掏空也过。新增 `test_laser_scan_triggers_pose_update`(只发 LaserScan、不发 goal,强制走 `laserReceived` 并要求 `amcl_pose` 出估计位姿);给两个已有 action 测试补数值断言(位姿 0,0,0 + 单位四元数 + 协方差对角 0.25/0.25/(π/12)²)。顺带修了个 flaky:`main()` 单线程 `spin` + detached `execute()` 线程调 `succeed()` 抢 `status==0` → 改 `MultiThreadedExecutor`。连跑 3 次 `GROUNDTRUTH_VERIFIED`。 |
| `interface_level/parameter_server/task_001_basic_param` | ⏳ **待办,卡在 OpenRouter 额度**。需要用 `build_groundtruth.py`(要 LLM)重新翻译。注:`ros2_code/source/param.cpp` 里的缓存逻辑其实翻译得还算忠实(getImpl 命中短路 / subscribe-on-miss / update 门控 / invalidateParentParams 走父命名空间),真正的问题是 `param.cpp` 没被链接进任何 node/test、`param_cache_node.cpp` 的 `main()` 只 spin、`test_runtime_ros2.py` 只调 ROS2 内置参数服务 → 掏空 `param.cpp` 测试照过。修法建议:把 `param.cpp` 编进一个 node、开个 `CacheQuery` 服务,写真缓存语义测试(stale-hit / 父失效 / subscribe-on-miss)。额度恢复后做。 |
| `interface_level/publisher_subscriber/task_002_custom_msg_basic` | ✅ **确认误报**。`function_splice` 掏的是 subscriber `callback`,真正的空在 publisher 侧(`timer_callback` 填 name/age/height),`msg.name=="Alice"` 那组断言经 test 自带 subscriber 真实验证。不用管。 |
| `interface_level/publisher_subscriber/task_007_latched_publisher` | ✅ **确认误报**。掏的是 `main()`;测试直接 import `LatchedPubSubNode` 类,验证 `QoSDurabilityPolicy.TRANSIENT_LOCAL` + 迟到订阅者收到最后消息。不用管。 |
| `system_level/multi_node/task_001_rosserial_python_integration` | ✅ **确认误报**。`SerialClient.py` 里有 6 个 `__init__`,`_py_extract_function("__init__")` 匹配到**第一个**(`Publisher.__init__`),不是 `SerialClient.__init__`;后者被 `test_serial_client_dependency_injection` 充分测。不用管。 |
| `interface_level/service_client/task_005_gazebo` | ⚠️ **不是误报但优先级低**。`spawn_urdf_model_client` runtime 确实没被调用(runtime 那个 `gazebo_interface_node` 是另一个 `SetBool` 桩),只有 `hasattr`/`callable`+文本正则碰它。但两个同构姊妹函数(`spawn_sdf_model_client`/`set_model_configuration_client`)+ `test_gazebo_interface_imports` 的文本断言把迁移模式锁住了。建议补:对桩 `task_005_gazebo/srv/SpawnEntity` 服务真调 `spawn_urdf_model_client` 并断言返回值。可延后。 |

---

<details><summary>原始分析(展开)</summary>

已经逐个读过测试内容,结论:

| Task | 诊断 | 优先级 |
|---|---|---|
| `interface_level/parameter_server/task_001_basic_param` | **最严重**:groundtruth翻译本身可能绕过了难点。原始ROS1挖空的是"带缓存失效机制的参数缓存层"(`getImpl`带`use_cache`、`invalidateParentParams`),现在的groundtruth只是个用ROS2内置通用get/set服务的简单节点,测试测的是ROS2自带机制,根本没测到"缓存"这个核心需求。**需要重新翻译,不只是补测试** | 🔴 高 |
| `interface_level/parameter_server/task_004_turtlrbot3_params` | **确认是纯静态文本测试**,断言全是`re.search(pattern, code_content)`,没有任何真实进程启动/交互,是彻头彻尾的假"runtime test"。**需要重写成真实runtime测试** | 🔴 高 |
| `interface_level/action_server/task_006_amcl_navigation` | 真测试(真启动节点、真发action goal),但只测了action外壳能不能连上/有没有回复,没测挖空的粒子滤波核心算法算出来的具体姿态数值对不对。**需要补充断言,检查具体数值** | 🟡 中 |
| `interface_level/publisher_subscriber/task_002_custom_msg_basic` | 断言看起来真实具体(`msg.name=="Alice"`这种)。大概率是投毒工具误伤(挖空函数可能不在这次测试触发的代码路径上) | 🟢 低,建议抽查确认后可以忽略 |
| `interface_level/publisher_subscriber/task_007_latched_publisher` | 断言很扎实(检查QoS TRANSIENT_LOCAL、迟加入订阅者收到最后消息),正好在测latched publisher的核心语义 | 🟢 低,大概率误伤 |
| `interface_level/service_client/task_005_gazebo` | 断言检查真实service响应内容 | 🟢 低,大概率误伤 |
| `system_level/multi_node/task_001_rosserial_python_integration` | 断言检查真实对象状态和消息内容 | 🟢 低,大概率误伤 |

**下一步该做的**:优先处理前两个(🔴),中间那个(🟡)补个断言就行,后面4个(🟢)可以先抽1-2个人工确认一下投毒工具的判断确实是误报,如果确认误报就不用管了。

## 已经修复的两个真实bug(供参考,方法可复用)

1. `task_003_tutorial_baseline`:CMake报错"add_custom_target...already exists"——原因是`rosidl_generate_interfaces`生成消息/服务Python绑定时会自动调用一次`ament_python_install_package`,跟CMakeLists.txt里手写的又调用了一次、用的是**同一个包名**,冲突了。修法:把手写Python代码的目录**改名**(比如加`_py`后缀),跟接口自动生成那部分的包名区分开,同步改CMakeLists.txt和所有import路径。
2. `task_002_custom_srv`:测试报`SyntaxError: unexpected character after line continuation character`——原因是用`repr(多行python脚本)`塞进`python3 -c '...'`这种bash单引号命令时,**换行符被转成了字面上的反斜杠+n两个字符**,不是真换行。修法:别用`repr()`塞命令行,把脚本**写到临时文件**里再`python3 <tempfile>`执行。

## `groundtruth_pipeline/`目录下每个文件的作用

| 文件 | 作用 |
|---|---|
| `task_context.py` | 读取一个task的所有上下文:`ros1_code/*.py|cpp`(挖空文件)、`ros1_code/source/*`(clone的原始参考,可能没有)、`REFERENCE_NOTES.md`(网页tutorial摘要,可能没有)、`metadata.json`、`README.md`、`tests/test_oracle_ros2.py` |
| `prompts.py` | 给LLM的system prompt(填TODO、产出完整可编译package骨架、写真实调用被测文件的运行时测试、禁止在测试里自己重写逻辑替代真调用) + `[FILENAME: x]`解析器 |
| `openrouter_client.py` | OpenRouter API封装,默认模型`anthropic/claude-opus-4.6`(跟仓库里judge脚本一致) |
| `docker_verify.py` | Docker执行引擎:起容器、`colcon build`、找`test_runtime_ros2.py`、跑`pytest-3`(不能用`python3 -m pytest`,会有`sys.path`把本地`launch/`目录顶替真实ROS2 `launch`包的坑)、超时兜底不炸穿 |
| `build_groundtruth.py` | 主编排器:prompt→LLM→解析文件→docker验证→失败就把报错回灌LLM重试(默认4轮)→存结果。`--task <dir>`单个,`--all --skip-existing`批量续跑 |
| `build_manifest.py` / `task_manifest.json` | 生成/存储108个task的状态清单 |
| `generate_dockerfiles.py` | 给每个`GROUNDTRUTH_VERIFIED`的task生成`Dockerfile_real` |
| `docker/Dockerfile.base` | 共享基础镜像定义 |
| `extract_candidate.py` | 从模型的`dialogue.md`提取干净代码(兼容新旧两种格式) |
| `function_splice.py` | 定位挖空函数边界(C++花括号匹配/Python缩进匹配),TODO检测要求前面紧跟注释符号(不能裸`"TODO"`,会误匹配`getOdomPose`这种函数名) |
| `evaluate_candidate.py` | 单个candidate评测:把候选翻译换进groundtruth包,用groundtruth自己的`Dockerfile_real`原样build+run。支持`--candidate-dialogue`直接读dialogue.md,`--with-function-diagnostic`额外跑"只测挖空部分"诊断 |
| `score_benchmark.py` | 批量扫描`results{N}/<model>/<category>/<task>/dialogue.md`跑分,`--resume`断点续跑,每条边跑边存 |
| `audit_harness.py` | 静态审计:关键词扫描测试文件是否提到目标文件名 |
| `mutation_test.py` | **投毒测试**:把groundtruth对应挖空的函数替换成空壳,重跑测试,PASS则标`SUSPICIOUS_SHADOW_TEST`,支持`--resume` |
| `MEETING_SUMMARY.md` | 给mentor开会用的四问四答总结(判定标准/Docker怎么搭/怎么跑/新旧对比数据) |
| `HANDOFF.md` | 就是这份文件 |

## 环境相关提醒

- `export OPENROUTER_API_KEY=...`每个新终端要重设一次
- `export GROUNDTRUTH_BASE_IMAGE=groundtruth-base:humble` 用固定镜像跑`verify_only.py`/`mutation_test.py`等,否则会退回慢速的原始镜像
- 长任务一律`nohup python3 -u xxx.py ... > xxx.log 2>&1 &`后台跑,`-u`避免输出缓冲卡住误判
- WSL2跑Docker偶尔会莫名断(containerd自己重启,不是内存问题),`score_benchmark.py`/`mutation_test.py`/`build_groundtruth.py`都支持`--resume`/`--skip-existing`断点续跑,断了原样重跑命令就行
- 后台跑之前**用`ps aux | grep <脚本名>`确认没有重复进程在跑**,避免资源浪费/冲突

## 完整待办清单(按优先级)

1. ⏳ 🔴 **重新翻译`task_001_basic_param`** —— **卡在 OpenRouter 额度(402)**,需 `build_groundtruth.py`(要 LLM)。额度恢复后做,修法见上面的进度表。
2. ✅ 🔴 **重写`task_004_turtlrbot3_params`** —— 已完成(见进度表),已 `GROUNDTRUTH_VERIFIED`。
3. ✅ 🟡 **`task_006_amcl_navigation`补断言** —— 已完成(见进度表),连跑 3 次 `GROUNDTRUTH_VERIFIED`。
4. ✅ 🟢 **抽查 4 个** —— task_002 / task_007 / task_001_rosserial 确认误报;task_005_gazebo 是真影子测试但优先级低(建议见进度表)。
5. ✅ `verify_only.py` 重验 + `generate_dockerfiles.py` 重生成(task_004/006 已做,`GROUNDTRUTH_STATUS.json` 已刷新)。
6. ⏳ **正在跑** `score_benchmark.py --results-run results2 --with-function-diagnostic --out score_results2.json --resume`(后台,日志 `score_results2_rerun.log`)。已把 `score_results2.json` 里 task_004/006 的 12 行删掉让 `--resume` 重跑,旧文件备份在 `score_results2.json.bak-*`。这轮会补齐之前 84/103 阶段没覆盖的 ~144 个组合 + 重跑 12 个,共 156 个。
7. ⏳ 跑完第 6 步后,按新的 `score_results2.json` 更新 `MEETING_SUMMARY.md` 第 4 节的对比数字(第 2 节已更新为 108/108 + 投毒复核结论)。
8. 如果论文需要,扩展到`results1`/`results3`/`results4`/`results5`跑`score_benchmark.py`
9. 写作时,投毒测试的结果(47真实/7可疑/40测不了)是很好的方法论素材,建议写进论文的limitations/validity部分,不要回避

## 一句话现状总结

108个task的groundtruth全部真实build+run验证过,evaluate候选模型的链路也跑通并拿到了真实数据,现在处于"质量精修"阶段——投毒测试筛出的 7 个可疑 case 已全部人工处理完:task_004/006 重写并重新验证,task_001 等 OpenRouter 额度恢复后重新翻译,其余 4 个是工具误伤或低优先级。`score_benchmark.py` results2 全量正在后台重跑。
