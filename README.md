# visitor_wayfinding_robot —— 访客带路机器人

面向展厅访客的「语音问路 + 带路」机器人「小静」。访客直接说出目的地（无需唤醒词），
机器人用本地语音识别理解地点，通过底盘 REST 导航到目标点位，并用平台 TTS 播报。
到达后会按点位原地转身展示，再与访客继续对话；没人搭话则自动返回待命点并转回原朝向。

- 默认地图 `消防`（可用 `map` 参数覆盖）
- 点位：`乐知教育`（待命点）、`核心力量跑酷`、`创想谷`、`厕所`、`跆拳道馆`
- ROS 2 Jazzy / ament_python 包名：`learn3_d`
- 当前目录：`/home/nvidia/guide_robot/learn/visitor_wayfinding_robot`

## 功能

- **连续语音识别**：直接从 `/lyre/audio_stream` 取麦克风音频，96 kHz 重采样到
  16 kHz，能量 VAD 断句后用本地 faster-whisper（默认 `small` / `int8`）识别，
  不需要先喊唤醒词；带热词偏置、初始提示和幻觉过滤。
- **语音 / 文本双入口**：本地麦克风、平台 ASR（`/audio_asr/iat`，启动时自动登记
  唤醒词「小静」）、文本话题 `/tour/chat` 三个入口统一去重后处理。
- **可随时打断播报**：关键话术（到达、让行提示）播报期间不关麦克风，观众可以
  直接说「没事了你回去吧」等指令，不用等机器人说完。
- **地点理解**：`places.yaml` 别名匹配（含「1 号点」等自动别名）；「带我去 X」
  直接出发，只说地名先确认；「我想练跆拳道」这类间接表达交给千问推断，结果必须
  经 `places.resolve()` 校验为真实点位后才允许导航。
- **底盘导航**：默认点对点直达（`nav_direct=true`），也可按主线逐段下发；
  每段前检查定位与底盘健康并开启 AEB，到点做 0.35 m 位置校验。
- **定位容错**：区分 `slam_status` 0/1/2/3/4/5/6/7/8，能在定位降级时自动重定位，
  定位空闲时自动重载当前地图；重定位初值优先吸附 3 m 内已知点位。
- **到点转身展示**：按点位配置原地左转/右转（默认 90°），转完播报到达，
  保持 10 秒后转回原方向；转身前确认正前方无人，转完用定位 yaw 校验并补转。
- **回程转回朝向**：离开待命点时记下朝向，回到待命点后自动转回去；
  启动时另有一次兜底记录，开机不在待命点也不会漏记。
- **前向防撞**：头部深度相机 ROI 测距，正常避障交给底盘，绕不过去时停车让行；
  **永不放弃目标、不冷却**，人不走就每 15 秒重复喊话，路通后自动续行。
- **交互状态机**：待命主动问候、送达等待、明确结束语立即回程、导航中急停/改道/
  暂停问答、超时自动回待命点、播报回声过滤。
- **专属女声音色**：本项目播报前临时切 `tiangong_v4`，播完恢复平台默认
  `tiangong_v2`，不影响 learn2_Z 等其他项目。
- **Web 工作台**（可选）：`web/` 下独立进程，查看状态、相机、日志并下发白名单指令。

## 目录结构

```
visitor_wayfinding_robot/
├── src/learn3_d/
│   ├── config/places.yaml          待命点、地点别名、中转点、主线顺序
│   ├── launch/tour.launch.py       一键启动（参数均可命令行覆盖）
│   ├── learn3_d/
│   │   ├── tour_chat.py            主节点：语音/文本 -> 理解 -> 导航 + 播报
│   │   ├── nav_core.py             底盘导航核心：分段规划、定位恢复、AEB、防撞、原地转身
│   │   ├── nav_route.py            独立路线导航节点/可复用 TourNavigator
│   │   ├── mic_asr.py              本地麦克风识别（VAD + faster-whisper）
│   │   ├── front_guard.py          前向深度防撞（FrontGuard / GuardNode）
│   │   ├── places.py               地点别名、主线/支线解析
│   │   ├── qwen.py                 Qwen（vLLM OpenAI 兼容）客户端
│   │   └── chat_say.py             TTS 播报测试工具
│   ├── package.xml
│   └── setup.py
└── web/                            监控工作台（独立于导航节点）
    ├── monitor.py                  HTTP 服务（默认 8765）
    ├── index.html / live.js
    ├── start_monitor.sh
    └── README.md
```

## 环境依赖

- ROS 2 Jazzy，平台工作空间 `/home/nvidia/xos`（提供 `lyre_msgs`、`interaction_msgs`）
- 底盘 SDK：`robot_controller`（默认从 `/home/nvidia/robot_controller/src` 注入，
  可用环境变量 `ROBOT_CONTROLLER_SRC` 覆盖）
- Python：`faster-whisper`、`numpy`、`pyyaml`
- 底盘 REST 服务：默认 `http://192.168.11.10:9090`
- 大模型：`http://120.48.75.178:7006/v1`，模型 `Qwen3.5-122B-A10B`（需网络可达）
- Web 工作台额外依赖：`psutil`、`opencv-python`、`PyYAML`

## 构建

```bash
cd /home/nvidia/guide_robot/learn/visitor_wayfinding_robot
source /opt/ros/jazzy/setup.bash
source /home/nvidia/xos/setup.bash

# IDE 内置终端如报 GLIBCXX 相关错误，先清掉 IDE 注入的库路径
# unset LD_LIBRARY_PATH

colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` 下源码与 install 是硬链接关系，之后改 `.py` 不需要重新编译，
只要重启节点即可。`places.yaml` 会安装到 `install/learn3_d/share/learn3_d/config/`，
`tour.launch.py` 会安装到 `install/learn3_d/share/learn3_d/launch/`。

## 一键启动

```bash
pkill -f "install/learn3_d/lib/learn3_d/tour_chat"
source /opt/ros/jazzy/setup.bash
source /home/nvidia/xos/setup.bash
source /home/nvidia/guide_robot/learn/visitor_wayfinding_robot/install/setup.bash
ros2 launch learn3_d tour.launch.py
```

常用覆盖方式（不用改代码）：

```bash
# 换地图 / 换待命点
ros2 launch learn3_d tour.launch.py map:=消防 reloc_poi:=乐知教育

# 按主线逐段走（每站会停），默认是点对点直达
ros2 launch learn3_d tour.launch.py nav_direct:=false

# 关闭语音播报，仅测试导航
ros2 launch learn3_d tour.launch.py enable_voice:=false

# 送达后等待时间改为 0（不自动回程）
ros2 launch learn3_d tour.launch.py await_timeout:=0

# 不开导航，只做对话
ros2 launch learn3_d tour.launch.py enable_nav:=false
```

没有麦克风时可用文本话题直接测试：

```bash
ros2 topic pub --once /tour/chat std_msgs/msg/String "{data: '带我去创想谷'}"
ros2 topic pub --once /tour/chat std_msgs/msg/String "{data: '这是什么地方'}"
ros2 topic pub --once /tour/chat std_msgs/msg/String "{data: '没事了，回去吧'}"
```

单独做 TTS 播报测试：

```bash
ros2 run learn3_d chat_say
```

单独跑路线导航（不经过语音）：

```bash
ros2 run learn3_d nav_route --ros-args \
  -p map:=消防 -p start:=乐知教育 \
  -p pois:="核心力量跑酷,创想谷,厕所,跆拳道馆" \
  -p track:= -p reloc_poi:=乐知教育
```

## 到点转身行为

每个点位可独立配置转身方向和角度（`arrival_turns`、`arrival_turn_angle`）：

| 点位 | 默认动作 |
| --- | --- |
| 核心力量跑酷 | 右转 90° |
| 创想谷 | 左转 90° |
| 厕所 | 左转 90° |
| 跆拳道馆 | 左转 90° |
| 乐知教育（待命点） | 不做点位转身，回程时转回离开时的朝向 |

完整流程：

1. 到达点位，先按配置原地转身；
2. 转完播报「我们到 X 了，还有什么事吗？」；
3. 保持转身姿态 `arrival_turn_hold` 秒（默认 10 秒）；
4. 自动转回原方向，再进入 35 秒等待观众。

安全与精度保障：

- **转身前**先等正前方清空（最多 8 秒）；一直有人就放弃这次转身，但不影响到达播报。
- **转身后**用定位 yaw 校验实际角度，偏差超过 15° 自动按剩余角度补转一次。
- 转身过程中如果收到「没事了你回去吧」、新目的地或叫停，会提前结束保持并优先处理。

## 启动参数

`tour.launch.py` 暴露的参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `map` | `消防` | 底盘地图名 |
| `reloc_poi` | `乐知教育` | 重定位参考点位，也是待命点 |
| `track` | 空 | 空=点对点；填轨道名才按录制轨道走 |
| `enable_nav` | `true` | 是否连接底盘导航 |
| `nav_direct` | `true` | `true`=直接前往目标；`false`=按主线逐段走 |
| `wake_word` | `小静` | 启动时自动登记的平台唤醒词 |
| `enable_voice` | `true` | 是否使用平台 TTS |
| `enable_greet` | `false` | 开机是否立刻打招呼（默认关，改为有人靠近再说） |
| `enable_presence` | `true` | 是否检测有人靠近并主动问候 |
| `presence_distance` | `0.8` | 触发问候的前方距离（米） |
| `await_timeout` | `35.0` | 送达后无人搭话自动回待命点，0=关闭 |
| `pause_timeout` | `35.0` | 导航中被叫停后无人说话自动回待命点，0=关闭 |
| `pending_timeout` | `90.0` | 提议目的地后的确认有效期，0=不过期 |
| `qwen_base_url` | `http://120.48.75.178:7006/v1` | 模型接口地址 |
| `qwen_model` | `Qwen3.5-122B-A10B` | 模型名 |
| `qwen_timeout` | `30.0` | 模型请求超时（秒） |

其余行为参数未在 launch 中声明，调试时可直接运行节点覆盖：

```bash
ros2 run learn3_d tour_chat --ros-args \
  -p front_guard_distance:=0.4 \
  -p arrival_turn_angle:=90.0 \
  -p enable_mic:=false
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `host` / `rest_port` | `192.168.11.10` / `9090` | 底盘 REST 地址 |
| `nav_timeout` | `180.0` | 单次导航超时（秒） |
| `arrival_turns` | 见上表 | 到点转身配置，格式 `点位:left,点位:right`，支持中文左右 |
| `arrival_turn_angle` | `90.0` | 到点转身角度（度） |
| `arrival_turn_hold` | `10.0` | 转身后保持多少秒再转回 |
| `voice_id` | `tiangong_v4` | 本项目播报使用的女声音色 |
| `voice_default` | `tiangong_v2` | 播完恢复的平台默认男声 |
| `front_guard_enabled` | `true` | 前向防撞开关 |
| `front_guard_distance` | `0.5` | 直行停车距离（米） |
| `obstacle_repeat` | `15.0` | 被挡不离开时重复喊话的间隔（秒），最小 5 秒 |
| `obstacle_text` / `obstacle_resume_text` | 见代码 | 去程让行 / 续行话术 |
| `obstacle_home_text` / `obstacle_home_resume_text` | 见代码 | 回程让行 / 续行话术 |
| `enable_mic` | `true` | 本地麦克风连续识别 |
| `mic_rate_in` | `96000` | 麦克风实际采样率 |
| `vad_threshold` | `600.0` | 能量 VAD 阈值 |
| `vad_silence_s` | `0.45` | 端点静音时长 |
| `asr_model` | `small` | faster-whisper 模型 |
| `asr_beam_size` | `1` | 识别 beam |
| `presence_confirm` | `3` | 连续多少帧检测到人才算有人靠近 |
| `greet_cooldown` | `20.0` | 两次主动问候的最小间隔（秒） |
| `history_turns` | `6` | 送给大模型的历史对话轮数 |
| `places_file` | 空 | 自定义 places.yaml 路径 |
| `log_file` | `~/guide_logs/tour_chat_日期.log` | 对话日志路径 |

## 地点配置 places.yaml

```yaml
standby: 乐知教育        # 待命点

aliases:                # 别名 -> 底盘点位名
  创想谷: 创想谷
  3号点: 创想谷

transfer_points:         # 只作中转，不出现在可去地点清单
  - start_point

main_line:               # 主线必须按物理顺序排列
  - 乐知教育
  - 核心力量跑酷
  - 创想谷
  - 厕所
  - 跆拳道馆

# branches:              # 可选：支线点位 -> 挂接的主线点
#   某支线点: 创想谷
```

- 点位名必须与底盘地图 `get_pois()` 返回的 `name` 一致。
- 同一地点可配多个别名；配了中文别名的点位，播报和模型清单显示中文名。
- 带数字后缀的点位会自动生成「N 号点 / 第 N 个点」等别名。
- `nav_direct=true` 时主线只用于配置完整性，不参与逐段下发；
  改为 `false` 才会按主线顺序依次经过每个点，不跳点。
- 到点转身与 `places.yaml` 无关，由节点参数 `arrival_turns` 配置；
  改点位名后记得同步改这里，否则该点位不会转身。

## 交互规则

| 场景 | 说法示例 | 行为 |
| --- | --- | --- |
| 直接出发 | 带我去创想谷 / 去厕所 | 直接导航 |
| 只说地名 | 创想谷 | 先问「需要我带你去吗」，确认后出发 |
| 间接表达 | 我想练跆拳道 | 千问推断地点，校验后询问确认 |
| 结束 / 回程 | 没事了、回去吧、不用了、再见 | 立即返回待命点，不等超时 |
| 到达播报中说回程 | 没事了你回去吧 | 播报中也能识别，播完立即回程 |
| 导航中急停 | 停、停一下、别动、站住、刹车 | 取消当前导航，进入暂停 |
| 导航中改道 | 带我去跆拳道馆 / 只说「创想谷」 | 取消当前任务并改去新地点 |
| 导航中被叫住 | 机器人 / 小静 / 你好 / 这是什么 | 暂停导航并回答，随后追问是否继续 |
| 想换地方 | 我要去别的地方 | 停车后询问新目标 |
| 继续 | 继续、接着走、可以走了 | 恢复原目标导航 |
| 送达后 | 35 秒无人搭话 | 自动回待命点并转回原朝向 |

- 回声过滤覆盖最近 25 秒的播报内容，避免机器人把自己的话当成指令。
- 大模型不可用时统一回「不好意思，我没太听清，您再说一遍好吗？」并冷却 15 秒。
- 被问到「这是哪」时会把当前点位注入 system prompt，不会编造位置。

## 音色说明

平台 TTS 全局只有一套音色，所以本项目采用「播报前切女声、播完切回默认」的方式：

- 本项目（visitor_wayfinding_robot）和 guided_tour_robot 使用 `tiangong_v4`（女声）；
- learn2_Z 等其他项目不切音色，保持平台默认 `tiangong_v2`（男声）；
- 切换走 `/intelligent_interaction/tts/update_voice_id`；服务不可用或超时会告警，
  并在 60 秒内跳过切换，不阻塞播报。

## 导航与安全策略

- 导航前 `check_ready()`：定位状态可用、底盘无 A/B 类故障、开启 AEB；
  定位空闲（`slam_status=4`）会先自动重载当前地图启动定位。
- 定位状态处理：
  - `0` 正常；
  - `1/2/3/5` 允许主动重定位；
  - `6/7/8` 底盘正在自行恢复，只等待不打断；
  - `4` 定位服务未启动，自动 `start_localization`，失败则明确提示人工处理。
- 重定位初值优先级：3 m 内最近已知点位坐标 > 最近 90 秒可信位姿 > 当前冻结位姿。
- 每段 `move_to` 前要求定位连续稳定 3 秒；相邻段可复用上一段的稳定观测。
- 底盘报失败但实际位姿距目标 ≤1 m 时按到达处理。
- 前向防撞：ROI 内 5% 分位距离 ≤0.5 m，或无效像素占比 ≥6%，连续 3 帧确认后
  取消导航并播报让行；前方需连续清空 1 秒才重新下发。
- 被挡后目标一直保留：不放弃、不冷却，人不走就每 15 秒重复提醒，路通自动续行。
- 停车使用「cancel + 等任务退出 + 再 cancel」，不使用零速度指令。
- 正常避障优先交给底盘，应用层只在绕不过去时停车让行，不主动绕行。
- 原地转身用 `move_by(move_mode=1)`，正角度左转、负角度右转；
  转身前等正前方清空，转身后校验角度并补转，失败不影响后续交互。

## Web 工作台

```bash
cd /home/nvidia/guide_robot/learn/visitor_wayfinding_robot/web
bash start_monitor.sh --port 8765
```

浏览器访问 `http://<机器人IP>:8765/`。只读接口展示主机/ROS/相机/电量/任务状态；
写操作仅限启动/停止 `tour_chat` 节点、按 `places.yaml` 白名单下发导航、
发送任意文本到 `/tour/chat`，且请求体必须带 `{"confirm": true}`。
详见 [web/README.md](web/README.md)。

## 日志与排障

- 对话日志：`~/guide_logs/tour_chat_YYYYMMDD.log`，每条记录带时间、类型
  （听到 / 文本输入 / 播报 / 导航 / 改道 / 暂停 / 忽略 / 障碍 / 转身）。
- 常见问题：

| 现象 | 排查方向 |
| --- | --- |
| 说话完全没反应 | 平台 ASR 未使能；检查 `/intelligent_interaction/lyre/enable` 是否持续为 true |
| 能听到但不导航 | 看日志是否被判定为闲聊/回声；确认 places.yaml 点位名与底盘一致 |
| 导航被拒 | 查看 `slam_status`、定位置信度、底盘 DTC；维护模式未退出会被拒绝 |
| 定位频繁掉 | 用最近点位坐标做重定位初值；确认机器人实际停在点位附近 |
| 播报没声音 | TTS 服务是否就绪；确认 `enable_voice:=true` 且 `audio_guide` 可用 |
| 播报不是女声 | 查日志是否出现「音色服务未就绪/超时」；服务恢复后 60 秒冷却结束会再切 |
| 到点不转身 | 检查点位名是否与 `arrival_turns` 完全一致；日志应有「原地左转/右转」 |
| 转身方向反了 | 确认底盘 `move_by` 正角度的实际方向；必要时把 `arrival_turns` 的 left/right 对调 |
| 转身角度不准 | 看日志「转身实际 ±…°，补转 ±…°」；底盘精度差会补转一次，仍偏差大需现场标定 |
| 回程不转回朝向 | 日志应有「记下待命朝向」；定位不可用时无法记录/转身，属预期 |
| 原地转向卡顿 | 属于底盘底层执行问题，可改 `nav_direct:=true` 减少转向次数 |

## 已知边界

- 底盘导航为闭源模块，应用层只能下发任务级目标，不能修改规划/避障参数；
  导航异常优先排查地图质量、点位录制、定位状态和底盘配置。
- 当前 `places.yaml` 未配置支线（`branches`），所有可去点位都在主线上。
- 大模型为远程服务，断网时会走兜底话术，导航与语音识别仍可用。
- 平台音色是全局设置，本项目播报期间其他项目若恰好也播报，会短暂使用女声；
  播完立即恢复默认男声。
