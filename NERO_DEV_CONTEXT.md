# Nero双臂接入LeRobot开发上下文

> **用途**：本文件是开发传话筒，用于在不同主机、不同AI对话会话之间传递关键上下文。
> 阅读对象：人类开发者 + AI助手。
> 维护规则：每次完成重要改动或确认关键决策后更新此文件。

---

## 一、目标

将两条**松灵NERO 7自由度机械臂**接入 [LeRobot](https://github.com/huggingface/lerobot) 框架，实现：

1. **模式A** — 软中继示教录制：人拖动主臂（leader），PC读角度转发给从臂（follower），100Hz
2. **模式B** — 双臂独立PC控制（`BiNeroFollower`），用于策略推理阶段
3. 配合 **Intel RealSense D435i** 相机采集图像观测

硬件环境：Windows 11 PC，无显卡（模型训练在另一台主机上），两条臂各接一个USB-CAN适配器。

---

## 二、硬件与SDK关键信息

### CAN总线
- 协议：CAN 2.0B，1 Mbps，每帧独立
- 连接：每条臂一个独立USB-CAN适配器（`agx_cando` 接口）
- SDK：`pyAgxArm`，位于 `D:\repo\lerobot\pyAgxArm`

### CAN ID分配（速查）

| 方向 | ID范围 | 内容 |
|------|--------|------|
| 主臂→总线（leader广播） | `0x501~0x507` | 7关节角度，rad |
| 臂→PC（慢速反馈） | `0x2A1~0x2A9` | 25ms周期 |
| 臂→PC（快速反馈） | `0x251~0x257` | 5ms周期 |
| PC→臂（关节控制） | `0x155, 0x156, 0x157, 0x170` | 位置命令 |
| PC→臂（模式设置） | `0x470` | Byte0: `0x00`=normal, `0xFA`=leader, `0xFC`=follower |
| PC→臂（重力补偿设置） | `0x487` | 7关节独立，uint8，0~10（SDK暂未实现） |
| 臂→PC（重力补偿反馈） | `0x488` | — |

### pyAgxArm SDK关键API

```python
from pyAgxArm import ArmModel, AgxArmFactory, create_agx_arm_config

arm_cfg = create_agx_arm_config(
    robot=ArmModel.NERO,
    firmeware_version=fw_profile_str,  # ⚠️ SDK拼写错误，参数名是 firmeware_version，不能改
    channel="0",                        # Windows agx_cando通道号，字符串
    interface="agx_cando",
)
arm = AgxArmFactory.create_arm(arm_cfg)

arm.connect()
arm.enable()
arm.set_leader_mode()               # 零力拖动，广播0x501~0x507
arm.set_follower_mode()             # 接收move_j命令
arm.set_normal_mode()               # 普通单臂PC控制
arm.get_leader_joint_angles()       # 读0x501~0x507，返回list[float]，7个关节，rad
arm.get_joint_angles()              # 标准反馈，返回list[float]，7个关节，rad
arm.move_j(joints: list[float])     # 有轨迹规划，安全
arm.move_js(joints: list[float])    # MIT直通，无平滑
arm.disable()
arm.disconnect()
```

> **SDK拼写错误**：`create_agx_arm_config` 的参数名是 `firmeware_version`（非标准拼写）。
> 代码注释已标注 `# NOTE: SDK typo — do not change to "firmware_version"`，永远不要修正这个拼写。

### 固件版本说明（重要）

| 版本来源 | 说明 |
|----------|------|
| 上位机显示的 `V2.0.7` | **关节驱动板（节点）固件版本**，与SDK无关 |
| 上位机显示的 `H-V1.3.0` | **硬件版本** |
| `NeroFW` 枚举 / `firmware_version` 参数 | **臂主控板SDK驱动档位**，与节点固件无关 |

`NeroFW` 枚举对应关系：

| 字符串值 | 对应 | 说明 |
|----------|------|------|
| `"default"` | `NeroFW.DEFAULT` | ≤1.10，**当前使用的保守默认值** |
| `"1.11"` | `NeroFW.V111` | |
| `"1.12"` | `NeroFW.V112` | |
| `"1.20"` | `NeroFW.V120` | |
| `"1.21"` | `NeroFW.V121` | ≥1.21 |

**当前策略**：代码默认 `firmware_version = "default"`，实机连接后调 `arm.get_firmware()` 确认主控板版本，再按需更新。

### 固件≥1.12安全注意
- `disable()` 在 leader 模式下会被忽略
- 断开连接前必须先调 `set_follower_mode()`，代码已实现此逻辑

---

## 三、代码改动清单

### 新增文件

| 文件 | 状态 | 说明 |
|------|------|------|
| `src/lerobot/robots/nero_follower/__init__.py` | ✅ 已创建 | 导出 `NeroFollower`, `NeroFollowerConfig` |
| `src/lerobot/robots/nero_follower/config_nero_follower.py` | ✅ 已创建 | `NeroFollowerConfig` dataclass |
| `src/lerobot/robots/nero_follower/nero_follower.py` | ✅ 已创建 | `NeroFollower(Robot)` 完整实现 |
| `src/lerobot/robots/bi_nero_follower/__init__.py` | ✅ 已创建 | 导出 `BiNeroFollower`, `BiNeroFollowerConfig` |
| `src/lerobot/robots/bi_nero_follower/config_bi_nero_follower.py` | ✅ 已创建 | `BiNeroFollowerConfig` dataclass |
| `src/lerobot/robots/bi_nero_follower/bi_nero_follower.py` | ✅ 已创建 | `BiNeroFollower(BimanualMixin, Robot)` 实现 |
| `src/lerobot/teleoperators/nero_leader/__init__.py` | ✅ 已创建 | 导出 `NeroLeader`, `NeroLeaderConfig` |
| `src/lerobot/teleoperators/nero_leader/config_nero_leader.py` | ✅ 已创建 | `NeroLeaderConfig` dataclass |
| `src/lerobot/teleoperators/nero_leader/nero_leader.py` | ✅ 已创建 | `NeroLeader(Teleoperator)` 完整实现 |

### 修改文件

| 文件 | 修改内容 |
|------|----------|
| `src/lerobot/robots/utils.py` | 添加 `nero_follower`、`bi_nero_follower` 两个 `elif` 分支 |
| `src/lerobot/teleoperators/utils.py` | 添加 `nero_leader` 的 `elif` 分支 |
| `src/lerobot/scripts/lerobot_teleoperate.py` | 在 robots import 块添加 `bi_nero_follower`、`nero_follower`；在 teleoperators import 块添加 `nero_leader` |

### 关键设计决策

- **`nero_follower.py` 中不调用 `resolve_firmware_profile()`**：config 里存的直接是 SDK profile 字符串（如 `"default"`），不是 semver，直接传给 `create_agx_arm_config(firmeware_version=...)` 即可
- **action/observation key 格式**：`joint_1.pos` ~ `joint_7.pos`，单位 rad，float
- **`BiNeroFollower` 相机前缀约定**：顶层 `config.cameras` 中的相机保持无前缀（共享相机），左/右臂各自的相机加 `left_` / `right_` 前缀，与 `BiSOFollower` 保持一致
- **断开连接顺序**：`set_follower_mode()` → `disable()` → `disconnect()`（固件≥1.12安全要求）

---

## 四、待办事项

### 实机连接后必须验证

- [ ] `arm.get_joint_angles()` 是否直接返回 `list[float]`，还是返回 `MessageAbstract` 对象（需要 `.msg[i]` 访问）
- [ ] `arm.get_firmware()` 返回结果，确认主控板固件对应哪个 `NeroFW` 档位，更新 `firmware_version` 默认值
- [ ] `move_j`（有轨迹规划）vs `move_js`（MIT直通）跟随效果对比，决定最终使用哪个
- [ ] 两臂 CAN 通道号确认（哪条臂是 `"0"`，哪条是 `"1"`）

### 低优先级

- [ ] **重力补偿接口（0x487）**：pyAgxArm SDK 的 Nero 端无此实现（仅 Piper 6关节有 `set_joint_assistance_rating()`）。需扩展 SDK 或用 raw CAN 发送。延后处理。

### 相机配置

- Intel RealSense D435i：LeRobot 支持，配置类 `RealSenseCameraConfig`
- 具体参数（分辨率、fps、是否使用深度）未定，根据实际需求配置

---

## 五、模式A启动命令（软中继示教）

```shell
# 安装lerobot后在此仓库根目录运行
lerobot-teleoperate \
    --teleop.type=nero_leader \
    --teleop.channel=0 \
    --robot.type=nero_follower \
    --robot.channel=1 \
    --fps=100
```

> `channel=0` 是主臂（leader，零力拖动），`channel=1` 是从臂（follower，执行命令）。
> 实际通道号需根据USB-CAN适配器插入顺序确认。

### 加相机示例

```shell
lerobot-teleoperate \
    --teleop.type=nero_leader \
    --teleop.channel=0 \
    --robot.type=nero_follower \
    --robot.channel=1 \
    --robot.cameras='{front: {type: realsense, serial_number_or_name: "...", width: 640, height: 480, fps: 30}}' \
    --fps=100
```

---

## 六、仓库 git 状态说明

`git status` 显示 1400+ 个 `D`（deleted）状态文件，这些**不是本次开发造成的**。

原因：这个仓库是从上游 LeRobot 仓库 clone 下来的，这些文件在工作树中被删除（可能是手动清理了非必要文件），但 git 还记录着。本次实际新增约9个文件、修改3个文件（见上方"代码改动清单"）。

---

*最后更新：2026-11-25*
