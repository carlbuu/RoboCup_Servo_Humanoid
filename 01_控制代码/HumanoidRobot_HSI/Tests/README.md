# 主机测试

这些测试模拟硬件接口，不能证明相机、串口接线、实际落脚或机器人平衡已经通过验证。

## 当前资料包的主要入口

进入本目录，使用 Python 3.11 或更新版本、PowerShell 7，以及可在本机运行的 GCC。
GCC 应加入 PATH，或通过 `-Compiler` 指定其完整路径。主机 GCC 与 Keil 的 ARM 编译器用途不同。

```powershell
python ./test_walk_late_camera.py
./run_dual_gait_tests.ps1
./run_walk_turn_handoff_tests.ps1
./run_walk_late_vision_tests.ps1
```

`test_walk_late_camera.py` 从资料包中的 19 版视觉文件提取真实纠偏类，使用标准库验证连续建议、平移、丢线和回差门限，不依赖相机或 OpenCV。

其余三项使用实际相机解析器、调度器、播放器和动作表，在主机上模拟硬件接口。输出位于各自的 `*_build` 目录，已由 `.gitignore` 排除。

## 需要额外输入的测试

| 入口 | 使用条件 |
|---|---|
| `run_three_mode_tests.ps1` | 针对其他配置的综合断言，不能用来宣称当前全部配置通过；视觉文件参数仅用于源码完整性核对 |
| `run_motion_timing_audit.ps1` | 包含对图形附加直行阶段的固定断言，与当前默认关闭该阶段的配置存在差异 |
| `test_walk_recovery_original_limit.c` | 保留原收脚端点差值断言；关键帧端点差值与单个 20 ms 插值变化不是同一指标 |
| `run_group_only_stand_tests.ps1`、`run_left_turn_settle_tests.ps1`、`run_shape_extra_step_tests.ps1` | 针对特定动作配置的断言；测试前后比对当前源码哈希，配置不同可能失败 |
| `run_left_leg_recovery_tests.ps1` | 还需通过 `-BaselineVisionSource` 提供匹配的参考调度源码；资料包不含该参考文件 |
| 其他视觉 Python 测试 | 针对配套参考版本的 API 和行为；需显式设置 `VISION_REFERENCE_SOURCE`，不能直接改绑 19 版后认定断言仍适用 |

这些入口仅定位当前工程。源码完整性检查使用运行前的内存快照，不再依赖旧 JSON 校验记录，也不会切换到其他工程目录。

参考视觉测试还可能需要 NumPy 和 OpenCV；MaixCAM 上运行生产脚本则使用设备自身的 MaixPy 环境。当前版本的标准库测试不需要安装这些依赖。

## 持续检查

根目录 `.github/workflows/host-checks.yml` 配置了当前标准库视觉测试和三组主要 C 主机测试。工作流实际结果以 GitHub Actions 运行记录为准。Keil 构建与硬件验证需另外进行。
