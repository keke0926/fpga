# 构建与验证

## 快速使用

直接下载 `firmware/GuangWenVision_FULL_V16_I3.bit` 可恢复全部已集成功能。该文件与原 V16 发布位流逐字节一致，SHA256 为：

```text
25ee46a93e4ece2246cd455c1634a980ba864d6f4a7dc671e482291fc6cc1339
```

## 工程重建

安装 Efinity 2025.1.110.5.9，器件选择 Ti60F225，时序模型 I3。在 Efinity 中打开 `project/Ti60_Demo.xml`，顶层为 `example_top`。源码、生成的 IP RTL、字库、模板、接口配置和原约束均包含在工程内。也可运行：

```powershell
powershell -ExecutionPolicy Bypass -File scripts/build.ps1 -EfinityRoot D:/Efinity/2025.1
```

脚本执行 interface、map、pnr、pgm，生成新的构建结果。工具版本、布局布线与机器环境会影响结果；重新生成的位流需要重新检查时序，不继承归档位流的签核结论。比赛现场可直接使用已签核的随附位流，避免时间不足时重新编译。

## 文件校验

Python 3 标准库即可执行：

```text
python scripts/verify_submission.py
```

脚本检查提交清单中的全部源码 SHA256、位流、工程设计文件和约束引用。它用于确认代码和位流未丢失或混版，不代替硬件测试。

## 证据说明

- `evidence/SUBMISSION_MANIFEST.json`：本次提交的完整文件清单、原版本来源和核验结果。
- `evidence/SIGNOFF_V16.json`：V16 原约束 I3 签核记录，112 报告族、114 项检查。
- `evidence/resource_report.rpt`：原发布版本资源报告。
- `evidence/setup/`、`evidence/hold/`：原发布版本全局时序及结构检查摘录。
- `evidence/BOARD_FEEDBACK_V16.json`：用户下载后的实物反馈与汇总诊断记录。

历史证明文件中保留原开发环境绝对路径，便于追溯；运行本提交工程和校验脚本不依赖这些历史目录。

## 当前能力边界

实时采集显示、模式与阈值控制、通用识别、参考示教、板端零件检测流程、中文标注及冻结恢复已集成。V16 实测确认功能正常，同时残缺螺母和异类法兰盘出现“未见候选”。因此本提交保留功能实现，并明确缺陷定位／召回仍需优化。

没有把离线开发样本结果写成工业准确率；没有把有效显示更新频率写成每秒零件检测次数。Flash 固化、跨断电参考持久化、任意字体／工件／光照的可靠性不属于本次已完成验收的结论。
