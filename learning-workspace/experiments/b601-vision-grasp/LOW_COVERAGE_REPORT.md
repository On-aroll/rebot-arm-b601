# 低掩码覆盖率的物理开发验证

2026-10-21。此实验复用旧图像验证集，按“存在目标、检出有效、被0.75覆盖率门限拒绝”选出全部31张图像，对应23个场景。它是有选择的开发诊断，含同场景的多姿态，**不作为31个新独立任务测试，也不估计总体成功率**。

## 实际执行

每张图像从相同seed重新reset，在仿真时间0初始化机器人的已知关节姿态，确认未改动物体free-joint；随后全部由真实物理关节运动执行。使用冻结的`vision-multiview-01`和匹配残差，初始RGB-D预测必须精确匹配原图的推理输出。采用原控制，无额外观察门限，禁止重试；A/B/C轮流作为目的地。

| 原图域 | 开发图像 | 不同场景 | 完整成功 | 重试 | 错误成功 |
| --- | ---: | ---: | ---: | ---: | ---: |
| legacy_HOME / EGL | 7 | 7 | 7/7 | 0 | 0 |
| 新HOME/观察位 / GLFW | 24 | 16 | 24/24 | 0 | 0 |

全部覆盖率范围0.2884–0.7267；同一批原始预测中24个位置误差大于5mm。物理任务成功说明当前模拟夹持、摩擦和有界位置校正在这些开发场景能容忍这些估计。它不证明低覆盖率估计普遍准确，或实机也有相同容差。

最初未匹配渲染后端的运行在精确图像核对失败后终止，保留为`low-coverage-development-01`，不纳入上述成绩。[渲染差异诊断](RENDER_BACKEND_FINDING.md)记录了原因及重生成数据的处理。

## 审计

`low-coverage-egl-development-02/audit.json`和`low-coverage-glfw-development-02/audit.json`均通过：31次运行共17,990条动作状态、4,509帧配对RGB-D、62帧精确决策RGB-D、449,363个物理步，0禁止接触。最终评分15,500个物理步独立重算；初始机器人姿态、原图预测、禁止重试和全部预选案例均核对，没有按结果过滤案例。

```bash
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python low_coverage_probe.py --name my-low-coverage-egl --source-domain legacy
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python low_coverage_audit.py --run my-low-coverage-egl
MUJOCO_GL=glfw OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python low_coverage_probe.py --name my-low-coverage-glfw --source-domain new
MUJOCO_GL=glfw OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python low_coverage_audit.py --run my-low-coverage-glfw
```

## 下一候选假设

原0.75门限把可执行的任务拒绝为失败。开发配置`configs/feedback-coverage-development-02.json`将最小覆盖率设为0.25，检验这是否减少无效重试和任务拒绝；该值是实验参数，尚未验证通过。置信度、检测器形状/深度检查、工作区、最大覆盖率、两帧稳定性和最大位置更新限制继续按原配置执行。

后续将此参数与统一EGL的新视觉/匹配残差一起做新的物理开发测试，再用至少30个新场景/块正式验收。当前混合数据候选和原0.75门限的正式250次测试照冻结设计完成，V1保留。
