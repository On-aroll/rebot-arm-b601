# 多姿态数据的渲染后端差异与修复

2026-10-21 开发诊断发现：`observation-images-01` 合并的原HOME图像来自EGL，而新增多姿态图像来自GLFW。原轮次及模型保留，当前正式候选仍按冻结设计完成测试；不能把旧图像验证的变化单独归因于机器人姿态。

## 证据

MuJoCo在导入时读取`MUJOCO_GL`并选择`GLContext`。`sim.py`在导入MuJoCo之前设置默认EGL；`observation_dataset.py`先导入MuJoCo，未在进程启动时显式设置后端时会先选GLFW。此时之后设置环境变量不能切换已选中的上下文。

同样的场景seed、机器人初始关节、相机标定，分别在两个明确指定的后端重渲染：

| 原图 | EGL RGB差异元素 | GLFW RGB差异元素 | 非匹配后端最大深度差 |
| --- | ---: | ---: | ---: |
| legacy_HOME，seed5200073，图像73 | 0 | 26,352 | 0.456054m |
| 新HOME，seed6400001，图像259 | 26,225 | 0 | 0.456054m |
| 新观察位0，同seed，图像260 | 26,080 | 0 | 0.539574m |
| 新观察位1，同seed，图像261 | 25,135 | 0 | 0.496130m |

匹配后端的深度数组也逐元素完全相同。最大深度差为整幅图的值，不能把它解释成方块定位误差。导入的上下文实际模块分别为`mujoco.egl`和`mujoco.glfw`。本地MuJoCo源码证据位于`.venv/lib/python3.12/site-packages/mujoco/rendering/classic/gl_context.py`。

最初 `low-coverage-development-01` 未显式指定后端，试图复现第一张EGL原图时得到不同图像和漏检，精确初始预测核对失败后终止。该失败运行保留，不能作为抓取性能证据。随后按原图域分成EGL和GLFW两次开发探测，要求每次实际初始预测精确匹配原始图像推理。

## 修复与下一候选

保留原混合数据，使用相同训练/验证种子和机器人初始姿态，显式在进程启动前设置`MUJOCO_GL=egl`，重新生成 `outputs/observation-images-egl-01`。同一场景始终留在原训练或验证划分；这次重生成不作为新独立任务测试。

- 训练1,792张，1,280个场景：新256场景×3姿态+原1,024张。
- 验证640张，384个场景：新128场景×3姿态+原256张。
- 训练SHA256：`19bc570f868fef38635ed29984e12b1d7eaa971bd307c80ba46ae2bea798a355`
- 验证SHA256：`ad6f98964257dc21214db033e22dfcd95090da74458e4f403d3d7c4fa3647eb2`
- `render_backend.json`记录实际`mujoco.egl`上下文。

`render_backend_audit.py`已用保存的seed和机器人初始关节重新渲染全部768+384张新增图像，RGB和深度逐元素一致；标签体积、空标签、训练/验证种子和旧数据前缀核对也通过。新增标签体积核对65,866个像素，保留完全遮挡的正样本（训练20张、验证11张）。

模型 `vision-multiview-egl-01` 从保留V1视觉权重训练20epoch，按既有验证排序选epoch15：在同一统一EGL验证集上，V1漏检47/546，新模型29/546，空图均0/94误检。原HOME漏检11→4、新HOME5→2、观察位0为13→10、观察位1为18→13。检出集合不同，条件平均误差不作直接精度比较。

匹配残差 `goal-policy-multiview-egl-01` 也已完成，1,468个可用训练样本，517个验证样本，300epoch选293。同一517样本条件平均误差1.674→1.424mm，P955.320→5.137mm；仍需新物理任务验证。

- 视觉SHA256：`45409a61d623c21781f6ce094a5f710ef9beb2619b0160fc58d36dfdcd3301ab`
- 残差SHA256：`a876203d0c85e77138eb933b4daa05093a64f632683b9c5e7f226b2839968f1a`

新模型尚未采用。下一候选还将验证覆盖率开发参数，见 [物理诊断](LOW_COVERAGE_REPORT.md)。

```bash
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python observation_dataset.py --name my-egl-images --train-scenes 256 --validation-scenes 128 --train-seed 6300000 --validation-seed 6400000
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python render_backend_audit.py --run my-egl-images
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python observation_data_audit.py --run my-egl-images
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python vision_runner.py --name my-egl-vision --samples 1792 --epochs 20 --initial outputs/vision-iteration-03/perception.pt --reuse-data outputs/my-egl-images --training-seed 20261023
```

前轮250次正式测试和审计完成后，已修复`observation_dataset.py`、`feedback_audit.py`的导入顺序，通过`sim`先初始化默认渲染环境；`create_run`的后续provenance增加MuJoCo版本、实际GLContext模块和请求环境值。旧轮次通过归档精确源码审计。

所有后续训练、开发、正式任务和复现命令显式指定同一渲染后端。若要把不同后端作为域随机化，应预先声明、标记各域和分层评估，不能混入后再解释为单一因素改进。下一候选 `multiview-egl-development-02` 已按统一EGL启动。
