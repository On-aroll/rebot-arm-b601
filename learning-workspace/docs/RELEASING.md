# GitHub 维护与发布

## 仓库结构

此目录应作为独立部署仓库的根目录。若合并到已有仿真仓库，应明确使用子目录，并调整 Actions 的工作目录和可复用 workflow 路径；不能直接覆盖原仓库 README、许可证或其他项目文件。

## 首次发布

1. 把本目录代码提交到选定的 GitHub 仓库，不上传 `.venv/`、`.tools/`、`config.json`、数据、模型或日志。
2. 查看 `Linux software checks` 在 Ubuntu 22.04 / 24.04 上的执行结果。
3. 在 Actions 中选择 `Publish Linux Release` → `Run workflow`，选择要发布的分支。也可推送与 VERSION 一致的标签，例如 `v0.1.0`。
4. 发布任务会先运行 CPU 安装和真实 ACT 自检，再上传版本归档、单包校验文件、`SHA256SUMS` 和 `install-release.py`。
5. 使用 Release 的实际下载链接在另一台目标电脑验证安装，再更新 VERIFICATION.md 的验证记录。

工作流只使用 GitHub 内置的 `GITHUB_TOKEN`，发布 job 申请 `contents: write`；不需要把个人令牌写进仓库。同版本 Release 已存在时工作流失败，不替换已有资产。私有仓库的 Actions 用量受账号配额限制。

## 用户安装入口

公开仓库发布后，将下面的 `OWNER/REPO` 替换为实际仓库名：

```bash
curl -fsSL https://github.com/OWNER/REPO/releases/latest/download/install-release.py | python3 - --repo OWNER/REPO
```

指定版本、GPU 或安装目录：

```bash
python3 install-release.py --repo OWNER/REPO --version v0.1.0 --device cuda --prefix "$HOME/Robotics"
```

Release 下载器要求系统 Python 3.10+。首次下载的脚本作为安装入口执行；它通过 HTTPS 下载 Release 元数据和附件，再核对归档 SHA256。校验和与资产存放在同一仓库，用于发现损坏或错包，不等于独立的发布者签名。

私有仓库用户须先获得仓库访问权，例如使用已登录的 GitHub CLI 下载脚本：

```bash
gh release download --repo OWNER/REPO --pattern install-release.py --dir /tmp/seeed-release
GH_TOKEN="$(gh auth token)" python3 /tmp/seeed-release/install-release.py --repo OWNER/REPO
```

不要把访问令牌写进脚本或提交到 Git。公开发布前也应确认仓库的可见性符合维护者意图。

## 升级与旧版本

重复执行同一安装命令会解析最新 Release。默认位置为：

```text
~/.local/share/seeed-lerobot/
  releases/v0.1.0/        # 该版本代码、独立 Python 环境
  shared/config.json     # 各版本共享的个人配置
  shared/data/            # 数据、模型、评估
  shared/logs/
  shared/.state/
  current -> releases/v0.1.0
```

新版本安装及自检成功后才切换 `current`。失败的新版本目录保留用于排错，旧版本入口保持可用；重新安装同一版本会同步其现有环境，失败时可能需要重跑。升级不会自动迁移数据格式；后续如有不兼容变更，须在更新记录中说明并先备份 shared/。不要在录制或训练进行中升级。

启动：`~/.local/share/seeed-lerobot/current/robotctl`。已安装的旧版本可直接通过 `releases/v旧版本/robotctl` 启动；它们使用同一份 shared/ 数据，须留意跨版本兼容性。

## 后续版本

修改 VERSION、CHANGELOG.md、必要的锁文件和测试，提交 PR；CI 通过后合并并发布对应标签。更新第三方源码时同步 wheel、ORIGIN.json、许可证和 CPU/CUDA 锁文件，运行实际软件自检。硬件测试结果单独记录，不能由 CI 软件通过推断。
