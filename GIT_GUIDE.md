# Git 常用指令速查

## 工作流 (三板斧)

```bash
# 1. 看改了啥
git status

# 2. 添加改动
git add -A              # 添加所有改动
git add main/app_main.cpp   # 只添加某个文件

# 3. 提交
git commit -m "修复了人脸检测阈值问题"

# 4. 推送
git push
```

## 日常会用到的

```bash
# 看提交历史
git log --oneline -10        # 最近10条,一行一条

# 看某个文件的改动
git diff                     # 还没add的改动
git diff --staged            # 已经add的改动

# 撤销操作
git restore 文件名            # 撤销还没add的修改(恢复原样)
git restore --staged 文件名   # 取消add(回到未暂存)
git reset --soft HEAD~1      # 撤销最近1次commit,改动保留

# 分支
git branch                   # 看当前分支
git checkout -b 新分支名      # 创建并切换到新分支
git checkout main            # 切回main

# 从GitHub拉取最新
git pull

# 查看某个提交的作者和时间
git log -1
```

## 典型日常流程

```bash
写代码 → git status → git add -A → git commit -m "xxx" → git push
```

## 提交信息规范

```bash
# 好的提交信息 (英文动词开头)
git commit -m "fix: 修复人脸检测阈值过高导致漏检"
git commit -m "feat: 接入害虫检测双模型切换"
git commit -m "refactor: 把摄像头初始化抽成独立函数"
git commit -m "docs: 更新README编译说明"

# 前缀约定:
#   feat:  新功能
#   fix:   bug修复
#   refactor: 重构(不改功能)
#   docs:  文档
#   chore: 杂项配置
```

---

# 哪些上传, 哪些不上传

## esp32p4c5_edgeAI_lot (已配好)

| ✅ 上传 | ❌ 不上传 (已在.gitignore) |
|---------|---------------------------|
| `main/*.cpp` `main/*.hpp` 源代码 | `build/` 编译产物 |
| `components/` BSP组件 | `managed_components/` 自动下载的依赖 |
| `CMakeLists.txt` 构建配置 | `.omc/` 状态文件 |
| `partitions.csv` 分区表 | |
| `sdkconfig` `sdkconfig.defaults` | |
| `scripts/*.bat` 编译烧录脚本 | |
| `dependencies.lock` 依赖锁 | |
| `README.md` `PROJECT_OVERVIEW.md` | |

## PestYOLO (Python项目)

| ✅ 上传 | ❌ 不上传 |
|---------|-----------|
| `*.py` 源代码 | `__pycache__/` `.pyc` |
| `requirements.txt` 依赖 | `.venv/` `venv/` 虚拟环境 |
| `*.ipynb` Jupyter | `datasets/` 图片数据集 |
| `main.py` `app.py` | `*.pt` 大模型文件(>50MB建议Git LFS) |
| `data.yaml` 配置 | `runs/` 训练输出 |
| | `.DS_Store` `Thumbs.db` |

## 关键原则

```
能通过编译/脚本重新生成的 → 不上传
手写的源代码和配置      → 必须上传
第三方库/依赖           → 不上传 (通过包管理器装)
大文件(>100MB)         → 不上传, 用 Git LFS 或网盘
密码/密钥/Token        → 绝对不上传!
```
