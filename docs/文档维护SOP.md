# 文档维护 SOP（可执行）

状态：`active`

## 1. 目标

控制文档数量、降低重复内容、保持入口清晰。

---

## 2. 主文档清单（只维护这几份）

- `README.md`
- `对外接口文档.md`
- `部署与启动.md`
- `联调与验收.md`
- `故障排查.md`
- `架构与设计.md`
- `文档维护SOP.md`

---

## 3. 新增内容决策规则

新增内容前，先判断归属：

1. 外部系统怎么接：写入 `对外接口文档.md`
2. 怎么拉起系统：写入 `部署与启动.md`
3. 怎么联调判定成功：写入 `联调与验收.md`
4. 出问题怎么定位：写入 `故障排查.md`
5. 为什么这么设计：写入 `架构与设计.md`

只有在“专题明显独立且长期维护”时才允许新建 md。

---

## 4. 归档规则

满足任意条件可归档为 `archive`：

- 内容被主文档吸收后只剩历史价值
- 属于阶段性结论（核对、复盘、一次性说明）
- 与主文档重复超过 50%

归档时在文档开头写明：

- 状态：`archive`
- 主入口：指向对应主文档

---

## 5. 每次改文档的执行步骤（精确到新开终端）

### 5.1 新开终端 A：进入工作区并查看变更

```bash
cd /home/raybot/raybot_chassis_ws
git status --short
```

### 5.2 新开终端 B：本地预览（可选）

如果你使用 markdown 预览插件，可直接在 IDE 预览；命令行无需额外服务。

### 5.3 新开终端 C：术语一致性检查

```bash
cd /home/raybot/raybot_chassis_ws
rg "task_state.*cmd" docs
rg "task_cmd" docs
rg "/raybot/base_pose|base_pose_topic|pose_timeout_sec" docs
```

### 5.4 新开终端 D：目录入口检查

```bash
cd /home/raybot/raybot_chassis_ws
rg "^# " docs/README.md docs/对外接口文档.md docs/部署与启动.md docs/联调与验收.md docs/故障排查.md docs/架构与设计.md
```

### 5.5 收尾

```bash
cd /home/raybot/raybot_chassis_ws
git status --short
```

---

## 6. 版本节奏建议

- 每次功能变更：同步更新主文档
- 每周一次：清理重复章节
- 每月一次：复核 `README.md` 导航是否仍准确
