## Agent 1 - Agent 1
第一版本：核心功能增加与优化总结
本版本在不改变底层框架（environment, sensor 等）的前提下，完整实现了 AgentEnge 的自主决策体系：

1. 行为决策状态机 (Priority-based Logic)
构建了四层优先级的任务处理逻辑：

🚨 紧急求生：燃料 < 180 时强制切换至补给模式，支持记忆检索与路径重规划。

🕳️ 高效填洞：持有 Tile 时优先寻路至最近已知洞口，具备失效检测与自动清理机制。

📦 智能拾取：在负载范围内（< 3）自动规划路径拾取视野内资源。

🔍 策略探索：采用 7步长犁地式 (Boustrophedon) 扫描算法，并限制 y≤46 以优化能耗（后期要改，”自适应“）。

2. 核心算法集成
A 路径规划*：基于曼哈顿距离的启发式搜索，完美处理静态与记忆障碍物。

三级重规划 (Re-planning)：当路径失效或成本（燃料/时间）过高时，支持最多 3 次动态局部重算。


3. 记忆与感知增强
对象手动持久化：解决了 FuelStation 无法自动存入记忆的限制。

动态环境适应：实时验证 Hole 存在性，对象消失后自动更新记忆并切换下一个目标。

4. 鲁棒性与调试
全流程日志：分类记录感知、模式切换、路径规划及动作结果，极大提升调试效率。

边界约束：严格遵守 y 轴边界限制

checkcheck。。。。。。
## Group7AgentBase 基类（随时可override）
Group7AgentBase 是给组内 Agent 复用的“策略脚手架”。
各自的只要专注于 `think()` 决策，其他通用能力（路径、通信、动作执行、基础生存）都能直接复用。

### 0. 每一步发生了啥
在当前实现里，一个 tick 的执行顺序是：
1. Environment 先刷新对象并 `messages.clear()`
2. 每个 Agent 执行 `sense()` + `communicate()`（这里 Group7AgentBase 会自动发基础协议消息）
3. 每个 Agent 执行 `think()` + `act()`

所以你在 `think()` 里读到的是“本步新收到的消息”；消息不会跨步自动保留。

### 1. Base 当前已经有这些：
1. 路径规划与移动
- `planPath()`：A*（曼哈顿）
- `stepToward(target)`：自动复用/重算路径并给出下一步移动
- `clearPlan()`：切换目标时可主动清理旧路径

2. 探索与阻挡判断
- `exploreZigZag()`：默认低开销探索
- `isBlocked(x, y)`：统一考虑环境障碍 + 记忆障碍

3. 燃料与油站
- `rememberFuelStationsInSensorRange()`：把油站写入记忆（很重要）
- `findFuelStationInMemory()`
- `shouldRefuel(safetyMargin)`：按“到油站距离 + 裕量”判定是否回补

4. 通信协议（默认开启）
- 协议格式：`G7P1|step|from|to|type|x|y|payload`
- 自动广播：新发现 Tile/Hole/Obstacle、FuelStation(一次)、成功 Pickup/Fill 事件
- 可用 `parseProtocolMessage()` 解析消息
- 可重写 `appendCustomMessages(outbox)` 追加你自己的消息

5. 动作执行
- 基类 `act()` 已安全处理 `MOVE/PICKUP/PUTDOWN/REFUEL` 与 blocked 异常
- 若你要自定义 `act()`（例如打日志），建议 `super.act(thought)` 继续走基类逻辑

### 2. Example A：切换目标时避免还在跟着旧的‘缺德地图’走
当策略从“去 tile”突然切到“去油站”时，建议先清计划：
```java
if (shouldRefuel(FUEL_MARGIN)) {
    clearPlan();  // 避免沿着旧目标继续走
    TWFuelStation fs = findFuelStationInMemory();
    if (fs != null) return stepToward(new Int2D(fs.getX(), fs.getY()));
}
```

### 3. Example B：追加自定义消息（由基类定协议，各自写消息）
```java
@Override
protected void appendCustomMessages(List<Message> outbox) {
    String payload = "mode=explore,fuel=" + (int) this.getFuelLevel();
    outbox.add(createProtocolMessage(CommType.OBS_SENSOR_SNAPSHOT, this.getX(), this.getY(), payload));
}
```
读取队友消息：
```java
private void consumeMessages() {
    for (Message m : this.getEnvironment().getMessages()) {
        ParsedMessage pm = parseProtocolMessage(m);
        if (pm == null || this.getName().equals(pm.from)) continue;
        if (pm.type == CommType.ACTION_FILL_HOLE) {
            // 队友刚填洞，可清理自己的旧目标
            clearPlan();
        }
    }
}
```

### 4. 基类使用小贴士
1. 最好不要重写 `communicate()`
- Group7AgentBase 已经把它设成 `final`，自定义消息请用 `appendCustomMessages()`

2. 忘记记忆油站
- 只靠默认感知，油站不一定进入工作记忆；建议每步先调用 `rememberFuelStationsInSensorRange()`，更安全~

3. 只改 `think()` 不处理目标切换
- 策略优先级变化时记得 `clearPlan()`，不然可能会出现'脚没跟上脑子'的情况

4. 读消息时没过滤自己
- 处理消息时建议 `if (pm.from.equals(getName())) continue;`

5. override `act()` 后没调用 `super`
- 会丢失基类的动作执行与事件广播逻辑（球球了，有些广播消息对模型学习很重要QAQ）


## Agent 2 - AgentHanny
AgentHanny 是在 Group7AgentBase 上实现的策略代理，当前正在测试我组会上提出的生存模型，Agent主打一个“蝗虫过境”。\
SimpleTWAgent: 我也要吃吗？ <img src="assets/markdown/doialso.jpg" alt="MemeImg" width="54" height="54"> \
AgentHanny: 事已至此，先建模再召回再更新梯度再选择计划再执行再干饭吧。

1. 思考阶段与目标优先级
- 思考时先同步全局Memory再想怎么吃饭。
- 然后更新梯度，更新完了想想怎么吃饭。
- 快耗尽燃料优先找猫粮并吃饭。
- 低燃料优先找猫粮并吃饭。
- 当前格子可以捡东西/放东西，做完了再看去哪里吃饭。
- 背着tile的时候召回并精排找hole，找好了再吃饭。
- 背包未满的时候召回并精排找tile，找到了再吃饭。
- 无目标时之字形探索，探索累了吃饭。

2. 记忆侧卡（MemorySideCard）与全同步（像不像学英语时候边走路边拿个单词小卡片）
- 新增 MemorySideCard，补足原 WorkingMemory 缺失字段：`firstSeen`、`lastSeen`、`disappearAt`、`spawnObserved`。
- AgentHanny 每回合同步自身观测与队友消息到 SideCard，并同步写回 WorkingMemory（新增/更新/删除都保持一致）。
- 对于“上一步该格为空、本步新出现”的对象，标记 `spawnObserved=true`，之后用 `firstSeen + lifetime` 做硬约束判断存活，同时本条数据由于违反指数衰减模型的伯努利分布假设故将不会计入梯度更新。

3. Agent在线大学习（第一次组会提到的指数衰减生存模型）
- 维护三类的hazard（λ）：`hazardTile`、`hazardHole`、`hazardObstacle`。
- 基于“匹配/不匹配”在线SGD更新梯度，并约束“同一对象每个 observation interval（当前为3steps）最多更新一次”。
- 学习与梯度日志写入 `src/tileworld/agent/runtime/<agent name>-model-updates.log`，避免狂刷控制台。

4. 协作消息与每回合记忆刷新 (摁住我的`F5`)
- 在 Base 协议基础上，AgentHanny 每步额外广播一条压缩视野快照（对象+空格）。
- 各 Agent 可据此更新 `lastSeen`、补充缺失对象，并利用“空格证据”判断对象消失或刚生成。
- 拾取 Tile / 填洞动作仍通过 Base 事件广播，方便队友及时清理过期记忆。

5. 召回与懒删除（Recall + Lazy Delete）
- 计划前进行粗筛：`top-20 + 5 buffer`，buffer是给lazy delete兜底用的，以免删了太多召回不了几个选项。
- 对候选按存活概率与距离排序；若 `p_alive < p_theta` 或超期则懒删除记忆并跳过。
- 无可行候选时回退到 Zig-Zag 探索，保证行为连续性与计算开销可控。

6. 收敛稳定性优化（模型震荡了？我不信。）
- 采用“每步按类型聚合梯度、每类只更新一次”：
  - 将同一步内来自多个对象/消息的 matched 与 mismatch 信号先汇总，再更新 `hazardTile/hazardHole/hazardObstacle`。
  - 避免“同一步连续多次更新同一全局 λ”导致的震荡放大。
- 引入学习率调度（Schedule）：
  - 时间衰减（time decay）：随着 step 增加，学习率逐步减小。
  - 事件衰减（event-based decay）：当某类型在同一步内 mismatch 事件偏多时，自动下调该类型 mismatch 学习率；稳定时再缓慢恢复。
- 保留并结合边界稳定机制：
  - `λ` 下/上界裁剪、单步更新幅度裁剪（gradient step clipping）、样本来源权重（self > team）。
- 设计原因：
  - 早期版本中，matched 高频样本易把 λ 快速压到极小值，而后单次 mismatch 又会强烈反弹，出现非收敛振荡。
  - 聚合 + 调度后，更新更平滑、极端跳变显著减少，参数轨迹更接近“缓慢漂移并在区间内稳定”。

7. 多少个‘蒸蚌！’历史统计：
- 实验设置：环境里创建6个同样的‘影分身の術’然后运行5000回合数次，去除运气太背没找到猫粮的情况后取大致均值
- v1 - 活着优先 （在Group7BaseAgent上的最小实现）
  - Final Reward: ~465.
- v2 - 蝗虫过境（指数衰减建模+hazard-aware的贪婪式计划）
  - Final Reward: Para1 ~1200; Para2 ~1400.

8. 下一步这个Agent还应该做些啥
- 目前测试（部署6个AgentHanny时）下来发现性能瓶颈在Agent喜欢在同一个区域工作，导致经常出现重复工作，后续需要如下优化：
  - Agent有自己的工作区动态偏好，与其他Agent相对分开，这样也能提升模型学习数据的IID率并减少oscillation
  - Agent广播自己的当前的目标实体，同时在Planning召回阶段忽略已经被队友的plan锁定的实体
  - 也不要忘了在取消当前plan(比如赶着吃猫粮)时，广播释放当前目标，避免无限锁定
- 当前暂时没有考虑携带多块tiles，而是根据建模选择到达时存活率最高的下一地点
  - 回头加入可断点重排的计算EU(whole-plan)模式并使用更加鲁棒的召回算法来生成初排plan列表（不然它可是NP-Hard问题啊啊）
  - Plan包括捡起{1...available-carry-space}个tile并送到{1...expected-carried-tiles}个hole里
- 等待下一次组会
