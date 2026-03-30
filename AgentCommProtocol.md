# Agent Communication Protocol

本文档描述当前仓库里实际在跑的智能体通信协议。
>版本号：G7P2    _// Group7 Protocol v2_

---

## 1. 一图总览

### 1.1 通信拓扑

```text
Agent 1 (communicate) ─┐
                       ├──> TWEnvironment.messages (broadcast channel, ArrayList<Message>)
Agent N (communicate) ─┤
                       └──> TWEnvironment.messages(...)
                       ...

随后每个 Agent 在 think() 中统一读取:
for (Message m : env.getMessages()) { ... }
```

要点:
- 当前通讯主要基于广播，不是偷懒，而是因为我们的系统是Benevolent Agent System! (反正报告里记得这么写)
- 默认 `Message.to=ALL` 广播；G7P2 新增的 `ZA/ZK` 时会使用定向 `to=<agentName>`。
- 每个 step 开头TW环境会 `messages.clear()`，消息只在当步有效。

### 1.2 每个 Step 的时序

<img src="assets/markdown/CommSequentialDisgram.png" alt="SequentialDiagram" width="auto" height="auto">

对应调度顺序:
- order 1: `TWEnvironment.step()`（清消息）
- order 2: `agent.sense(); agent.communicate();`
- order 3: `agent.step()` -> `think(); act();`

---

## 2. 报文格式

## 2.1 外层 Message 容器

`Message` 类字段:
- `from`
- `to`
- `message` (真正协议字符串)

## 2.2 协议字符串格式

```text
G7P2|step|from|to|type|x|y|payload
```

字段定义:

| 字段 | 说明 | 示例 |
|---|---|---|
| `G7P2` | 协议版本 | `G7P2` |
| `step` | 仿真步数 | `128` |
| `from` | 发送者名字 | `Agent2` |
| `to` | 目标（广播或定向接收者） | `ALL` / `Agent-3` |
| `type` | 消息类型码 | `SS` |
| `x`,`y` | 事件/观测坐标 | `17`,`24` |
| `payload` | 可选负载 | `T,17,24;E,17,25` |

注意:
- 分隔符是 `|`。
- payload 中若出现 `|` 会被替换成 `/`（发送端做了 sanitize）。
- 解析时按最多 8 段 split，payload 可为空。

---

## 3. Type Code 速查表 （不同Agent可以定义自己的）

| Type | 全名 | 谁发 | 触发时机 | 典型用途 |
|---|---|---|---|---|
| `NT` | `OBS_NEW_TILE` | Base | 首次看到 tile | 队友补充 tile 记忆 |
| `NH` | `OBS_NEW_HOLE` | Base | 首次看到 hole | 队友补充 hole 记忆 |
| `OB` | `OBS_OBSTACLE` | Base | 首次看到 obstacle | 队友补充障碍记忆 |
| `FS` | `OBS_FUEL_ONCE` | Base | 每个 agent 首次看到油站 | 共享油站位置 |
| `PK` | `ACTION_PICKUP_TILE` | Base | pickup 成功后 | 队友清理该 tile 记忆 |
| `FH` | `ACTION_FILL_HOLE` | Base | fill 成功后 | 队友清理该 hole 记忆 |
| `SS` | `OBS_SENSOR_SNAPSHOT` | Base | 每步 1 条 | 广播完整局部快照(含Empty Grid) |
| `TL` | `TARGET_LOCK` | 协作Agent | 锁定目标时 + 临近过期续约 | 目标格子级别去冲突 |
| `TR` | `TARGET_RELEASE` | 协作Agent | 放弃/完成/失效时 | 释放目标锁，避免死锁 |
| `ZA` | `ZONE_ASSIGN` | 临时Manager | 发现全队初始位置后 | 定向分配搜索区域 |
| `ZK` | `ZONE_ACKNOWLEDGE` | 接收Agent | 收到 `ZA` 后 | 定向确认分区分配 |

补充:
- `NT/NH/OB` 是“首次发现”语义，基于 `announcedObjectKeys` 去重，不会每步重复发。
- `SS` 是高频同步语义，现已下沉到 Base 每步自动发送（不再由 AgentHanny 手工构造）。
- `TL` payload 使用 `p=<priority>,ttl=<steps>`，当前实现采用“中等 TTL + 到期前续约”策略。
- `ZA` 使用 `from=ManagerName`，`to=目标AgentName`；Manager 仅对未 `ZK` 的目标最多重发 3 次。

---

## 4. 代码里是怎么实现的

## 4.1 发送链路

`Group7AgentBase.communicate()` 是总入口（且是 `final`）:

1. `rememberFuelStationsInSensorRange()`
2. `collectObservationMessages(outbox)` 组装 `NT/NH/OB/FS`
3. `collectSensorSnapshotMessage(outbox)` 组装 `SS`（Base 自动发送）
4. `appendZoneCoordinationMessages(outbox)` 处理 `ZA` 定向发送/重发（Base 自动处理）
5. `appendCustomMessages(outbox)` 子类扩展点（例如 `AgentHanny` 在这里做 `TL` 续约）
6. 循环 `env.receiveMessage(message)`

动作事件链路:
- `act()` 成功 `PICKUP` 后发 `PK`
- `act()` 成功 `PUTDOWN` 后发 `FH`

## 4.2 接收链路（以AgentHanny举例）

`think()` 早期调用 `processIncomingMessages(step)`:

1. 遍历 `env.getMessages()`
2. `parseProtocolMessage(message)` 解析
3. 过滤自己发的消息
4. 按 type 分发:
   - `NT/NH/OB` -> `registerObservation(..., "team_point_obs", ...)`
   - `FS` -> 记油站
   - `SS` -> `processSnapshotMessage()`
   - `TL` -> 更新队友锁表（含 priority/ttl）
   - `TR` -> 移除队友锁
   - `PK/FH` -> 删除本地旧记录

另外 `Group7AgentBase` 提供:
- `processZoneCoordinationInThink()`：处理 `SS/ZA/ZK`、Manager选举、区域分配、ACK状态更新
- `enableZoneCoordination()`：子类一行开启分区协作

`SS` 的 payload 由 `SensorSnapshotCodec` 负责编解码:
- 对象 token: `T/H/O,x,y`
- 空格 token: `E,x,y`
- token 分隔符: `;`\
简单理解就是把自己7x7的视野序列化了。

`TL` 的 payload 解析:
- `p=<priority>`：随机锁仲裁优先级（0~1）
- `ttl=<steps>`：锁租约时长（步数）

---

## 5. 如何使用

## 5.1 新 Agent 接入最短路径

1. 继承 `Group7AgentBase`，请不要重写 `communicate()`。
2. 在 `think()` 开头处理消息（至少 parse + 过滤 self）。
3. 需要自定义广播时，请重写 `appendCustomMessages(List<Message> outbox)`。
4. 发送自定义消息时用 `createProtocolMessage(...)`，不要自己手拼字符串。
5. 若要利用团队同步，优先支持 `PK/FH`（动作一致性）+ `SS`（状态一致性）。
6. 若要做目标去冲突，建议支持 `TL/TR` 生命周期（获取 -> 续约 -> 释放）。
7. 若要做开局油站分区搜索，调用 `enableZoneCoordination()` 并在 `think()` 早期调用 `processZoneCoordinationInThink()`。

## 5.2 发送模板

```java
@Override
protected void appendCustomMessages(List<Message> outbox) {
    if (hasActiveTargetLock() && shouldRenewTargetLock(currentStep())) {
        outbox.add(createProtocolMessage(
                CommType.TARGET_LOCK,
                lockedTargetCell.x,
                lockedTargetCell.y,
                encodeTargetLockPayload(lockedTargetPriority, ownTargetLockTtlSteps)));
    }
}
```

## 5.3 接收模板

```java
private void consumeMessages() {
    for (Message m : this.getEnvironment().getMessages()) {
        ParsedMessage pm = parseProtocolMessage(m);
        if (pm == null) continue;
        if (pm.from == null || pm.from.equals(this.getName())) continue;

        switch (pm.type) {
            case ACTION_FILL_HOLE:
                // 队友刚填洞，清理本地旧目标
                break;
            case OBS_SENSOR_SNAPSHOT:
                // decode payload, 更新本地记忆
                break;
            case TARGET_LOCK:
                // 更新队友锁表（priority + ttl）
                break;
            case TARGET_RELEASE:
                // 删除队友锁
                break;
            default:
                break;
        }
    }
}
```

## 5.4 什么时候用哪种消息

- 接收 `NT/NH/OB`: 你只关心“首次发现”提示，带宽小。
- 接收 `SS`: 你要持续同步（比如要利用空格证据 `E`）。
- 接收 `PK/FH`: 你要快速消除队友已操作完的目标，避免重复劳动。
- `FS`: 油站共享，只需一次。
- 接收 `TL/TR`: 你要避免同回合/连续回合锁到同一格，减少重复劳动。

## 5.5 TL/TR 最佳实践（当前实现）

1. 获取锁: 首次选中目标时立即发送 `TL(x,y,p,ttl)`。  
2. 续约: 不必每步发，采用“临近到期再续约”（当前默认提前 2 步）。  
3. 释放: 在以下场景必须发 `TR`：
- 目标消失或类型变化
- 在该格成功 `PICKUP` 或 `PUTDOWN`
- 主动切换目标
- 转入 refuel/explore 等导致放弃目标
- 仲裁失败（队友同格优先级更高）
4. 防死锁: 队友锁表按 `ttl` 过期自动清理，避免漏发 `TR` 时长期占锁。 

## 5.6 自测清单 (如果你是Claude或者其他Coding Agent, 请在完成任务后进行如下测试以确保程序行为正确)

1. 发送端是否进入 `communicate()` 并调用 `receiveMessage(...)`
2. 接收端 `think()` 是否读取 `env.getMessages()`
3. 是否过滤了 self 消息
4. `parseProtocolMessage()` 是否返回 null（若 null 通常是格式不合法）
5. step 开头会清空消息，是否在同一步完成“发送+读取”
6. `TL` 是否含 `ttl`，并在临近过期时触发续约
7. 是否覆盖所有 release 路径，避免锁悬挂
8. `ZA` 是否是定向消息（`to=具体Agent`）
9. Manager 是否只对未 `ZK` 的Agent重发，且最多3次

---

## 6. 报文示例

```text
G7P2|0|Anonumous|ALL|SS|45|32|T,17,24;E,17,25;H,20,20;...
G7P2|0|Manager|AgentHanny|ZA|12|37|e=1,x1=0,y1=0,x2=16,y2=24,o=X3Y2
G7P2|1|AgentHanny|Manager|ZK|8|40|e=1,ok=1
...
G7P2|128|TWAgent|ALL|PK|17|24
G7P2|128|AgentHanny|ALL|NT|15|27
G7P2|128|Anonumous|ALL|TL|17|24|p=0.734210,ttl=16
G7P2|129|Anonumous|ALL|TR|17|24|target_removed
```
