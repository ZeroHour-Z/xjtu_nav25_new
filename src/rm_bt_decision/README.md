# rm_bt_decision

ROS 2 (Humble) 行为树框架，基于 `py_trees`，支持 YAML/ Python 直写树，适配 Nav2 `NavigateToPose` 动作，支持 web 可视化（`py_trees_js`）。

## 运行

```
colcon build --packages-select rm_bt_decision
. install/setup.bash
ros2 launch rm_bt_decision bt.launch.py
```

## 参数
- `tree`: 行为树 YAML 路径（默认安装内置示例）。
- `tick_hz`: tick 频率（默认 10 Hz）。
- `use_web_viewer`: 是否启用 py_trees_ros web 可视化（需要 `py_trees_js`）。

## 编写树（YAML）
参见 `config/trees/nav.yaml`。

## Python 直写树
可以直接 import 行为并在 Python 里构建树，再传给 `py_trees_ros.trees.BehaviourTree` 或 `py_trees.trees.BehaviourTree`。

---

## 新增行为/动作（指南）

下面以新增一个自定义动作为例，说明从代码到 YAML 的完整流程。

1. 创建文件：在 `rm_bt_decision/actions/` 内新建 Python 文件，例如 `my_action.py`。
2. 实现类并注册：使用 `@register("YourActionType")` 装饰器注册到行为注册表。

```python
from rclpy.node import Node
import py_trees
from ..registry import register

@register("MyAction")
class MyAction(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node, foo: str = "bar"):
        super().__init__(name)
        self.node = node
        self.foo = foo

    def setup(self, **kwargs):
        pass

    def initialise(self):
        pass

    def update(self) -> py_trees.common.Status:
        # 返回 SUCCESS / FAILURE / RUNNING
        return py_trees.common.Status.SUCCESS
```

3. 暴露模块：在 `rm_bt_decision/actions/__init__.py` 中 import 该类，并加入 `__all__`，以便加载器在导入 `actions` 包时完成注册。

```python
from .my_action import MyAction
__all__ = [
    # ... 其他
    "MyAction",
]
```

4. 在 YAML 中使用：

```yaml
- type: MyAction
  name: 示例
  params:
    foo: "baz"
```

5. 运行：重建并启动。

```
colcon build --packages-select rm_bt_decision
. install/setup.bash
ros2 launch rm_bt_decision bt.launch.py
```

### 参数传递与声明约定

- 加载器会将 YAML 节点 `params` 以 `**params` 的形式传入构造函数。
- 如行为需要 ROS 参数（`rclcpp/rclpy` 参数服务器）持久化配置，建议在构造函数或 `setup()` 中：
  - 使用 `node.has_parameter` 检查并 `declare_parameter` 默认值；
  - 使用 `node.get_parameter(name).value` 读取运行时值；
  - 对需要全局统一管理的配置，建议放到命名空间（例如 `supply.*`）。

---

## 补给管理（SupplyManagerAction）配置方式变更

`SupplyManagerAction` 现已改为使用内置 ROS 参数，而非在 YAML 中内联传参。你可以通过 ROS 参数动态配置，无需修改树文件。

- 命名空间：`supply.*`
- 关键参数：
  - `supply.server_name`（string，默认 `navigate_to_pose`）
  - `supply.cancel_on_terminate`（bool，默认 `true`）
  - `supply.nav_timeout_s`（double，可空，默认 `120.0`）
  - `supply.hp_param`（string，默认 `hp`）
  - `supply.ammo_param`（string，默认 `ammo`）
  - `supply.hp_low`、`supply.hp_high`（double，默认 `200.1`、`399.9`）
  - `supply.ammo_low`、`supply.ammo_high`（double，默认 `10.1`、`99.9`）
  - `supply.frame_id`（string，默认 `map`）
  - `supply.heal_x`、`supply.heal_y`、`supply.heal_yaw`（double，默认 `-4.0, -6.0, 0.0`）
  - `supply.reload_x`、`supply.reload_y`、`supply.reload_yaw`（double，默认 `-4.0, -6.0, 0.0`）

YAML 中对应节点应简化为：

```yaml
- type: SupplyManagerAction
  name: 补给管理
```

运行时示例：

```
ros2 param set /rm_bt_decision_node supply.hp_low 180.0
ros2 param set /rm_bt_decision_node supply.reload_x -3.5
``` 