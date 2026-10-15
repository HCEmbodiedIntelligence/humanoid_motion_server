# A motion 旧基线差异审计

旧 HC pin：63d7609d7c759325c4abe8d755840866d59ec161；来源 C/build/C/motion-reference.git reference/h0-motion，从官方远端取回。比较对象为 A 初次交付 df2188eb80cdb3630866b3bd467cbda2ffda8411；完整文本差异 A-motion-baseline.diff 保留原始检查结果。此差异包含外层7ee7893已有改进及A权限改动，不能全部算作A新增。

## 能力与兼容结论

| 范围 | 核查与处置 |
| --- | --- |
| MoveJ / MoveL / MoveP、ServoJ / ServoP | 五种入口、SDK执行和最终RTC保留。MoveP waypoint resolver、运动单位转换仍保留；现有pipeline和SDK测试覆盖。 |
| 仲裁、停止、到达判定 | 原优先级/关节集合互斥保留；增加按有效采样时间仲裁、过期不得抢占、Move不自动恢复、反馈稳定才成功。 |
| 节点实时路径 | 保留端点配置、FK；改为最新反馈/Servo缓存，输入回调不等待IK，控制周期过大时停止/重建；明确源时间校验，FK保留原测量stamp。 |
| 发布安全 | 新增单次publication token、采样/求解/发布时效及互斥撤销；A增加HC限时许可和epoch，独立运行未配置permit时保持原入口。 |
| SDK边界 | 三个.so、全部SDK headers和SHA256SUMS与旧pin逐字节一致。MANIFEST仅改变依赖布局描述。外层新增限位边界恢复和teleop_posture选项，不替换SDK算法。 |
| C++扩展接口 | 原submitMove/updateServo调用形式通过默认参数/重载保留；增加rebaseFinalJointTarget接口及JointCommand字段。外部自定义ISdkMotionBackend需要实现新方法并重编译，不宣称C++二进制ABI兼容。 |
| SDK加载与安装 | 原可配置deps prefix改成校验/usr/local一致依赖及进程RPATH；hpp-fcl从旧2.4.5改为外层已验证2.4.4，其他精确版本见FindRoboManipDependencies.cmake。旧HC patch不能继续套用。SDK散列、ldd、加载与CMake来源测试通过。 |
| 机器人接入 | 差异唯一删除的两文件是launch/registered_robot.launch.py和docs/registering_robot.md；已从旧对象原样恢复，原文件SHA核对一致。作为可选兼容，不加入日常HC启动。 |
| 原生兼容依赖 | 注册式入口的start_teleop=true仍需另外安装teleop_vr_recv；使用start_teleop=false可只启driver/motion。保持该旧包可选，不将它加回日常HC强制依赖。 |
| 新增能力 | Python pose运行器、Meshcat/仿真、键盘Servo、调试工具及原生示例来自外层版本，全部保留并安装。 |
| interfaces | HC gitlink始终为49b6f791dc1d824350def9260d8cb4841c12a5c1，远端HEAD亦相同；action/msg契约未改。 |

验证：旧pin所有文件在最终目录均存在；SDK二进制/头文件散列无差异；恢复的两文件与旧对象完全一致。恢复launch可导入，generate_launch_description产生6个延迟动作，无节点实际启动。CTest覆盖24项目，包含Move三类/Servo两类、RTC不能绕过、限位恢复、源时间、慢求解失效、取消/超时、双臂并行、SDK ABI及HC许可ROS测试；前次结果见A主报告。

该审计只证明代码保全及已执行离线验证，不代表旧注册式硬件或真机已经验证。版权文件缺失仍由项目权利人补齐，未创建或替换许可。
