#pragma once

#include <string>

namespace motion_control {

/// motion_control 各子模块的统一生命周期接口。
class CoreModule {
 public:
  virtual ~CoreModule() = default;

  /// 返回模块名称，便于日志和诊断输出。
  virtual std::string name() const = 0;
  /// 初始化模块资源；默认无额外配置。
  virtual bool configure() { return true; }
  /// 启动模块；默认无额外动作。
  virtual bool start() { return true; }
  /// 停止模块；默认无额外动作。
  virtual void stop() {}
};

}  // 命名空间 motion_control
