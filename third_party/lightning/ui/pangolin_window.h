// 空 UI stub：lightning-lm 建图内核里 ui_ 恒为 nullptr（从不 SetUI），
// 这里只提供编译所需的类型声明，任何调用都不会执行。
#pragma once
#include <memory>
#include "common/nav_state.h"
#include "common/keyframe.h"

namespace lightning {
namespace ui {
class PangolinWindow {
   public:
    void UpdateNavState(const NavState&) {}
    void UpdateScan(CloudPtr, const SE3&) {}
    void UpdateKF(const Keyframe::Ptr&) {}
    void Quit() {}
};
}  // namespace ui
}  // namespace lightning
