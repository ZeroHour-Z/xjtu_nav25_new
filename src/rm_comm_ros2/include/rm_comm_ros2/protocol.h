// #include "ros/ros.h"
#include <cstdint>
#include <map>
#include <string>

#pragma pack(1)
typedef struct {           // 都使用朴素机器人坐标系,前x,左y,上z
  uint8_t  frame_header;   // 帧头 0x72
  uint8_t  color;          // 机器人颜色（0=RED, 1=BLUE）
  uint8_t  sentry_command; // 命令
  uint8_t  eSentryState;   // 当前状态
  uint8_t  eSentryEvent;   // 事件
  uint16_t hp_remain;      // 剩余生命值
  uint16_t bullet_remain;  // 剩余子弹量
  float    time_remain;    // 剩余时间，单位秒
  float    time_test;
  uint32_t reserve_2 : 16;
  uint32_t reserve_3 : 32;
  uint32_t reserve_4 : 32;
  uint32_t reserve_5 : 32;
  uint32_t reserve_6 : 32;
  uint32_t reserve_7 : 32;
  uint32_t reserve_8 : 32;
  uint32_t reserve_9 : 32;
  uint32_t reserve_10 : 32;
  uint32_t reserve_11 : 32;
  uint32_t reserve_12 : 32;
  uint32_t reserve_13 : 32;
  uint8_t  frame_tail; // 帧尾 0x21
} navCommand_t;
#pragma pack()

#pragma pack(1)
typedef struct {         // 都使用朴素机器人坐标系,前x,左y,上z
  uint8_t  frame_header; // 帧头 0x72
  float    x_speed;      // x 方向速度
  float    y_speed;      // y 方向速度
  float    x_current;    // 当前 x 坐标
  float    y_current;    // 当前 y 坐标
  float    x_target;     // 当前 x 坐标
  float    y_target;     // 当前 y 坐标
  float    yaw_current;  // 当前云台偏航角
  float    yaw_desired;  // 期望云台偏航角
  uint8_t  sentry_region;
  float    time_test;
  uint32_t reserve_2 : 8;
  uint32_t reserve_3 : 32;
  uint32_t reserve_4 : 32;
  uint32_t reserve_5 : 32;
  uint32_t reserve_6 : 32;
  uint32_t reserve_7 : 32;
  uint32_t reserve_8 : 32; // 填充到64字节，保持帧尾为最后一字节
  uint8_t  frame_tail;     // 帧尾 0x4D
} navInfo_t;
#pragma pack()

#if defined(__cplusplus)
static_assert(sizeof(navInfo_t) == 64, "navInfo_t must be 64 bytes");
static_assert(sizeof(navCommand_t) == 64, "navInfo_t must be 64 bytes");
#endif

enum sentry_region {
  local = 0,
  flat,
  hole,
  narrow_lane,
  slope,
  step,
  fluctuate,
};

enum sentry_state_e {
  standby = 0,         // 待命状态
  attack,              // 进攻状态，该状态需视野中出现敌人
  patrol,              // 巡逻状态，固定几个点的巡逻状态
  stationary_defense,  // 原地不动防守状态
  constrained_defense, // 约束防守状态，适用于第一次死亡前，前哨站被击毁后
  error,               // 错误状态，即进入了不该进入的状态转移表
  logic,               // 逻辑状态，在这里面做逻辑处理和强制状态转换
  pursuit,             // 追击状态，此时追击坐标为敌人消失的坐标
  supply, // 补给状态，弹丸打完或血量低下会进入此状态，attack、patrol、free_defense的补给状态
  go_attack_outpost, // 只推前哨站状态
};

enum sentry_event_e {
  none = 0,
  match_begin,                       // 比赛开始
  turn_off_auto,                     // 关自动模式
  turn_on_auto,                      // 开自动模式
  target_detected,                   // 目标出现
  target_disappear,                  // 目标消失
  being_hit,                         // 受击
  ammunition_capacity_reached_limit, // 发弹量达到上限
  HP_low,                            // 生命值低下
  logic_operate,                     // 逻辑操作，作为debug判断，不会有实际用处
  HP_regeneration,                   // 生命值恢复
  revive,                            // 复活
  outpost_destroyed,                 // 我方前哨站被击毁
  radar_lock_target,                 // 雷达锁定目标
  operator_intervene,                // 云台手干预
};

static const std::map<uint8_t, std::string> state_map = {
    {sentry_state_e::standby, "standby"},
    {sentry_state_e::attack, "attack"},
    {sentry_state_e::patrol, "patrol"},
    {sentry_state_e::stationary_defense, "stationary_defense"},
    {sentry_state_e::constrained_defense, "constrained_defense"},
    {sentry_state_e::error, "error"},
    {sentry_state_e::logic, "logic"},
    {sentry_state_e::pursuit, "pursuit"},
    {sentry_state_e::go_attack_outpost, "go_attack_outpost"}};

static const std::map<uint8_t, std::string> event_map = {
    {sentry_event_e::none, "none"},
    {sentry_event_e::match_begin, "match_begin"},
    {sentry_event_e::turn_off_auto, "turn_off_auto"},
    {sentry_event_e::turn_on_auto, "turn_on_auto"},
    {sentry_event_e::target_detected, "target_detected"},
    {sentry_event_e::target_disappear, "target_disappear"},
    {sentry_event_e::being_hit, "being_hit"},
    {sentry_event_e::ammunition_capacity_reached_limit, "ammunition_capacity_reached_limit"},
    {sentry_event_e::HP_low, "HP_low"},
    {sentry_event_e::logic_operate, "logic_operate"},
    {sentry_event_e::HP_regeneration, "HP_regeneration"},
    {sentry_event_e::revive, "revive"},
    {sentry_event_e::outpost_destroyed, "outpost_destroyed"},
    {sentry_event_e::radar_lock_target, "radar_lock_target"},
    {sentry_event_e::operator_intervene, "operator_intervene"}};