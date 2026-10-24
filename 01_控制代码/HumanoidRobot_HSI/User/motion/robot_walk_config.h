#ifndef ROBOT_WALK_CONFIG_H
#define ROBOT_WALK_CONFIG_H

/* 只修改ROBOT_RUN_MODE选择模式；修改后重新编译、下载、复位。 */
#define ROBOT_MODE_ACTION_DEBUG 1U  /* 动作调试：执行ROBOT_DEBUG_MOTION。 */
#define ROBOT_MODE_VISION_DEBUG 2U  /* 图形调试：图形动作一次，然后回站。 */
#define ROBOT_MODE_FULL_RUN     3U  /* 完整流程：巡线纠偏、图形动作与恢复。 */
#define ROBOT_MODE_LINE_DEBUG   4U  /* 巡线调试：先走一批，站稳后纠偏，忽略图形。 */
#ifndef ROBOT_RUN_MODE
#define ROBOT_RUN_MODE ROBOT_MODE_FULL_RUN
#endif
#if (ROBOT_RUN_MODE < ROBOT_MODE_ACTION_DEBUG) || (ROBOT_RUN_MODE > ROBOT_MODE_LINE_DEBUG)
#error "ROBOT_RUN_MODE must be 1, 2, 3 or 4"
#endif

/* 动作名称在motion_groups.h；沿用动作表loop字段决定是否循环。 */
#ifndef ROBOT_DEBUG_MOTION
#define ROBOT_DEBUG_MOTION 		motion_walk_forward
#endif
#define ROBOT_SLOW_STAGE_STEPS     30UL
/* Verified single-foot forward endpoints in BOTH 13-frame gait tables. */
#define ROBOT_WALK_RIGHT_STEP_FRAME 6U
#define ROBOT_WALK_LEFT_STEP_FRAME  10U

#define ROBOT_CONTROL_PERIOD_MS    20U
#define ROBOT_STARTUP_SETTLE_MS     500U
#define ROBOT_ACTION_START_DELAY_MS 1000U

/* 原版geometry_vision7：0对应SERIAL_MODE="ascii"，1对应"binary"。 */
#ifndef CAMERA_PROTOCOL_BINARY
#define CAMERA_PROTOCOL_BINARY 0U
#endif
#if (CAMERA_PROTOCOL_BINARY != 0U) && (CAMERA_PROTOCOL_BINARY != 1U)
#error "CAMERA_PROTOCOL_BINARY must be 0 or 1"
#endif

/* 1输出原始字节、DMA和错误恢复诊断；日常使用0，只保留模式/动作/通信统计。 */
#ifndef ROBOT_RX_DIAGNOSTICS_ENABLE
#define ROBOT_RX_DIAGNOSTICS_ENABLE 0U
#endif

/* 旧策略说明（ROBOT_WALK_LATE_VISION=0）：模式4固定先走一批再纠偏。
 * 行走、收脚、转向、停稳期间丢弃全部巡线CMD；站稳后只采用新CMD。
 * 模式4无指令时一直等待，不使用CORRECTION_WAIT_MS超时续走。
 * 下列AUTOSTART和CORRECTION_WAIT_MS仅控制模式3。 */
#ifndef ROBOT_STM_AUTOSTART
#define ROBOT_STM_AUTOSTART 0U
#endif
#ifndef ROBOT_STM_WALK_BATCH_STEPS
#define ROBOT_STM_WALK_BATCH_STEPS 4U
#endif
#ifndef ROBOT_STM_CORRECTION_WAIT_MS
#define ROBOT_STM_CORRECTION_WAIT_MS 4000U
#endif

/* 下标从0开始：当前13帧行走表在第5帧回环、第12帧计数并退出。 */
#ifndef ROBOT_WALK_LOOP_START_FRAME
#define ROBOT_WALK_LOOP_START_FRAME 4U
#endif
#ifndef ROBOT_WALK_CYCLE_END_FRAME
#define ROBOT_WALK_CYCLE_END_FRAME 11U
#endif
#ifndef ROBOT_WALK_STOP_FRAME
#define ROBOT_WALK_STOP_FRAME 11U
#endif

/* 一个完整左右腿周期按2步换算；按实机步数定义调整，不修改动作角度。 */
#ifndef ROBOT_WALK_STEPS_PER_CYCLE
#define ROBOT_WALK_STEPS_PER_CYCLE 2U
#endif
#define ROBOT_WALK_MAX_BATCH_STEPS 20U

/* 新策略：每个完整左右腿周期帧10..11采样，周期末决策。
 * 0保留旧站稳测量策略；图形调试/动作调试不启用。停止和图形始终独立处理。 */
#ifndef ROBOT_WALK_LATE_VISION
#define ROBOT_WALK_LATE_VISION 1U
#endif
#define ROBOT_WALK_LATE_ACTIVE ((ROBOT_WALK_LATE_VISION != 0U) && \
    ((ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN) || (ROBOT_RUN_MODE == ROBOT_MODE_LINE_DEBUG)))
#define ROBOT_WALK_SAMPLE_START_FRAME 10U
#define ROBOT_WALK_SAMPLE_END_FRAME   11U
/* 只改变采样/调度，不改变动作表。0可回退完整的原收脚纠偏流程。 */
#ifndef ROBOT_WALK_TURN_HANDOFF_ENABLE
#define ROBOT_WALK_TURN_HANDOFF_ENABLE 1U
#endif
#define ROBOT_WALK_RIGHT_SAMPLE_START_FRAME 9U
#define ROBOT_WALK_RIGHT_HANDOFF_FRAME     10U
#define ROBOT_WALK_LEFT_HANDOFF_FRAME      6U
/* 只容纳调度抖动；不是新增等待。超出原路径预算的重试不再执行旧转向。 */
#define ROBOT_WALK_HANDOFF_DELAY_BUDGET_MS 100U
#if ROBOT_WALK_TURN_HANDOFF_ENABLE != 0U && ROBOT_WALK_TURN_HANDOFF_ENABLE != 1U
#error "ROBOT_WALK_TURN_HANDOFF_ENABLE must be 0 or 1"
#endif
#if ROBOT_WALK_RIGHT_SAMPLE_START_FRAME > ROBOT_WALK_RIGHT_HANDOFF_FRAME || ROBOT_WALK_RIGHT_HANDOFF_FRAME >= ROBOT_WALK_CYCLE_END_FRAME || ROBOT_WALK_LEFT_HANDOFF_FRAME >= ROBOT_WALK_CYCLE_END_FRAME
#error "Invalid walk turn handoff frames"
#endif
#if ROBOT_WALK_LEFT_HANDOFF_FRAME < ROBOT_WALK_LOOP_START_FRAME
#error "Left handoff must be reachable after the walking loop restart"
#endif
/* 1每周期独立决策；0恢复只在普通批次最后一个周期采样。批次仍按4步计。 */
#ifndef ROBOT_WALK_DECIDE_EACH_CYCLE
#define ROBOT_WALK_DECIDE_EACH_CYCLE 1U
#endif
#if ROBOT_WALK_DECIDE_EACH_CYCLE != 0U && ROBOT_WALK_DECIDE_EACH_CYCLE != 1U
#error "ROBOT_WALK_DECIDE_EACH_CYCLE must be 0 or 1"
#endif
/* 窗口跟随上面指定帧的实际时长；约2fps使用2条独立新结果连续同向确认。 */
#define ROBOT_WALK_VOTE_WINDOW        2U
#define ROBOT_WALK_VOTE_NEED          2U
#define CAMERA_WALK_SAMPLE_QUEUE_LENGTH 16U
#if ROBOT_WALK_VOTE_NEED == 0U || ROBOT_WALK_VOTE_NEED > ROBOT_WALK_VOTE_WINDOW
#error "Invalid walk vision vote configuration"
#endif
#if ROBOT_WALK_SAMPLE_START_FRAME > ROBOT_WALK_SAMPLE_END_FRAME || ROBOT_WALK_SAMPLE_END_FRAME != ROBOT_WALK_CYCLE_END_FRAME || ROBOT_WALK_STOP_FRAME != ROBOT_WALK_CYCLE_END_FRAME
#error "Late vision requires its sample end and stop at the cycle boundary"
#endif

/* TURN只采用左右方向，每条可执行指令播放1遍转向动作，不按角度换算。 */

/* 动作组外的额外停稳时间；默认0，回站/保持由各动作组承担。 */
#ifndef ROBOT_WALK_STARTUP_SETTLE_MS
#define ROBOT_WALK_STARTUP_SETTLE_MS 0U
#endif
#ifndef ROBOT_WALK_SETTLE_MS
#define ROBOT_WALK_SETTLE_MS 0U
#endif
/* 1：所有已回站的图形组都不再重复站立；0回到仅左腿去重的旧行为。 */
#ifndef ROBOT_SKIP_DUPLICATE_SHAPE_STAND
#define ROBOT_SKIP_DUPLICATE_SHAPE_STAND 1U
#endif
#if (ROBOT_SKIP_DUPLICATE_SHAPE_STAND != 0U) && (ROBOT_SKIP_DUPLICATE_SHAPE_STAND != 1U)
#error "ROBOT_SKIP_DUPLICATE_SHAPE_STAND must be 0 or 1"
#endif
/* 左右转末帧回站后的额外停稳；0直接完成，动作组内回站时间不变。 */
#ifndef ROBOT_WALK_TURN_SETTLE_MS
#define ROBOT_WALK_TURN_SETTLE_MS 0U
#endif
/* 仅模式3：0图形到来后在当前安全周期结束收脚，直接执行图形动作。
 * 1保留旧的回站后额外完整直行周期（左右各一脚、共2步）流程。 */
#ifndef ROBOT_SHAPE_STEP_BEFORE_ACTION
#define ROBOT_SHAPE_STEP_BEFORE_ACTION 0U
#endif
#if (ROBOT_SHAPE_STEP_BEFORE_ACTION != 0U) && (ROBOT_SHAPE_STEP_BEFORE_ACTION != 1U)
#error "ROBOT_SHAPE_STEP_BEFORE_ACTION must be 0 or 1"
#endif
#define ROBOT_WALK_SHAPE_QUEUE_LENGTH 4U

#endif
