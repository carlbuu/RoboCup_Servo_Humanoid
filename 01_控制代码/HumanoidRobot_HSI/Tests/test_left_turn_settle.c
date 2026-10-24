#define main existing_mode_test_main
#include "test_three_modes.c"
#undef main

static void TestDirectionSpecificSettling(void)
{
    uint8_t direction;
    for (direction = 0U; direction < 2U; ++direction)
    {
        uint32_t began, duration, stopped, settle, sends;
        MotionId_t motion = direction == 0U ? motion_turn_left : motion_turn_right;
        const MotionGroup_t *group = MotionGroups_Get(motion);
        ResetRobot(); WaitPhase(ROBOT_WALK_MEASURING); Tick();
        Command(direction == 0U ? CAMERA_WALK_LEFT : CAMERA_WALK_RIGHT, 1U); Tick();
        CHECK(MotionPlayer_GetMotion() == motion);
        began = s_tick;
        duration = RangeTime(motion, 0U, (uint8_t)(group->frameCount - 1U));
        while (s_tick - began < duration)
        {
            Command(CAMERA_WALK_FORWARD, 2U); Tick();
            CHECK(MotionPlayer_GetMotion() == motion);
        }
        CHECK(RobotControl_GetStatus().walkPhase == (ROBOT_WALK_TURN_SETTLE_MS == 0U
              ? ROBOT_WALK_MEASURING : ROBOT_WALK_SETTLING));
        CHECK(!MotionPlayer_IsActive());
        CheckStand();
        settle = ROBOT_WALK_TURN_SETTLE_MS;
        stopped = s_tick; sends = s_servoSends;
        while (RobotControl_GetStatus().walkPhase != ROBOT_WALK_MEASURING)
        {
            CHECK(s_tick - stopped < settle + 20U);
            Command(CAMERA_WALK_FORWARD, 2U); Tick();
            CHECK(MotionPlayer_GetMotion() == motion);
        }
        CHECK(s_tick - stopped == ((settle + 19U) / 20U) * 20U);
        CHECK(s_servoSends == sends); /* No repeated standing motion. */
        CHECK(RobotControl_GetStatus().lastCompletedCommandSequence != 0U);
        Tick(); Tick();
        CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING);
        Command(CAMERA_WALK_FORWARD, 2U); Tick();
        CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MOVING);
        CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
    }
    puts("PASS left/right timing, no duplicate stand, busy FWD rejected and fresh FWD accepted");
}

static void TestWalkingSettleUnchanged(void)
{
    uint32_t began;
    ResetRobot(); WaitPhase(ROBOT_WALK_MEASURING); Tick();
    Command(CAMERA_WALK_FORWARD, 2U); Tick();
    WaitPhase(ROBOT_WALK_TO_STAND);
    WaitPhase(ROBOT_WALK_SETTLING);
    began = s_tick;
    WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(s_tick - began == ROBOT_WALK_SETTLE_MS);
    CheckStand();
    puts("PASS walking recovery still settles for the original 300ms");
}

#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
static void TestShapeSettleUnchanged(void)
{
    uint32_t began;
    ResetRobot(); WaitPhase(ROBOT_WALK_MEASURING);
    Shape(3U); Tick();
    while (RobotControl_GetStatus().walkPhase == ROBOT_WALK_EVENT) { Tick(); }
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_SETTLING);
    CHECK(MotionPlayer_GetMotion() == motion_qr_left_leg);
    began = s_tick;
    WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(s_tick - began == ROBOT_WALK_SETTLE_MS);
    CHECK(RobotControl_GetStatus().eventsCompleted == 1U);
    puts("PASS previous left-leg duplicate-stand fix preserved with 300ms settle");
}
#endif

static void TestFinalTurnTargetRetry(void)
{
    uint32_t began, duration;
    const MotionGroup_t *group = MotionGroups_Get(motion_turn_left);
    ResetRobot(); WaitPhase(ROBOT_WALK_MEASURING); Tick();
    Command(CAMERA_WALK_LEFT, 1U); Tick();
    began = s_tick;
    duration = RangeTime(motion_turn_left, 0U, (uint8_t)(group->frameCount - 1U));
    while (s_tick - began < duration - 20U) { Tick(); }
    s_servoAccepted = 0U; Tick(); Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MOVING);
    CHECK(MotionPlayer_IsActive());
    CHECK(RobotControl_GetStatus().lastCompletedCommandSequence == 0U);
    s_servoAccepted = 1U; Tick();
    CHECK(RobotControl_GetStatus().walkPhase == (ROBOT_WALK_TURN_SETTLE_MS == 0U
          ? ROBOT_WALK_MEASURING : ROBOT_WALK_SETTLING));
    WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().lastCompletedCommandSequence != 0U);
    puts("PASS turn completion waits for successful final target submission");
}

int main(void)
{
    TestDirectionSpecificSettling();
    TestFinalTurnTargetRetry();
    TestWalkingSettleUnchanged();
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    TestShapeSettleUnchanged();
#endif
    TestWalkRecoveryContinuity();
    puts("ALL LEFT TURN SETTLE TESTS PASSED");
    return 0;
}
