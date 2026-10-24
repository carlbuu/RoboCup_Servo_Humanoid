/* Uses the existing hardware stubs with the real parser, scheduler and player. */
#define main existing_mode_test_main
#include "test_three_modes.c"
#undef main

#ifndef TEST_NONZERO_LEFT_END
#define TEST_NONZERO_LEFT_END 0
#endif

#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN || ROBOT_RUN_MODE == ROBOT_MODE_VISION_DEBUG
static void TestRecoveryForEveryShape(void)
{
    uint8_t shape;
    for (shape = 1U; shape <= 6U; ++shape)
    {
        uint32_t began;
        uint32_t duration;
        MotionId_t motion = MotionGroups_FromQr(shape);
        const MotionGroup_t *group = MotionGroups_Get(motion);
        uint8_t skip = (uint8_t)(motion == motion_qr_left_leg && !TEST_NONZERO_LEFT_END);
        ResetRobot();
        WaitPhase(ROBOT_WALK_MEASURING);
        Shape(shape); Tick();
        CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_EVENT);
        CHECK(MotionPlayer_GetMotion() == motion);
        began = s_tick;
        duration = RangeTime(motion, 0U, (uint8_t)(group->frameCount - 1U));
        while (RobotControl_GetStatus().walkPhase == ROBOT_WALK_EVENT) { Tick(); }
        CHECK(s_tick - began == duration);
        CHECK(RobotControl_GetStatus().eventsCompleted == 0U);
        if (skip)
        {
            CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_SETTLING);
            CHECK(MotionPlayer_GetMotion() == motion_qr_left_leg);
            CheckStand();
        }
        else
        {
            CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_RECOVER);
            CHECK(MotionPlayer_GetMotion() == motion_stand);
        }
        WaitPhase(ROBOT_WALK_MEASURING);
        CHECK(s_tick - began == duration + ROBOT_WALK_SETTLE_MS +
              (skip ? 0U : RangeTime(motion_stand, 0U, 0U)));
        CHECK(RobotControl_GetStatus().eventsCompleted == 1U);
        CheckStand();
    }
    puts("PASS all six shapes: left-leg guarded recovery saves only duplicate stand; other recovery unchanged");
}

static void TestLastFrameMustBeSubmitted(void)
{
    const MotionGroup_t *group = MotionGroups_Get(motion_qr_left_leg);
    uint8_t last = (uint8_t)(group->frameCount - 1U);
    uint32_t i;
    ResetRobot(); WaitPhase(ROBOT_WALK_MEASURING);
    Shape(3U); Tick();
    while (MotionPlayer_GetFrameIndex() != last) { Tick(); }
    s_servoAccepted = 0U;
    for (i = 0U; i < RangeTime(motion_qr_left_leg, last, last) + 100U; i += 20U) { Tick(); }
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_EVENT);
    CHECK(MotionPlayer_IsActive());
    CHECK(RobotControl_GetStatus().eventsCompleted == 0U);
    s_servoAccepted = 1U;
    Tick();
    CHECK(RobotControl_GetStatus().walkPhase ==
          (TEST_NONZERO_LEFT_END ? ROBOT_WALK_RECOVER : ROBOT_WALK_SETTLING));
    WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().eventsCompleted == 1U);
    puts("PASS rejected final target cannot start recovery or finish the event");
}

#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
static void TestFreshGateAfterLeftLeg(void)
{
    uint32_t began;
    ResetRobot(); WaitPhase(ROBOT_WALK_MEASURING);
    Shape(3U); Tick();
    while (RobotControl_GetStatus().walkPhase == ROBOT_WALK_EVENT) { Tick(); }
    while (RobotControl_GetStatus().walkPhase != ROBOT_WALK_MEASURING)
    {
        Command(CAMERA_WALK_FORWARD, 2U);
        Tick();
    }
    CHECK(RobotControl_GetStatus().eventsCompleted == 1U);
    began = s_tick;
    while (s_tick - began < ROBOT_STM_CORRECTION_WAIT_MS + 500U)
    {
        Tick();
        CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING);
    }
    Command(CAMERA_WALK_FORWARD, 2U); Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MOVING);
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
    puts("PASS busy FWD discarded; 300ms settle retained; only new post-settle FWD starts walking");
}
#endif
#endif

int main(void)
{
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN || ROBOT_RUN_MODE == ROBOT_MODE_VISION_DEBUG
    TestRecoveryForEveryShape();
    TestLastFrameMustBeSubmitted();
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    TestFreshGateAfterLeftLeg();
#endif
    TestTurnSettlingLatency();
    TestWalkRecoveryContinuity();
#elif ROBOT_RUN_MODE == ROBOT_MODE_LINE_DEBUG
    TestLineDebugBatchGate();
#else
    TestActionDebugIsolation();
#endif
    puts("ALL LEFT LEG RECOVERY TESTS PASSED");
    return 0;
}
