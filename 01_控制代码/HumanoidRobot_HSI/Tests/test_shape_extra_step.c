#define main existing_mode_test_main
#include "test_three_modes.c"
#undef main

#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN && ROBOT_SHAPE_STEP_BEFORE_ACTION
static uint32_t ExtraTime(void)
{
    return WalkBatchTime(1U) + RangeTime(motion_walk_tostand, 0U,
           (uint8_t)(MotionGroups_Get(motion_walk_tostand)->frameCount - 1U)) + ROBOT_WALK_SETTLE_MS;
}
static void Ready(void) { ResetRobot(); WaitPhase(ROBOT_WALK_MEASURING); Tick(); }

static void TestStationaryAndDuplicate(void)
{
    uint32_t began, returned;
    uint16_t seen = 0U;
    Ready(); Shape(3U); Shape(3U); Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_SHAPE_STEP);
    CHECK(RobotControl_GetStatus().eventsStarted == 0U);
    CHECK(RobotControl_GetStatus().pendingShapes == 1U);
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
    CHECK(MotionPlayer_GetFrameIndex() == 0U);
    CHECK(RobotControl_GetStatus().targetCycles == 1U);
    began = s_tick;
    while (RobotControl_GetStatus().walkPhase == ROBOT_WALK_SHAPE_STEP)
    {
        CHECK(MotionPlayer_GetFrameIndex() <= ROBOT_WALK_STOP_FRAME);
        seen |= (uint16_t)(1U << MotionPlayer_GetFrameIndex());
        Shape(3U); Command(CAMERA_WALK_FORWARD, 2U); Tick();
        CHECK(RobotControl_GetStatus().eventsStarted == 0U);
        CHECK(RobotControl_GetStatus().pendingShapes == 1U);
        CHECK(s_tick - began <= WalkBatchTime(1U));
    }
    CHECK(s_tick - began == WalkBatchTime(1U));
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_SHAPE_STEP_RETURN);
    CHECK((seen & (1U << 7U)) != 0U && (seen & (1U << 8U)) != 0U &&
          (seen & (1U << 9U)) != 0U && (seen & (1U << 10U)) != 0U &&
          (seen & (1U << 11U)) != 0U); /* 必须包含左腿重心转移、抬腿、前摆及落脚。 */
    CHECK(MotionPlayer_GetMotion() == motion_walk_tostand);
    returned = s_tick;
    while (RobotControl_GetStatus().walkPhase == ROBOT_WALK_SHAPE_STEP_RETURN) { Tick(); }
    CHECK(s_tick - returned == RangeTime(motion_walk_tostand, 0U,
          (uint8_t)(MotionGroups_Get(motion_walk_tostand)->frameCount - 1U)));
    if (ROBOT_WALK_SETTLE_MS == 0U)
    {
        CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_EVENT);
        CHECK(RobotControl_GetStatus().eventsStarted == 1U);
    }
    else
    {
        CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_SETTLING);
        CHECK(RobotControl_GetStatus().eventsStarted == 0U);
    }
    CheckStand();
    WaitPhase(ROBOT_WALK_EVENT);
    CHECK(s_tick - began == ExtraTime());
    CHECK(MotionPlayer_GetMotion() == motion_qr_left_leg);
    CHECK(RobotControl_GetStatus().eventsStarted == 1U);
    CHECK(RobotControl_GetStatus().pendingShapes == 0U);
    WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().eventsCompleted == 1U);
    began = s_tick;
    while (s_tick - began < ROBOT_STM_CORRECTION_WAIT_MS + 500U)
    { Tick(); CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING); }
    Command(CAMERA_WALK_FORWARD, 2U); Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MOVING);
    CHECK(RobotControl_GetStatus().targetCycles == ROBOT_STM_WALK_BATCH_STEPS / ROBOT_WALK_STEPS_PER_CYCLE);
    puts("PASS idle shape: one full forward cycle and both foot-steps, complete return/settle, duplicate merge, fresh-only resume");
}

static void TestArrivalDuringWalk(void)
{
    uint8_t scenario;
    const uint8_t frames[3] = {0U, 7U, 10U};
    for (scenario = 0U; scenario < 3U; ++scenario)
    {
        uint32_t began;
        Ready(); Command(CAMERA_WALK_FORWARD, 2U); Tick();
        while (MotionPlayer_GetFrameIndex() < frames[scenario]) { Tick(); }
        Shape(1U); Tick();
        CHECK(RobotControl_GetStatus().eventsStarted == 0U);
        CHECK(RobotControl_GetStatus().pendingShapes == 1U);
        WaitPhase(ROBOT_WALK_TO_STAND);
        CHECK(MotionPlayer_GetMotion() == motion_walk_tostand);
        CHECK(RobotControl_GetStatus().completedCycles == 0U); /* Recovery just started. */
        began = s_tick;
        WaitPhase(ROBOT_WALK_SHAPE_STEP);
        CHECK(s_tick - began == RangeTime(motion_walk_tostand, 0U,
              (uint8_t)(MotionGroups_Get(motion_walk_tostand)->frameCount - 1U)) + ROBOT_WALK_SETTLE_MS);
        CHECK(RobotControl_GetStatus().eventsStarted == 0U);
        began = s_tick;
        WaitPhase(ROBOT_WALK_EVENT);
        CHECK(s_tick - began == ExtraTime());
        CHECK(RobotControl_GetStatus().eventsStarted == 1U);
    }
    puts("PASS shapes at three walking phases: original safe stop and return occur before extra step");
}

static void TestPauseAndCorrection(void)
{
    uint8_t pause;
    for (pause = 0U; pause < 2U; ++pause)
    {
        uint32_t began;
        Ready(); Command(pause ? CAMERA_WALK_BLOCKED : CAMERA_WALK_HOLD, 0U); Tick();
        Shape(2U); Tick(); began = s_tick;
        while (s_tick - began < 5000U)
        {
            Tick();
            CHECK(RobotControl_GetStatus().eventsStarted == 0U);
            CHECK(RobotControl_GetStatus().walkPhase == (pause ? ROBOT_WALK_BLOCKED : ROBOT_WALK_MEASURING));
        }
        Command(CAMERA_WALK_FORWARD, 2U); Tick();
        CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_SHAPE_STEP);
        WaitPhase(ROBOT_WALK_EVENT);
    }
    Ready(); Command(CAMERA_WALK_RIGHT, 1U); Tick();
    Shape(6U); Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MOVING);
    WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(MotionPlayer_GetMotion() == motion_turn_right);
    CHECK(RobotControl_GetStatus().pendingShapes == 1U);
    CHECK(RobotControl_GetStatus().eventsStarted == 0U);
    Tick(); Command(CAMERA_WALK_FORWARD, 2U); Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_SHAPE_STEP);
    WaitPhase(ROBOT_WALK_EVENT);
    CHECK(MotionPlayer_GetMotion() == motion_qr_head_shake);
    puts("PASS HOLD/BLOCKED defer walking and fresh FWD releases it; turn finishes before approach");
}

static void TestQueueAndStartup(void)
{
    uint32_t began, approaches = 0U;
    RobotWalkPhase_t previous;
    ResetRobot(); Shape(3U); Shape(1U); Shape(2U);
    previous = RobotControl_GetStatus().walkPhase;
    began = s_tick;
    while (RobotControl_GetStatus().eventsCompleted < 3U)
    {
        Tick();
        CHECK(s_tick - began < 100000U);
        if (RobotControl_GetStatus().walkPhase == ROBOT_WALK_SHAPE_STEP && previous != ROBOT_WALK_SHAPE_STEP)
        { approaches++; }
        previous = RobotControl_GetStatus().walkPhase;
    }
    CHECK(approaches == 3U);
    CHECK(RobotControl_GetStatus().eventsStarted == 3U);
    CHECK(RobotControl_GetStatus().pendingShapes == 0U);
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING);
    Shape(3U); Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_SHAPE_STEP);
    puts("PASS startup-queued and consecutive shapes each get one complete-cycle approach; same shape allowed after completion");
}

static void TestSubmissionRetryAndBoundaries(void)
{
    uint32_t i;
    Ready(); Shape(1U); Tick();
    while (MotionPlayer_GetFrameIndex() != ROBOT_WALK_STOP_FRAME) { Tick(); }
    s_servoAccepted = 0U;
    for (i = 0U; i < RangeTime(motion_walk_forward, ROBOT_WALK_STOP_FRAME, ROBOT_WALK_STOP_FRAME) + 100U; i += 20U) { Tick(); }
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_SHAPE_STEP);
    CHECK(RobotControl_GetStatus().eventsStarted == 0U);
    s_servoAccepted = 1U; Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_SHAPE_STEP_RETURN);
    while (MotionPlayer_GetFrameIndex() != MotionGroups_Get(motion_walk_tostand)->frameCount - 1U) { Tick(); }
    s_servoAccepted = 0U;
    for (i = 0U; i < RangeTime(motion_walk_tostand,
          (uint8_t)(MotionGroups_Get(motion_walk_tostand)->frameCount - 1U),
          (uint8_t)(MotionGroups_Get(motion_walk_tostand)->frameCount - 1U)) + 100U; i += 20U) { Tick(); }
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_SHAPE_STEP_RETURN);
    CHECK(RobotControl_GetStatus().eventsStarted == 0U);
    s_servoAccepted = 1U; Tick();
    CHECK(RobotControl_GetStatus().walkPhase == (ROBOT_WALK_SETTLE_MS == 0U
          ? ROBOT_WALK_EVENT : ROBOT_WALK_SETTLING));
    CHECK(RobotControl_GetStatus().eventsStarted == (ROBOT_WALK_SETTLE_MS == 0U ? 1U : 0U));
    WaitPhase(ROBOT_WALK_EVENT);
    CHECK(RobotControl_GetStatus().eventsStarted == 1U);
    puts("PASS failed full-cycle/recovery targets cannot advance to the image action");
}
#endif

static void TestTurnZeroWait(void)
{
    uint8_t direction;
    if (ROBOT_RUN_MODE != ROBOT_MODE_FULL_RUN && ROBOT_RUN_MODE != ROBOT_MODE_LINE_DEBUG) { return; }
    for (direction = 0U; direction < 2U; ++direction)
    {
        ResetRobot(); WaitPhase(ROBOT_WALK_MEASURING); Tick();
        Command(direction ? CAMERA_WALK_RIGHT : CAMERA_WALK_LEFT, 1U); Tick();
        while (RobotControl_GetStatus().walkPhase == ROBOT_WALK_MOVING) { Tick(); }
        CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING);
        CheckStand();
    }
    puts("PASS both turns retain zero extra settle");
}

int main(void)
{
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN && ROBOT_SHAPE_STEP_BEFORE_ACTION
    TestStationaryAndDuplicate(); TestArrivalDuringWalk(); TestPauseAndCorrection();
    TestQueueAndStartup(); TestSubmissionRetryAndBoundaries(); TestTurnZeroWait();
#elif ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN || ROBOT_RUN_MODE == ROBOT_MODE_VISION_DEBUG
    ResetRobot(); WaitPhase(ROBOT_WALK_MEASURING); Shape(3U); Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_EVENT);
    CHECK(MotionPlayer_GetMotion() == motion_qr_left_leg);
    WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().eventsCompleted == 1U);
    TestTurnZeroWait();
    puts("PASS disabled feature or image-only mode remains immediate");
#elif ROBOT_RUN_MODE == ROBOT_MODE_LINE_DEBUG
    TestLineDebugBatchGate(); TestTurnZeroWait();
#else
    TestActionDebugIsolation(); TestTurnZeroWait();
#endif
    TestWalkRecoveryContinuity();
    puts("ALL SHAPE EXTRA STEP TESTS PASSED");
    return 0;
}
