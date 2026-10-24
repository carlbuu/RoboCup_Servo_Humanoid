/* Real camera parser, scheduler and player. Only hardware APIs are stubbed.
 * Keep the original endpoint regression intact; check every new 20ms output too. */
#define main original_three_modes_main
#define Servo_SendTargets Handoff_BaseSendTargets
#include "test_three_modes.c"
#undef Servo_SendTargets
#undef main

static uint8_t s_injectStopOnSubmit;
uint8_t Servo_SendTargets(void)
{
    if (s_injectStopOnSubmit != 0U)
    {
        s_injectStopOnSubmit = 0U;
        Command(CAMERA_WALK_BLOCKED, 0U);
    }
    return Handoff_BaseSendTargets();
}

static void Step(void)
{
    float before[SERVO_COUNT + 1];
    uint8_t id;
    memcpy(before, s_targets, sizeof(before));
    Tick();
    for (id = 1U; id <= SERVO_ACTIVE_COUNT; ++id)
    { CHECK(fabsf(s_targets[id] - before[id]) <= 5.0f); }
}

static void UntilPhase(RobotWalkPhase_t phase)
{
    uint32_t began = s_tick;
    while (RobotControl_GetStatus().walkPhase != phase)
    { CHECK(s_tick - began < 30000U); Step(); }
}

static void UntilFrame(uint8_t frame)
{
    uint32_t began = s_tick;
    while (MotionPlayer_GetMotion() != motion_walk_forward ||
           MotionPlayer_GetFrameIndex() != frame)
    { CHECK(s_tick - began < 30000U); Step(); }
}

static void Advance(uint32_t ms)
{
    uint32_t began = s_tick;
    while (s_tick - began < ms) { Step(); }
}

static void Result(uint8_t command)
{
    Command(command, command == CAMERA_WALK_FORWARD ? 2U :
        (command == CAMERA_WALK_LEFT || command == CAMERA_WALK_RIGHT ? 3U : 0U));
    RobotControl_Update(0U);
}

static void TwoResults(uint8_t command)
{
    Step(); Result(command); Advance(520U); Result(command); Step();
}

static void StartAt(TickType_t initialTick)
{
    memset(s_targets, 0, sizeof(s_targets));
    ResetLink(); s_tick = initialTick; RobotControl_Init();
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    UntilPhase(ROBOT_WALK_MEASURING);
    TwoResults(CAMERA_WALK_FORWARD);
#else
    UntilPhase(ROBOT_WALK_MOVING);
#endif
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
}

static void Start(void) { StartAt(0U); }

static void EarlyWindow(void)
{
    uint32_t began = s_tick;
    while (!RobotControl_GetStatus().rightSampleWindowOpen)
    { CHECK(s_tick - began < 20000U); Step(); }
    CHECK(MotionPlayer_GetFrameIndex() == 9U);
}

static void LateWindow(void)
{
    uint32_t began = s_tick;
    while (!RobotControl_GetStatus().sampleWindowOpen)
    { CHECK(s_tick - began < 20000U); Step(); }
    CHECK(MotionPlayer_GetFrameIndex() == 10U);
}

#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
static void BeginRight(void)
{
    Start(); EarlyWindow(); TwoResults(CAMERA_WALK_RIGHT);
    UntilPhase(ROBOT_WALK_TURN_HANDOFF);
    CHECK(MotionPlayer_GetMotion() == motion_turn_right);
    CHECK(MotionPlayer_GetFrameIndex() == 5U);
}
#endif

static void BeginLeftApproach(void)
{
    Start(); LateWindow(); TwoResults(CAMERA_WALK_LEFT);
    UntilPhase(ROBOT_WALK_TURN_APPROACH);
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
    CHECK(MotionPlayer_GetFrameIndex() == 12U);
    CHECK(RobotControl_GetStatus().pendingTurn == CAMERA_WALK_LEFT);
}

static void TestRightAndFreshStandingTurn(void)
{
    uint32_t opened, tail, ready, sequence;
    RobotControlStatus_t status;
    Start(); EarlyWindow(); opened = s_tick;
    TwoResults(CAMERA_WALK_RIGHT);
    CHECK(RobotControl_GetStatus().rightSampleCount == 2U);
    RobotControl_Update(0U); RobotControl_Update(0U);
    CHECK(RobotControl_GetStatus().rightSampleCount == 2U);
    UntilPhase(ROBOT_WALK_TURN_HANDOFF); tail = s_tick;
    CHECK(tail - opened == RangeTime(motion_walk_forward, 9U, 10U));
    status = RobotControl_GetStatus(); sequence = status.activeCommandSequence;
    CHECK(status.turnFromWalk && status.lastWalkExitFrame == 10U);
    CHECK(status.lastWalkPartialSteps == 2U);
    CHECK(status.lastWalkCompletedCycles == (ROBOT_WALK_DECIDE_EACH_CYCLE ? 0U : 1U));
    CHECK(MotionPlayer_GetFrameIndex() == 5U);
    CHECK(fabsf(s_targets[11] - 18.0f) < 0.001f);
    CHECK(fabsf(s_targets[12] + 18.0f) < 0.001f);
    UntilPhase(ROBOT_WALK_MEASURING); ready = s_tick;
    CHECK(ready - tail == 600U); CheckStand();
    CHECK(RobotControl_GetStatus().lastCompletedCommandSequence == sequence);
    {
        uint8_t bytes[64]; uint16_t len = CommandBytes(bytes, CAMERA_WALK_RIGHT, 3U);
        Camera_ProcessTimed(bytes, len, ready - 100U);
        Camera_ProcessTimed(bytes, len, ready - 80U);
        Step(); CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING);
    }
    Advance(4200U);
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING);
    Result(CAMERA_WALK_RIGHT); Step();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING);
    Advance(520U); Result(CAMERA_WALK_RIGHT); Step();
    CHECK(MotionPlayer_GetMotion() == motion_turn_right);
    CHECK(MotionPlayer_GetFrameIndex() == 0U);
    CHECK(!RobotControl_GetStatus().turnFromWalk);
    CHECK(RobotControl_GetStatus().activeCommandSequence != sequence);
    tail = s_tick; UntilPhase(ROBOT_WALK_MEASURING);
    CHECK(s_tick - tail == RangeTime(motion_turn_right, 0U, 5U));
    TwoResults(CAMERA_WALK_FORWARD);
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    CHECK(RobotControl_GetStatus().targetCycles == 1U);
#endif
    puts("PASS early RIGHT: exact phase, 600ms tail, arms interpolate, partial ledger, fresh standing correction and FWD");
}

static void TestLateRightUsesOriginalReturn(void)
{
    uint32_t returned;
    Start(); EarlyWindow();
    Step(); Result(CAMERA_WALK_RIGHT); /* only one early observation */
    UntilFrame(11U);
    CHECK(!RobotControl_GetStatus().rightSampleWindowOpen);
    Step(); Result(CAMERA_WALK_RIGHT); Advance(200U); Result(CAMERA_WALK_RIGHT);
    UntilPhase(ROBOT_WALK_TO_STAND); returned = s_tick;
    CHECK(MotionPlayer_GetMotion() == motion_walk_tostand);
    UntilPhase(ROBOT_WALK_MOVING);
    CHECK(s_tick - returned == 1300U);
    CHECK(MotionPlayer_GetMotion() == motion_turn_right);
    CHECK(MotionPlayer_GetFrameIndex() == 0U);
    CHECK(!RobotControl_GetStatus().turnFromWalk);
    puts("PASS late RIGHT: keep frame12, original unmodified return and standing turn; no reverse transition");
}

static void TestOldSplitSingleAndLatestDisagreement(void)
{
    uint8_t bytes[64]; uint16_t len;
    uint32_t completed;
    Start(); len = CommandBytes(bytes, CAMERA_WALK_RIGHT, 3U);
    Camera_ProcessTimed(bytes, 2U, s_tick);
    EarlyWindow(); Step();
    Camera_ProcessTimed(bytes + 2U, (uint16_t)(len - 2U), s_tick);
    RobotControl_Update(0U);
    CHECK(RobotControl_GetStatus().rightSampleCount == 0U);
    Result(CAMERA_WALK_RIGHT);
    RobotControl_Update(0U); RobotControl_Update(0U);
    CHECK(RobotControl_GetStatus().rightSampleCount == 1U);
    completed = MotionPlayer_GetCompletedCycles();
    UntilFrame(11U);
    CHECK(MotionPlayer_GetCompletedCycles() == completed);
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
    Start(); EarlyWindow(); TwoResults(CAMERA_WALK_RIGHT);
    Step(); Result(CAMERA_WALK_FORWARD);
    UntilFrame(11U);
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MOVING);
    puts("PASS split-old packet, repeated cache reads and latest disagreement cannot trigger early RIGHT");
}

static void TestLeftPathAndCancellation(uint8_t forwards)
{
    uint32_t began;
    BeginLeftApproach(); began = s_tick;
    if (forwards != 0U)
    {
        Advance(200U); Result(CAMERA_WALK_FORWARD);
        if (forwards == 2U) { Advance(520U); Result(CAMERA_WALK_FORWARD); }
    }
    if (forwards == 2U)
    {
        uint16_t completed = MotionPlayer_GetCompletedCycles();
        CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MOVING);
        while (MotionPlayer_GetCompletedCycles() == completed)
        {
            Step();
            CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
            CHECK(RobotControl_GetStatus().walkPhase != ROBOT_WALK_TURN_HANDOFF);
        }
        CHECK(!RobotControl_GetStatus().pendingTurn);
        puts("PASS two fresh FWD cancel LEFT approach and continue complete original support path");
        return;
    }
    UntilPhase(ROBOT_WALK_TURN_HANDOFF);
    CHECK(s_tick - began == RangeTime(motion_walk_forward, 12U, 12U) +
        RangeTime(motion_walk_forward, 4U, 6U));
    CHECK(MotionPlayer_GetMotion() == motion_turn_left);
    CHECK(MotionPlayer_GetFrameIndex() == 6U);
    CHECK(RobotControl_GetStatus().lastWalkExitFrame == 6U);
    CHECK(RobotControl_GetStatus().lastWalkPartialSteps == 1U);
    CHECK(RobotControl_GetStatus().lastWalkCompletedCycles ==
        (ROBOT_WALK_DECIDE_EACH_CYCLE ? 1U : 2U));
    CHECK(fabsf(s_targets[11] + 18.0f) < 0.001f);
    CHECK(fabsf(s_targets[12] - 18.0f) < 0.001f);
    began = s_tick; UntilPhase(ROBOT_WALK_MEASURING);
    CHECK(s_tick - began == 600U); CheckStand();
    puts("PASS LEFT: original 12->4->5->6 support path, exact timing, single FWD cannot cancel, 600ms tail");
}

static void TestApproachStop(uint8_t stop, uint8_t frame)
{
    uint16_t i;
    BeginLeftApproach(); UntilFrame(frame);
    Result(stop);
    for (i = 0U; i < 30U; ++i) { Result(CAMERA_WALK_FORWARD); }
    Step();
    CHECK(RobotControl_GetStatus().stopRequested);
    UntilPhase(ROBOT_WALK_TO_STAND);
    CHECK(MotionPlayer_GetMotion() == motion_walk_tostand);
    UntilPhase(stop == CAMERA_WALK_BLOCKED ? ROBOT_WALK_BLOCKED : ROBOT_WALK_MEASURING);
    CHECK(MotionPlayer_GetMotion() == motion_walk_tostand);
    CHECK(!RobotControl_GetStatus().pendingTurn);
    puts("PASS stop during LEFT approach cancels turn, completes original cycle and original return; FWD flood cannot resume");
}

static void TestSubmissionRetryCannotCommitExpiredTurn(uint8_t left)
{
    uint32_t began;
    if (left) { BeginLeftApproach(); UntilFrame(6U); }
    else { Start(); EarlyWindow(); TwoResults(CAMERA_WALK_RIGHT); UntilFrame(10U); }
    s_servoAccepted = 0U; began = s_tick; Advance(1000U);
    CHECK(s_tick - began == 1000U);
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
    CHECK(MotionPlayer_IsActive());
    s_servoAccepted = 1U; Step();
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
    CHECK(MotionPlayer_GetFrameIndex() == (left ? 7U : 11U));
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MOVING);
    puts("PASS endpoint submission failure cannot advance or consume; expired turn falls back along original gait");
}

static void TestRightStopAtCommit(void)
{
    uint32_t opened;
    Start(); EarlyWindow(); opened = s_tick; TwoResults(CAMERA_WALK_RIGHT);
    while (s_tick - opened < RangeTime(motion_walk_forward, 9U, 10U) - 20U) { Step(); }
    Command(CAMERA_WALK_BLOCKED, 0U); /* no Update(0): final control tick must drain it */
    Step();
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
    CHECK(MotionPlayer_GetFrameIndex() == 11U);
    UntilPhase(ROBOT_WALK_BLOCKED);
    CHECK(MotionPlayer_GetMotion() == motion_walk_tostand);
    puts("PASS STOP queued on final pre-handoff tick wins over confirmed RIGHT");
}

static void TestStopArrivesDuringFinalSubmission(void)
{
    uint32_t opened;
    Start(); EarlyWindow(); opened = s_tick; TwoResults(CAMERA_WALK_RIGHT);
    while (s_tick - opened < RangeTime(motion_walk_forward, 9U, 10U) - 20U) { Step(); }
    s_injectStopOnSubmit = 1U;
    Step(); /* Camera request arrives after first drain, during endpoint submission. */
    CHECK(!s_injectStopOnSubmit);
    CHECK(RobotControl_GetStatus().stopRequested);
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
    CHECK(MotionPlayer_GetFrameIndex() == 11U);
    UntilPhase(ROBOT_WALK_BLOCKED);
    CHECK(MotionPlayer_GetMotion() == motion_walk_tostand);
    puts("PASS STOP arriving during final submission is drained again before turn consumption");
}

static void TestClockWrapAndSmallSchedulingDelay(void)
{
    StartAt((TickType_t)(0xFFFFFFFFUL - 6500U));
    EarlyWindow(); TwoResults(CAMERA_WALK_RIGHT);
    s_tick += 40U; /* elapsed player time stays at 20ms per call; only wall clock is delayed. */
    UntilPhase(ROBOT_WALK_TURN_HANDOFF);
    CHECK(MotionPlayer_GetMotion() == motion_turn_right);
    CHECK(MotionPlayer_GetFrameIndex() == 5U);
    UntilPhase(ROBOT_WALK_MEASURING); CheckStand();
    puts("PASS tick wrap and 40ms scheduling jitter preserve fresh-window handoff");
}

#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
static void TestShapePriority(uint8_t duringTail)
{
    uint32_t began;
    if (duringTail) { BeginRight(); }
    else { BeginLeftApproach(); }
    began = s_tick; Shape(1U); Shape(1U); Step();
    CHECK(RobotControl_GetStatus().pendingShapes == 1U);
    UntilPhase(ROBOT_WALK_EVENT);
    CHECK(MotionPlayer_GetMotion() == MotionGroups_FromQr(1U));
    CHECK(RobotControl_GetStatus().eventsStarted == 1U);
    if (duringTail) { CHECK(s_tick - began == 600U); }
    UntilPhase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().eventsCompleted == 1U);
    CHECK(!RobotControl_GetStatus().pendingShapes);
    puts("PASS shape: cancels unconsumed LEFT or waits only for consumed RIGHT tail; no lost/duplicate event");
}

static void TestTailStopAndShape(void)
{
    BeginRight(); Shape(1U); Result(CAMERA_WALK_BLOCKED);
    Result(CAMERA_WALK_FORWARD); Result(CAMERA_WALK_FORWARD); Step();
    UntilPhase(ROBOT_WALK_BLOCKED);
    CHECK(RobotControl_GetStatus().eventsStarted == 0U);
    CHECK(RobotControl_GetStatus().pendingShapes == 1U);
    Advance(1000U);
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_BLOCKED);
    TwoResults(CAMERA_WALK_FORWARD);
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_EVENT);
    UntilPhase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().eventsCompleted == 1U);
    puts("PASS stop during consumed tail completes safe return, holds queued shape and requires fresh resume votes");
}
#endif

int main(void)
{
    TestRightAndFreshStandingTurn();
    TestLateRightUsesOriginalReturn();
    TestOldSplitSingleAndLatestDisagreement();
    TestLeftPathAndCancellation(0U);
    TestLeftPathAndCancellation(1U);
    TestLeftPathAndCancellation(2U);
    TestApproachStop(CAMERA_WALK_HOLD, 12U);
    TestApproachStop(CAMERA_WALK_BLOCKED, 5U);
    TestApproachStop(CAMERA_WALK_HOLD, 6U);
    TestSubmissionRetryCannotCommitExpiredTurn(0U);
    TestSubmissionRetryCannotCommitExpiredTurn(1U);
    TestRightStopAtCommit();
    TestStopArrivesDuringFinalSubmission();
    TestClockWrapAndSmallSchedulingDelay();
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    TestShapePriority(0U); TestShapePriority(1U); TestTailStopAndShape();
#endif
    puts("ALL WALK TURN HANDOFF TESTS PASSED; 13 joints <=5deg/20ms; HARDWARE NOT TESTED");
    return 0;
}
