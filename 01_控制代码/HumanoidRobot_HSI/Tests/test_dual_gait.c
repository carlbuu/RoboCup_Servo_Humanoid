/* Real camera parser, scheduler, player and tables; only hardware is simulated. */
#define main original_modes_main
#define Servo_SetTarget BaseSetTarget
#define Servo_SendTargets BaseSendTargets
#define Debug_Printf BaseDebugPrintf
#include "test_three_modes.c"
#undef main
#undef Servo_SetTarget
#undef Servo_SendTargets
#undef Debug_Printf

static uint8_t s_inject; /* one-shot request arriving DURING servo submission */
static uint32_t s_stepLogs, s_switchLogs, s_thresholdLogs;
static uint32_t s_countOracle;
static float s_attemptedTargets[SERVO_COUNT + 1];

void Debug_Printf(char *format, ...)
{
    va_list args;
    if (strstr(format, "[STEP]") != NULL) { s_stepLogs++; }
    if (strstr(format, "switch=done") != NULL) { s_switchLogs++; }
    if (strstr(format, "switch=requested") != NULL) { s_thresholdLogs++; }
    va_start(args, format); vprintf(format, args); va_end(args);
}

uint8_t Servo_SetTarget(uint8_t id, uint16_t speed, uint8_t acceleration, float angle)
{
    MotionId_t motion = MotionPlayer_GetMotion();
    uint16_t expected = motion == motion_stand ? SERVO_STARTUP_STAND_SPEED :
        (motion == motion_walk_forward_fast ? 2000U : 1200U);
    if ((motion == motion_qr_both_arms || motion == motion_qr_left_arm ||
         motion == motion_qr_right_arm) && (id == 11U || id == 12U))
    { expected = 2600U; }
    CHECK(speed == expected);
    CHECK(acceleration == (motion == motion_stand ? SERVO_STARTUP_STAND_ACCELERATION : 250U));
    s_attemptedTargets[id] = angle; /* Includes rejected targets: interpolation trace. */
    return BaseSetTarget(id, speed, acceleration, angle);
}

uint8_t Servo_SendTargets(void)
{
    uint8_t inject = s_inject; s_inject = 0U;
    if (inject == 1U || inject == 3U) { Command(CAMERA_WALK_BLOCKED, 0U); }
    if (inject == 2U || inject == 3U) { Shape(1U); }
    return BaseSendTargets();
}

static void Step(void)
{
    RobotControlStatus_t before = RobotControl_GetStatus(), after;
    uint8_t active = MotionPlayer_IsActive(), id;
    float old[SERVO_COUNT + 1];
    memcpy(old, s_attemptedTargets, sizeof(old));
    Tick(); after = RobotControl_GetStatus();
    /* Independent oracle: observe traversal of the two verified endpoints,
     * not completedCycles and not the production callback. */
    if (active && MotionPlayer_IsForwardMotion(before.motion) &&
        (before.walkPhase == ROBOT_WALK_MOVING || before.walkPhase == ROBOT_WALK_TURN_APPROACH) &&
        (before.frameIndex == 6U || before.frameIndex == 10U) &&
        s_servoAccepted && !s_rejectJoint &&
        (after.motion != before.motion || after.frameIndex != before.frameIndex || !MotionPlayer_IsActive()))
    { s_countOracle++; }
    CHECK(after.forwardSteps == s_countOracle);
    CHECK(after.forwardSteps == s_stepLogs);
    CHECK(after.forwardSteps >= before.forwardSteps);
    CHECK(before.gaitStage != ROBOT_GAIT_FAST || after.gaitStage == ROBOT_GAIT_FAST);
    if (after.gaitStage == ROBOT_GAIT_FAST)
    {
        CHECK(!after.rightSampleWindowOpen);
        CHECK(after.walkPhase != ROBOT_WALK_TURN_APPROACH && after.walkPhase != ROBOT_WALK_TURN_HANDOFF);
        CHECK(after.motion != motion_walk_forward);
    }
    for (id = 1U; id <= SERVO_ACTIVE_COUNT; ++id)
    { CHECK(fabsf(s_attemptedTargets[id] - old[id]) <= 5.0f); }
}

static void Advance(uint32_t ms)
{ uint32_t began = s_tick; while (s_tick - began < ms) { Step(); } }
static void Result(uint8_t cmd)
{ Command(cmd, cmd == CAMERA_WALK_FORWARD ? 2U :
    (cmd == CAMERA_WALK_HOLD || cmd == CAMERA_WALK_BLOCKED ? 0U : 3U)); RobotControl_Update(0U); }
static void Two(uint8_t cmd)
{ Step(); Result(cmd); Advance(520U); Result(cmd); Step(); }
static void Phase(RobotWalkPhase_t phase)
{
    uint32_t began = s_tick;
    while (RobotControl_GetStatus().walkPhase != phase)
    { CHECK(s_tick - began < 300000U); Step(); }
}
static void Count(uint32_t count)
{
    uint32_t began = s_tick;
    while (RobotControl_GetStatus().forwardSteps < count)
    { CHECK(s_tick - began < 300000U); Step(); }
    CHECK(RobotControl_GetStatus().forwardSteps == count);
}
static void Frame(uint8_t frame)
{
    uint32_t began = s_tick;
    while (!MotionPlayer_IsForwardMotion(MotionPlayer_GetMotion()) || MotionPlayer_GetFrameIndex() != frame)
    { CHECK(s_tick - began < 300000U); Step(); }
}
static void Window(void)
{
    uint32_t began = s_tick;
    while (!RobotControl_GetStatus().sampleWindowOpen || MotionPlayer_GetFrameIndex() != 10U)
    { CHECK(s_tick - began < 300000U); Step(); }
    CHECK(MotionPlayer_GetFrameIndex() == 10U);
}
static void Reset(void)
{
    memset(s_targets, 0, sizeof(s_targets)); memset(s_attemptedTargets, 0, sizeof(s_attemptedTargets));
    ResetLink(); RobotControl_Init();
    s_countOracle = s_stepLogs = s_switchLogs = s_thresholdLogs = 0U; s_inject = 0U;
    Phase(ROBOT_WALK_MEASURING); Advance(5000U);
    CHECK(RobotControl_GetStatus().forwardSteps == 0U);
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().gaitStage == ROBOT_GAIT_SLOW);
}
static void Start(void)
{ Reset(); Two(CAMERA_WALK_FORWARD); CHECK(MotionPlayer_GetMotion() == motion_walk_forward); }
static void Fast(void)
{
    uint32_t began = s_tick;
    while (RobotControl_GetStatus().gaitStage != ROBOT_GAIT_FAST)
    { CHECK(s_tick - began < 300000U); Step(); }
    CHECK(s_switchLogs == 1U && s_thresholdLogs == 1U);
}
static void ResumeFast(void)
{
    Two(CAMERA_WALK_FORWARD);
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward_fast);
    CHECK(RobotControl_GetStatus().gaitStage == ROBOT_GAIT_FAST);
}

static void TestThresholdAndFastContinuation(void)
{
    uint32_t count, began;
    Start(); CHECK(!RobotControl_BeginNewRun()); Count(29U);
    CHECK(!RobotControl_GetStatus().fastSwitchPending);
    Count(30U);
    CHECK(RobotControl_GetStatus().gaitStage == ROBOT_GAIT_SLOW);
    CHECK(RobotControl_GetStatus().fastSwitchPending);
    Phase(ROBOT_WALK_TO_STAND);
    CHECK(MotionPlayer_GetMotion() == motion_walk_tostand);
    Fast(); CHECK(RobotControl_GetStatus().fastSwitchSteps == 30U);
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward_fast);
    Count(34U); Frame(11U);
    began = s_tick; count = RobotControl_GetStatus().forwardSteps;
    while (RobotControl_GetStatus().forwardSteps == count) { Step(); }
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward_fast);
    CHECK(s_tick - began == RangeTime(motion_walk_forward_fast,11U,12U) + RangeTime(motion_walk_forward_fast,4U,6U));
    puts("PASS threshold 29/30, original return, switch at 30, fast 12->4 continuation, speed isolation");
}

static void TestRetry(uint8_t frame, uint8_t jointFailure)
{
    uint32_t before;
    Start(); Frame(frame); before = RobotControl_GetStatus().forwardSteps;
    if (jointFailure) { s_rejectJoint = 4U; } else { s_servoAccepted = 0U; }
    Advance(1200U); CHECK(RobotControl_GetStatus().forwardSteps == before);
    CHECK(MotionPlayer_GetFrameIndex() == frame);
    s_rejectJoint = 0U; s_servoAccepted = 1U; Step();
    CHECK(RobotControl_GetStatus().forwardSteps == before + 1U);
    RobotControl_Update(0U); RobotControl_Update(0U); Advance(100U);
    CHECK(RobotControl_GetStatus().forwardSteps == before + 1U);
    puts("PASS failed joint/queue endpoint retries count once only");
}

static void TestShapeAt12(void)
{
    Start(); Count(12U); Shape(1U); RobotControl_Update(0U);
    Phase(ROBOT_WALK_EVENT); CHECK(RobotControl_GetStatus().forwardSteps == 12U);
    Phase(ROBOT_WALK_MEASURING); CHECK(RobotControl_GetStatus().forwardSteps == 12U);
    CHECK(RobotControl_GetStatus().gaitStage == ROBOT_GAIT_SLOW);
    Two(CAMERA_WALK_FORWARD); Count(13U);
    puts("PASS shape at 12, shape extra cycles excluded, recovery next step is 13");
}

static void TestEveryShapeExcluded(void)
{
    uint8_t shape;
    for (shape = 1U; shape <= 6U; ++shape)
    {
        Start(); Count(4U); Shape(shape); RobotControl_Update(0U);
        Phase(ROBOT_WALK_EVENT);
        /* This is an accounting test for unchanged shape tables. The existing
         * both-arms 180deg/600ms return is 6deg/20ms: do not impose the walking
         * continuity assertion on it or change its tuned motion to pass a test. */
        {
            uint32_t began = s_tick;
            while (RobotControl_GetStatus().walkPhase != ROBOT_WALK_MEASURING)
            {
                CHECK(s_tick - began < 30000U); Tick();
                CHECK(RobotControl_GetStatus().forwardSteps == 4U);
                CHECK(s_stepLogs == 4U);
            }
        }
        CHECK(RobotControl_GetStatus().forwardSteps == 4U);
        CHECK(RobotControl_GetStatus().eventsCompleted == 1U);
        Two(CAMERA_WALK_FORWARD); Count(5U);
    }
    puts("PASS all six shape actions, leg lifts and optional shape walking excluded");
}

static uint32_t s_rawSteps;
static void RawFrame(MotionId_t motion, uint8_t frame)
{
    if (MotionPlayer_IsForwardMotion(motion) && (frame == 6U || frame == 10U))
    { s_rawSteps++; }
}
static void TestPlayerBothGaits(void)
{
    MotionId_t gait;
    uint8_t n;
    for (n = 0U; n < 2U; ++n)
    {
        gait = n ? motion_walk_forward_fast : motion_walk_forward;
        ResetLink(); MotionPlayer_Init(); MotionPlayer_SetFrameCompletedCallback(RawFrame);
        s_rawSteps = 0U;
        CHECK(MotionPlayer_StartCycles(gait, 1U, 4U, 11U, 11U));
        while (MotionPlayer_IsActive()) { (void)MotionPlayer_Update(20U); }
        CHECK(s_rawSteps == 2U); (void)MotionPlayer_Update(20U); CHECK(s_rawSteps == 2U);
        CHECK(MotionPlayer_ContinueWalkCycles(1U)); CHECK(MotionPlayer_GetFrameIndex() == 12U);
        if (n) { CHECK(!MotionPlayer_ArmWalkHandoff(10U)); }
        while (MotionPlayer_IsActive()) { (void)MotionPlayer_Update(20U); }
        CHECK(s_rawSteps == 4U);
        MotionPlayer_Start(gait); s_rawSteps = 0U;
        while (MotionPlayer_GetCompletedCycles() < 1U) { (void)MotionPlayer_Update(20U); }
        CHECK(MotionPlayer_IsActive() && MotionPlayer_GetFrameIndex() == 4U);
        CHECK(s_rawSteps == 2U);
        /* Non-walking ranges cannot emit forward-foot events. */
        CHECK(MotionPlayer_StartRange(motion_band_over,0U,0U));
        while (MotionPlayer_IsActive()) { (void)MotionPlayer_Update(20U); }
        CHECK(s_rawSteps == 2U);
        CHECK(MotionPlayer_StartRange(motion_left_translation,0U,2U));
        while (MotionPlayer_IsActive()) { (void)MotionPlayer_Update(20U); }
        CHECK(s_rawSteps == 2U);
    }
    puts("PASS both player IDs: finite/infinite/continuation boundaries, fast handoff rejection, non-walk exclusion");
}

static void TestThresholdPriority(uint8_t injection)
{
    Start(); Count(29U);
#if ROBOT_WALK_TURN_HANDOFF_ENABLE && ROBOT_WALK_DECIDE_EACH_CYCLE
    /* Keep a genuinely confirmed RIGHT pending at the same 30th endpoint. */
    while (!RobotControl_GetStatus().rightSampleWindowOpen) { Step(); }
    Two(CAMERA_WALK_RIGHT);
    CHECK(RobotControl_GetStatus().rightSampleCount == 2U);
#endif
    Frame(10U);
    Advance(RangeTime(motion_walk_forward,10U,10U) - 20U);
    s_inject = injection; Step();
    CHECK(RobotControl_GetStatus().forwardSteps == 30U);
    CHECK(RobotControl_GetStatus().fastSwitchPending);
    CHECK(!RobotControl_GetStatus().turnFromWalk);
    if (injection == 1U || injection == 3U)
    {
        Phase(ROBOT_WALK_BLOCKED);
        CHECK(RobotControl_GetStatus().gaitStage == ROBOT_GAIT_SLOW);
        CHECK(RobotControl_GetStatus().eventsStarted == 0U);
        CHECK(RobotControl_GetStatus().fastSwitchPending);
        Advance(1000U); Two(CAMERA_WALK_FORWARD);
    }
    if (injection == 2U || injection == 3U)
    {
        Phase(ROBOT_WALK_EVENT);
        CHECK(RobotControl_GetStatus().gaitStage == ROBOT_GAIT_SLOW);
        CHECK(RobotControl_GetStatus().forwardSteps == 30U);
        Phase(ROBOT_WALK_MEASURING);
        CHECK(RobotControl_GetStatus().eventsCompleted == 1U);
        ResumeFast();
    }
    CHECK(RobotControl_GetStatus().gaitStage == ROBOT_GAIT_FAST);
    CHECK(RobotControl_GetStatus().fastSwitchSteps == 30U);
    puts("PASS exact 30th endpoint STOP/shape priority, sticky request and authorized fast resume");
}

static void TestFastTurn(uint8_t direction, uint8_t cancel)
{
    uint32_t count, began;
    Start(); Fast(); Window(); began = s_tick;
    Two(direction); Phase(ROBOT_WALK_TO_STAND);
    CHECK(s_tick - began == RangeTime(motion_walk_forward_fast,10U,11U));
    count = RobotControl_GetStatus().forwardSteps;
    if (cancel) { Two(CAMERA_WALK_FORWARD); }
    if (cancel == 0U)
    {
        Phase(ROBOT_WALK_MOVING);
        CHECK(MotionPlayer_GetMotion() == (direction == CAMERA_WALK_LEFT ? motion_turn_left : motion_turn_right));
        CHECK(MotionPlayer_GetFrameIndex() == 0U);
        CHECK(!RobotControl_GetStatus().turnFromWalk);
        Phase(ROBOT_WALK_MEASURING);
        CHECK(RobotControl_GetStatus().forwardSteps == count);
        ResumeFast();
    }
    else
    {
        Phase(ROBOT_WALK_MOVING); CHECK(MotionPlayer_GetMotion() == motion_walk_forward_fast);
    }
    puts("PASS fast sample duration, full standing turn, fresh FWD cancellation and fast recovery");
}

static void TestFastStopShapeAndNewRun(void)
{
    uint32_t count;
    Start(); Fast(); Count(32U); Shape(2U); Result(CAMERA_WALK_BLOCKED);
    Phase(ROBOT_WALK_BLOCKED); count = RobotControl_GetStatus().forwardSteps;
    CHECK(RobotControl_GetStatus().gaitStage == ROBOT_GAIT_FAST);
    CHECK(!RobotControl_BeginNewRun()); /* queued shape is never discarded */
    Two(CAMERA_WALK_FORWARD); Phase(ROBOT_WALK_EVENT); Phase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().forwardSteps == count);
    ResumeFast(); Count(count + 1U); Result(CAMERA_WALK_HOLD); Phase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().gaitStage == ROBOT_GAIT_FAST);
    CHECK(RobotControl_BeginNewRun());
    s_countOracle = s_stepLogs = s_switchLogs = s_thresholdLogs = 0U;
    CHECK(RobotControl_GetStatus().forwardSteps == 0U);
    CHECK(RobotControl_GetStatus().gaitStage == ROBOT_GAIT_SLOW);
    Phase(ROBOT_WALK_MEASURING); Advance(5000U);
    CHECK(MotionPlayer_GetMotion() == motion_stand);
    Two(CAMERA_WALK_FORWARD); Count(1U);
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
    puts("PASS fast STOP/shape persistence, guarded explicit new run, startup authorization preserved");
}

#if ROBOT_WALK_TURN_HANDOFF_ENABLE && ROBOT_WALK_DECIDE_EACH_CYCLE
static void Early(void)
{
    uint32_t began = s_tick;
    while (!RobotControl_GetStatus().rightSampleWindowOpen)
    { CHECK(s_tick - began < 300000U); Step(); }
}
static void TestSlowHandoffLedger(void)
{
    uint32_t count;
    Start(); Early(); Two(CAMERA_WALK_RIGHT); Phase(ROBOT_WALK_TURN_HANDOFF);
    CHECK(RobotControl_GetStatus().forwardSteps == 2U);
    Phase(ROBOT_WALK_MEASURING); Two(CAMERA_WALK_FORWARD); Count(3U);
    Start(); Window(); Two(CAMERA_WALK_LEFT); Phase(ROBOT_WALK_TURN_APPROACH);
    CHECK(RobotControl_GetStatus().forwardSteps == 2U);
    Phase(ROBOT_WALK_TURN_HANDOFF); CHECK(RobotControl_GetStatus().forwardSteps == 3U);
    Phase(ROBOT_WALK_MEASURING); Two(CAMERA_WALK_FORWARD); Count(4U);
    /* Odd total makes step 30 a RIGHT endpoint; required following LEFT is 31. */
    Count(29U); Count(30U); CHECK(RobotControl_GetStatus().fastSwitchPending);
    Fast(); CHECK(RobotControl_GetStatus().fastSwitchSteps == 31U);
    CHECK(RobotControl_GetStatus().forwardSteps == 31U);
    Start(); Window(); Two(CAMERA_WALK_LEFT); Phase(ROBOT_WALK_TURN_APPROACH);
    Two(CAMERA_WALK_FORWARD); Count(4U);
    CHECK(!RobotControl_GetStatus().turnFromWalk);
    Start(); Window(); Two(CAMERA_WALK_LEFT); Phase(ROBOT_WALK_TURN_APPROACH);
    Result(CAMERA_WALK_HOLD); Phase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().forwardSteps == 4U);
    count = RobotControl_GetStatus().forwardSteps; Advance(1000U);
    CHECK(RobotControl_GetStatus().forwardSteps == count);
    puts("PASS slow right/left handoff ledger, cancel/fallback, odd parity safe switch at 31 honestly recorded");
}
static void TestThresholdTurn(uint8_t direction)
{
    Start(); Count(28U);
    if (direction == CAMERA_WALK_RIGHT) { Early(); } else { Window(); }
    Two(direction); Phase(ROBOT_WALK_TURN_HANDOFF);
    CHECK(RobotControl_GetStatus().gaitStage == ROBOT_GAIT_SLOW);
    CHECK(RobotControl_GetStatus().fastSwitchPending);
    CHECK(RobotControl_GetStatus().forwardSteps == (direction == CAMERA_WALK_RIGHT ? 30U : 31U));
    Phase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().gaitStage == ROBOT_GAIT_FAST);
    ResumeFast();
    puts("PASS confirmed turn at threshold completes before phase transition; extra approach step counted");
}
#endif

int main(void)
{
    TestPlayerBothGaits();
    TestThresholdAndFastContinuation();
    TestRetry(6U,0U); TestRetry(10U,0U); TestRetry(6U,1U); TestRetry(10U,1U);
    TestShapeAt12();
    TestEveryShapeExcluded();
    TestThresholdPriority(1U); TestThresholdPriority(2U); TestThresholdPriority(3U);
    TestFastTurn(CAMERA_WALK_LEFT,0U); TestFastTurn(CAMERA_WALK_RIGHT,0U);
    TestFastTurn(CAMERA_WALK_RIGHT,1U);
    TestFastStopShapeAndNewRun();
#if ROBOT_WALK_TURN_HANDOFF_ENABLE && ROBOT_WALK_DECIDE_EACH_CYCLE
    TestSlowHandoffLedger(); TestThresholdTurn(CAMERA_WALK_LEFT); TestThresholdTurn(CAMERA_WALK_RIGHT);
#endif
    puts("ALL DUAL GAIT TESTS PASSED; software completions only; hardware NOT tested");
    return 0;
}
