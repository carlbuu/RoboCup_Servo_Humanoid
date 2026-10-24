/* Real parser/player/scheduler; hardware submission is stubbed. The existing
 * recovery table and its original 5-degree test are deliberately unchanged. */
#define main legacy_three_modes_main
#define Debug_Printf Legacy_Debug_Printf
#include "test_three_modes.c"
#undef Debug_Printf
#undef main

static uint32_t s_boundaryLogs, s_sampleLogs, s_continueLogs;
/* Independent expected schedule from the current table, rounded per frame to
 * the host's 20ms control tick. Keep exact equality checks at the boundary. */
static uint32_t SampleWindowTime(void)
{
    return RangeTime(motion_walk_forward, ROBOT_WALK_SAMPLE_START_FRAME,
                     ROBOT_WALK_SAMPLE_END_FRAME);
}
void Debug_Printf(char *format, ...)
{
    va_list arguments;
    if (strstr(format, "[BOUNDARY]")) { s_boundaryLogs++; }
    if (strstr(format, "[SAMPLE]")) { s_sampleLogs++; }
    if (strstr(format, "return=0 stand=0")) { s_continueLogs++; }
    va_start(arguments, format); vprintf(format, arguments); va_end(arguments);
}

static void SendLateCommand(uint8_t command, uint8_t arg)
{
    uint8_t bytes[64];
    uint16_t length;
#if CAMERA_PROTOCOL_BINARY
    bytes[0]=0xAAU; bytes[1]=0x57U; bytes[2]=command;
    bytes[3]=arg; bytes[4]=(uint8_t)(command^arg); bytes[5]=0x0AU; length=6U;
#else
    static const char * const names[8] =
    {"HOLD","FWD","TURNL","TURNR","BLOCKED","BACK","SHIFTL","SHIFTR"};
    length=(uint16_t)sprintf((char *)bytes,"CMD,%s,%u\n",names[command],(unsigned int)arg);
#endif
    Camera_Process(bytes,length);
}
static void Votes(uint8_t command, uint8_t arg, uint8_t count)
{
    uint8_t i;
    for(i=0U;i<count;i++) { Tick(); SendLateCommand(command,arg); RobotControl_Update(0U); }
    Tick();
}
static void Ready(void)
{
    ResetRobot();
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    WaitPhase(ROBOT_WALK_MEASURING);
#else
    WaitPhase(ROBOT_WALK_MOVING);
#endif
}
static void StartOrdinary(void)
{
    Ready();
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    Votes(CAMERA_WALK_FORWARD,2U,ROBOT_WALK_VOTE_NEED);
#endif
    CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_MOVING);
    CHECK(RobotControl_GetStatus().targetCycles==2U);
}
static void WaitWindow(void)
{
    uint32_t began=s_tick;
    while(!RobotControl_GetStatus().sampleWindowOpen) { CHECK(s_tick-began<20000U); Tick(); }
    CHECK(MotionPlayer_GetFrameIndex()==10U);
#if ROBOT_WALK_DECIDE_EACH_CYCLE
    CHECK(MotionPlayer_GetCompletedCycles()==0U);
#else
    CHECK(MotionPlayer_GetCompletedCycles()==1U);
#endif
}
#if !ROBOT_WALK_DECIDE_EACH_CYCLE
static void TestLateWindowAndContinuation(void)
{
    uint32_t batch, began, seq;
    StartOrdinary();
    while(MotionPlayer_GetFrameIndex()<10U) { Tick(); }
    Votes(CAMERA_WALK_LEFT,255U,5U); /* first-cycle frame10 is deliberately ignored */
    CHECK(!RobotControl_GetStatus().sampleWindowOpen);
    CHECK(RobotControl_GetStatus().sampleCount==0U);
    CHECK(MotionPlayer_IsActive());
    WaitWindow(); began=s_tick; batch=RobotControl_GetStatus().walkBatch;
    Votes(CAMERA_WALK_FORWARD,2U,3U); seq=Camera_GetStatus().receivedTick;
    CHECK(seq>began); CHECK(RobotControl_GetStatus().sampleCount==ROBOT_WALK_VOTE_WINDOW);
    while(RobotControl_GetStatus().walkBatch==batch)
    {
        CHECK(MotionPlayer_GetMotion()==motion_walk_forward);
        Tick();
    }
    CHECK(s_tick-began==SampleWindowTime());
    CHECK(MotionPlayer_GetCompletedCycles()==2U);
    CHECK(MotionPlayer_GetFrameIndex()==12U);
    CHECK(MotionPlayer_IsActive()); CHECK(s_continueLogs>0U);
    began=s_tick;
    while(MotionPlayer_GetFrameIndex()==12U) { Tick(); }
    CHECK(s_tick-began==720U); CHECK(MotionPlayer_GetFrameIndex()==4U);
    puts("PASS second-cycle-only table-timed window; FWD keeps 11->12->4, no stand/restart");
}
#else
static void TestLateWindowAndContinuation(void)
{
    uint32_t batch, began, walking;
    StartOrdinary(); walking=s_tick; batch=RobotControl_GetStatus().walkBatch;
    Votes(CAMERA_WALK_RIGHT,3U,2U); /* before frame10: never an accepted candidate */
    CHECK(RobotControl_GetStatus().sampleCount==0U);
    WaitWindow(); began=s_tick;
    CHECK(s_tick-walking==WalkBatchTime(1U)-SampleWindowTime());
    Votes(CAMERA_WALK_FORWARD,2U,2U);
    while(MotionPlayer_GetCompletedCycles()<1U)
    {
        CHECK(MotionPlayer_GetMotion()==motion_walk_forward); Tick();
    }
    CHECK(s_tick-began==SampleWindowTime());
    CHECK(RobotControl_GetStatus().walkBatch==batch);
    CHECK(RobotControl_GetStatus().targetCycles==2U);
    CHECK(MotionPlayer_GetFrameIndex()==12U);
    CHECK(MotionPlayer_IsActive());
    CHECK(!RobotControl_GetStatus().sampleWindowOpen);
    CHECK(RobotControl_GetStatus().sampleCount==0U);
    began=s_tick;
    while(MotionPlayer_GetFrameIndex()==12U) { Tick(); }
    CHECK(s_tick-began==720U); CHECK(MotionPlayer_GetFrameIndex()==4U);
    while(!RobotControl_GetStatus().sampleWindowOpen)
    {
        CHECK(MotionPlayer_GetMotion()==motion_walk_forward);
        CHECK(MotionPlayer_GetFrameIndex()>=4U); Tick();
    }
    CHECK(MotionPlayer_GetCompletedCycles()==1U);
    CHECK(MotionPlayer_GetFrameIndex()==10U);
    Votes(CAMERA_WALK_FORWARD,2U,2U);
    while(RobotControl_GetStatus().walkBatch==batch)
    {
        CHECK(MotionPlayer_GetMotion()==motion_walk_forward);
        CHECK(MotionPlayer_GetFrameIndex()>=10U); Tick();
    }
    CHECK(s_tick-walking==WalkBatchTime(2U));
    CHECK(MotionPlayer_GetCompletedCycles()==2U);
    CHECK(MotionPlayer_GetFrameIndex()==12U);
    CHECK(RobotControl_GetStatus().targetCycles==2U);
    CHECK(s_continueLogs>0U);
    puts("PASS each cycle has table-timed window; current four-step duration and 11->12->4 preserved, no stand");
}
static void TestCycleVotesCannotLeak(void)
{
    uint32_t began;
    StartOrdinary(); WaitWindow(); Votes(CAMERA_WALK_RIGHT,3U,1U);
    while(MotionPlayer_GetCompletedCycles()<1U) { Tick(); }
    CHECK(MotionPlayer_GetMotion()==motion_walk_forward);
    CHECK(MotionPlayer_GetFrameIndex()==12U);
    CHECK(RobotControl_GetStatus().sampleCount==0U);
    while(!RobotControl_GetStatus().sampleWindowOpen) { Tick(); }
    CHECK(MotionPlayer_GetCompletedCycles()==1U); began=s_tick;
    Votes(CAMERA_WALK_RIGHT,3U,1U);
    CHECK(RobotControl_GetStatus().sampleCount==1U);
    while(s_tick-began<520U) { Tick(); }
    Votes(CAMERA_WALK_RIGHT,3U,1U);
    CHECK(RobotControl_GetStatus().sampleCount==2U);
    CHECK(MotionPlayer_GetMotion()==motion_walk_forward);
    WaitPhase(ROBOT_WALK_TO_STAND);
    CHECK(s_tick-began==SampleWindowTime());
    WaitPhase(ROBOT_WALK_MOVING);
    CHECK(MotionPlayer_GetMotion()==motion_turn_right);
    puts("PASS first-cycle single RIGHT cannot count toward second-cycle confirmation; two new RIGHT samples required");
}
#endif
static void TestCachedCorrection(uint8_t command, MotionId_t motion)
{
    uint32_t batch;
    StartOrdinary(); WaitWindow(); batch=RobotControl_GetStatus().walkBatch;
    Votes(command,255U,3U);
    WaitPhase(ROBOT_WALK_TO_STAND);
    CHECK(RobotControl_GetStatus().walkBatch==batch);
    CHECK(MotionPlayer_GetCompletedCycles()==0U); /* recovery is a different playback */
    Votes(CAMERA_WALK_LEFT,3U,5U); /* a new correction cannot replace the frozen action */
    WaitPhase(ROBOT_WALK_MOVING);
    CHECK(MotionPlayer_GetMotion()==motion);
    CHECK(RobotControl_GetStatus().targetCycles==1U);
    WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(MotionPlayer_GetMotion()==motion); CheckStand();
    puts("PASS sampled correction survives return; single action; no duplicate stand");
}
static void TestSplitOldAndLatestVote(void)
{
    uint8_t bytes[64]; uint16_t length; uint32_t old;
    StartOrdinary(); old=s_tick;
    length=CommandBytes(bytes,CAMERA_WALK_LEFT,3U);
    Camera_ProcessTimed(bytes,2U,old);
    WaitWindow();
    Camera_ProcessTimed(bytes+2U,(uint16_t)(length-2U),s_tick+1U);
    RobotControl_Update(0U);
    CHECK(RobotControl_GetStatus().sampleCount==0U);
    Votes(CAMERA_WALK_LEFT,3U,3U);
    Votes(CAMERA_WALK_RIGHT,3U,1U); /* latest disagrees: old majority is invalid */
    CHECK(RobotControl_GetStatus().sampleCount==ROBOT_WALK_VOTE_WINDOW);
    while(MotionPlayer_GetCompletedCycles()<(ROBOT_WALK_DECIDE_EACH_CYCLE ? 1U : 2U)) { Tick(); }
    CHECK(MotionPlayer_GetMotion()==motion_walk_forward);
    CHECK(MotionPlayer_GetFrameIndex()==12U);
    puts("PASS cross-window split frame rejected; latest disagreement invalidates winner");
}
static void TestStops(uint8_t command)
{
    uint16_t i;
    uint32_t walking;
    StartOrdinary(); walking=s_tick;
    SendLateCommand(command,0U);
    for(i=0U;i<30U;i++) { SendLateCommand(CAMERA_WALK_FORWARD,2U); }
    Tick(); CHECK(RobotControl_GetStatus().stopRequested);
    WaitPhase(ROBOT_WALK_TO_STAND);
    /* Only current cycle was completed; even an overflowing direction queue cannot swallow stop. */
    CHECK(s_tick-walking==WalkBatchTime(1U));
    WaitPhase(command==CAMERA_WALK_BLOCKED ? ROBOT_WALK_BLOCKED : ROBOT_WALK_MEASURING);
    for(i=0U;i<250U;i++) { Tick(); }
    CHECK(MotionPlayer_GetMotion()==motion_walk_tostand);
    Votes(CAMERA_WALK_FORWARD,2U,3U);
    CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_MOVING);
    CHECK(!RobotControl_GetStatus().stopRequested);
    puts("PASS stop survives ordinary FIFO overflow; current-cycle return; fresh-vote resume");
}
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN && ROBOT_SHAPE_STEP_BEFORE_ACTION != 0U
static void TestShapeAndStop(void)
{
    StartOrdinary(); Shape(1U); Shape(1U); Tick();
    CHECK(RobotControl_GetStatus().pendingShapes==1U);
    while(MotionPlayer_GetCompletedCycles()<1U) { Tick(); }
    CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_MOVING); /* finish full batch */
    WaitPhase(ROBOT_WALK_SHAPE_STEP);
    CHECK(RobotControl_GetStatus().targetCycles==1U);
    CHECK(RobotControl_GetStatus().eventsStarted==0U);
    SendLateCommand(CAMERA_WALK_HOLD,0U); Shape(1U); Tick();
    WaitPhase(ROBOT_WALK_SHAPE_STEP_RETURN);
    WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().pendingShapes==1U);
    CHECK(RobotControl_GetStatus().eventsStarted==0U);
    Votes(CAMERA_WALK_FORWARD,2U,3U);
    CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_EVENT); /* no second extra cycle */
    WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().eventsCompleted==1U);
    CHECK(RobotControl_GetStatus().pendingShapes==0U);
    CHECK(MotionPlayer_GetMotion()==motion_qr_left_arm);
    CheckStand();
    puts("PASS independent shape dedupe; full batch + exactly one extra cycle; stop postpones action");
}
#endif
static void TestSubmissionBoundary(void)
{
    uint16_t ticks;
    StartOrdinary(); WaitWindow(); Votes(CAMERA_WALK_FORWARD,2U,3U);
    while(MotionPlayer_GetFrameIndex()!=11U) { Tick(); }
    s_servoAccepted=0U;
    for(ticks=0U;ticks<40U;ticks++) { Tick(); }
    CHECK(MotionPlayer_GetCompletedCycles()==(ROBOT_WALK_DECIDE_EACH_CYCLE ? 0U : 1U));
    CHECK(RobotControl_GetStatus().sampleWindowOpen);
    s_servoAccepted=1U; Tick();
    CHECK(MotionPlayer_GetCompletedCycles()==(ROBOT_WALK_DECIDE_EACH_CYCLE ? 1U : 2U));
    CHECK(MotionPlayer_GetFrameIndex()==12U);
    puts("PASS final submission failure cannot advance cycle or execute boundary decision");
}
static void TestRightTwoStepThenOrdinaryBatch(void)
{
    uint32_t batch;
    StartOrdinary(); WaitWindow(); Votes(CAMERA_WALK_RIGHT,255U,3U);
    WaitPhase(ROBOT_WALK_TO_STAND); WaitPhase(ROBOT_WALK_MOVING);
    CHECK(MotionPlayer_GetMotion()==motion_turn_right);
    WaitPhase(ROBOT_WALK_MEASURING); Votes(CAMERA_WALK_FORWARD,2U,3U);
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    CHECK(RobotControl_GetStatus().targetCycles==1U);
#else
    CHECK(RobotControl_GetStatus().targetCycles==2U);
#endif
    batch=RobotControl_GetStatus().walkBatch;
    while(!RobotControl_GetStatus().sampleWindowOpen) { Tick(); }
    Votes(CAMERA_WALK_FORWARD,2U,3U);
    while(RobotControl_GetStatus().walkBatch==batch) { Tick(); }
    CHECK(RobotControl_GetStatus().targetCycles==2U);
    CHECK(MotionPlayer_GetFrameIndex()==12U);
    puts("PASS right-turn two-step exception returns to ordinary four-step batches");
}
static void StationaryReady(void)
{
    Ready();
#if ROBOT_RUN_MODE == ROBOT_MODE_LINE_DEBUG
    SendLateCommand(CAMERA_WALK_HOLD,0U); Tick();
    WaitPhase(ROBOT_WALK_MEASURING);
#endif
}
static void TestLowRateTwoFrameDecision(void)
{
    uint32_t opened, walking;
    uint8_t i;
    StartOrdinary(); walking=s_tick; WaitWindow(); opened=s_tick;
    while(s_tick-opened<260U) { Tick(); }
    SendLateCommand(CAMERA_WALK_RIGHT,3U); RobotControl_Update(0U);
    CHECK(RobotControl_GetStatus().sampleCount==1U);
    for(i=0U;i<10U;i++) { RobotControl_Update(0U); }
    CHECK(RobotControl_GetStatus().sampleCount==1U); /* one receive cannot count twice */
    while(s_tick-opened<780U) { Tick(); }
    SendLateCommand(CAMERA_WALK_RIGHT,3U); RobotControl_Update(0U);
    CHECK(RobotControl_GetStatus().sampleCount==2U);
    CHECK(MotionPlayer_GetMotion()==motion_walk_forward);
    WaitPhase(ROBOT_WALK_TO_STAND);
    CHECK(s_tick-opened==SampleWindowTime());
    CHECK(s_tick-walking==WalkBatchTime(ROBOT_WALK_DECIDE_EACH_CYCLE ? 1U : 2U));
    WaitPhase(ROBOT_WALK_MOVING);
    CHECK(MotionPlayer_GetMotion()==motion_turn_right);
    CHECK(RobotControl_GetStatus().targetCycles==1U);
    puts("PASS ~2fps: two distinct RIGHT samples 520ms apart adopted at safe boundary; first-cycle exit when enabled");
}
static void TestDirectionChangeBreaksConfirmation(void)
{
    StationaryReady(); Tick();
    SendLateCommand(CAMERA_WALK_LEFT,3U); Tick();
    CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_MEASURING);
    SendLateCommand(CAMERA_WALK_RIGHT,3U); Tick();
    SendLateCommand(CAMERA_WALK_LEFT,3U); Tick();
    CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_MEASURING);
    SendLateCommand(CAMERA_WALK_LEFT,3U); Tick();
    CHECK(MotionPlayer_GetMotion()==motion_turn_left);
    puts("PASS LEFT/RIGHT/LEFT cannot confirm; only final consecutive LEFT/LEFT starts one turn");
}
static void TestLowRateReturnCancellation(void)
{
    uint32_t began;
    StartOrdinary(); WaitWindow(); Votes(CAMERA_WALK_LEFT,3U,2U);
    WaitPhase(ROBOT_WALK_TO_STAND); began=s_tick;
    while(s_tick-began<320U) { Tick(); }
    SendLateCommand(CAMERA_WALK_FORWARD,2U); RobotControl_Update(0U);
    while(s_tick-began<840U) { Tick(); }
    SendLateCommand(CAMERA_WALK_FORWARD,2U); RobotControl_Update(0U);
    while(MotionPlayer_GetMotion()==motion_walk_tostand) { Tick(); }
    CHECK(s_tick-began==1300U);
    CHECK(MotionPlayer_GetMotion()==motion_walk_forward);
    puts("PASS ~2fps: two fresh FWD samples cancel pending turn during original return duration");
}
static void TestLatestSampleInvalidatesStationaryDecision(void)
{
    StationaryReady(); Tick();
    /* Several frames may be parsed before the lower-priority RobotTask runs. */
    SendLateCommand(CAMERA_WALK_LEFT,3U);
    SendLateCommand(CAMERA_WALK_LEFT,3U);
    SendLateCommand(CAMERA_WALK_LEFT,3U);
    SendLateCommand(CAMERA_WALK_FORWARD,2U);
    Tick();
    CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_MEASURING);
    CHECK(MotionPlayer_GetMotion()!=motion_turn_left);
    Votes(CAMERA_WALK_FORWARD,2U,3U);
    CHECK(MotionPlayer_GetMotion()==motion_walk_forward);
    puts("PASS latest FWD invalidates stale stationary TURNL decision");
}
static void TestConfirmedForwardCancelsUnexecutedCorrection(void)
{
    uint32_t began;
    StartOrdinary(); WaitWindow(); Votes(CAMERA_WALK_LEFT,3U,3U);
    WaitPhase(ROBOT_WALK_TO_STAND); began=s_tick;
    Votes(CAMERA_WALK_FORWARD,2U,3U);
    while(MotionPlayer_GetMotion()==motion_walk_tostand) { Tick(); }
    CHECK(MotionPlayer_GetMotion()==motion_walk_forward);
    CHECK(s_tick-began==1300U);
    CHECK(RobotControl_GetStatus().targetCycles==2U);
    puts("PASS confirmed fresh FWD samples during return cancel pending turn without extra standing wait");
}
static void TestSingleForwardDoesNotCancelCorrection(void)
{
    StartOrdinary(); WaitWindow(); Votes(CAMERA_WALK_LEFT,3U,3U);
    WaitPhase(ROBOT_WALK_TO_STAND); Votes(CAMERA_WALK_FORWARD,2U,1U);
    WaitPhase(ROBOT_WALK_MOVING);
    CHECK(MotionPlayer_GetMotion()==motion_turn_left);
    WaitPhase(ROBOT_WALK_MEASURING); Tick();
    /* Advice from before this pose boundary cannot authorize a second turn. */
    {
        uint8_t bytes[64]; uint16_t length=CommandBytes(bytes,CAMERA_WALK_LEFT,3U);
        uint8_t i;
        for(i=0U;i<3U;i++) { Camera_ProcessTimed(bytes,length,s_tick-2000U); }
        Tick(); CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_MEASURING);
    }
    SendLateCommand(CAMERA_WALK_LEFT,3U);
    SendLateCommand(CAMERA_WALK_LEFT,3U);
    SendLateCommand(CAMERA_WALK_LEFT,3U);
    SendLateCommand(CAMERA_WALK_FORWARD,2U);
    Tick(); CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_MEASURING);
    Votes(CAMERA_WALK_FORWARD,2U,3U);
    CHECK(MotionPlayer_GetMotion()==motion_walk_forward);
    puts("PASS single FWD cannot cancel; completed turn discards old votes and cannot repeat stale decision");
}
static void TestStopPreemptsReturnCancellation(uint8_t stop)
{
    StartOrdinary(); WaitWindow(); Votes(CAMERA_WALK_LEFT,3U,3U);
    WaitPhase(ROBOT_WALK_TO_STAND); Votes(CAMERA_WALK_FORWARD,2U,3U);
    SendLateCommand(stop,0U);
    SendLateCommand(CAMERA_WALK_FORWARD,2U);
    SendLateCommand(CAMERA_WALK_FORWARD,2U);
    SendLateCommand(CAMERA_WALK_FORWARD,2U);
    Tick();
    WaitPhase(stop==CAMERA_WALK_BLOCKED ? ROBOT_WALK_BLOCKED : ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().stopRequested);
    CHECK(MotionPlayer_GetMotion()==motion_walk_tostand);
    puts("PASS HOLD/BLOCKED override both cached turn and return FWD cancellation");
}
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
static void TestShapeDuringSampling(void)
{
    uint32_t walking;
    StartOrdinary(); walking=s_tick; WaitWindow(); Votes(CAMERA_WALK_RIGHT,3U,2U);
    Shape(1U); Tick();
    CHECK(MotionPlayer_GetMotion()==motion_walk_forward);
    CHECK(RobotControl_GetStatus().pendingShapes==1U);
    CHECK(RobotControl_GetStatus().sampleCount==0U);
    WaitPhase(ROBOT_WALK_TO_STAND);
    CHECK(s_tick-walking==WalkBatchTime((ROBOT_SHAPE_STEP_BEFORE_ACTION ||
        !ROBOT_WALK_DECIDE_EACH_CYCLE) ? 2U : 1U));
#if ROBOT_SHAPE_STEP_BEFORE_ACTION
    WaitPhase(ROBOT_WALK_SHAPE_STEP);
#endif
    WaitPhase(ROBOT_WALK_EVENT);
    CHECK(MotionPlayer_GetMotion()==MotionGroups_FromQr(1U));
    CHECK(RobotControl_GetStatus().eventsStarted==1U);
    WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().eventsCompleted==1U);
    puts("PASS shape during confirmed RIGHT window cancels correction; configured safe cycle/batch and event path preserved");
}
static void TestStopAndShapeCannotResumeFromInvalidVote(void)
{
    StartOrdinary(); SendLateCommand(CAMERA_WALK_BLOCKED,0U); Shape(1U); Tick();
    WaitPhase(ROBOT_WALK_BLOCKED); Tick();
    SendLateCommand(CAMERA_WALK_LEFT,3U);
    SendLateCommand(CAMERA_WALK_LEFT,3U);
    SendLateCommand(CAMERA_WALK_LEFT,3U);
    SendLateCommand(CAMERA_WALK_FORWARD,2U);
    Tick();
    CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_BLOCKED);
    CHECK(RobotControl_GetStatus().pendingShapes==1U);
    CHECK(RobotControl_GetStatus().stopRequested);
    puts("PASS final invalid vote cannot clear stop or start queued shape extra cycle");
}
static void TestShapePreemptsReturnCancellation(void)
{
    StartOrdinary(); WaitWindow(); Votes(CAMERA_WALK_LEFT,3U,3U);
    WaitPhase(ROBOT_WALK_TO_STAND); Votes(CAMERA_WALK_FORWARD,2U,3U);
    Shape(1U); Tick();
#if ROBOT_SHAPE_STEP_BEFORE_ACTION != 0U
    WaitPhase(ROBOT_WALK_SHAPE_STEP);
    CHECK(RobotControl_GetStatus().targetCycles==1U);
#endif
    WaitPhase(ROBOT_WALK_EVENT);
    CHECK(RobotControl_GetStatus().eventsStarted==1U);
    WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().eventsCompleted==1U);
    puts("PASS queued shape preempts cancellation and follows configured direct/extra action path");
}
#endif
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN && ROBOT_SHAPE_STEP_BEFORE_ACTION == 0U
static void TestDirectShapesSafeCycleAndRecovery(void)
{
    uint8_t shape;
    for(shape=1U;shape<=6U;shape++)
    {
        uint32_t began, returned;
        StartOrdinary(); began=s_tick;
        while(MotionPlayer_GetFrameIndex()<3U) { Tick(); }
        Shape(shape); Shape(shape); Tick();
        CHECK(RobotControl_GetStatus().pendingShapes==1U);
        CHECK(RobotControl_GetStatus().eventsStarted==0U);
        WaitPhase(ROBOT_WALK_TO_STAND); returned=s_tick;
        CHECK(returned-began==WalkBatchTime(1U)); /* current cycle, no half-support interruption */
        WaitPhase(ROBOT_WALK_EVENT);
        CHECK(s_tick-returned==RangeTime(motion_walk_tostand,0U,1U));
        CHECK(MotionPlayer_GetMotion()==MotionGroups_FromQr(shape));
        CHECK(RobotControl_GetStatus().eventsStarted==1U);
        WaitPhase(ROBOT_WALK_MEASURING);
        CHECK(RobotControl_GetStatus().eventsCompleted==1U);
        CHECK(RobotControl_GetStatus().pendingShapes==0U);
        CHECK(MotionPlayer_GetMotion()==MotionGroups_FromQr(shape)); CheckStand();
    }
    puts("PASS six direct shapes: current-cycle exit -> original return -> shape, no extra walk/stand");
}
static void TestDirectShapeWaitsForCurrentTurn(void)
{
    uint32_t began;
    StartOrdinary(); WaitWindow(); Votes(CAMERA_WALK_RIGHT,3U,2U);
    WaitPhase(ROBOT_WALK_TO_STAND); WaitPhase(ROBOT_WALK_MOVING); began=s_tick;
    CHECK(MotionPlayer_GetMotion()==motion_turn_right);
    while(MotionPlayer_GetFrameIndex()<3U) { Tick(); }
    Shape(1U); Tick();
    CHECK(MotionPlayer_GetMotion()==motion_turn_right);
    WaitPhase(ROBOT_WALK_EVENT);
    CHECK(s_tick-began==RangeTime(motion_turn_right,0U,(uint8_t)(MotionGroups_Get(motion_turn_right)->frameCount-1U)));
    CHECK(MotionPlayer_GetMotion()==MotionGroups_FromQr(1U));
    puts("PASS direct shape finishes current turn support/recovery, then acts without extra walk");
}
static void TestDirectShapeStopPriority(void)
{
    StartOrdinary(); Shape(1U); SendLateCommand(CAMERA_WALK_HOLD,0U); Tick();
    WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().eventsStarted==0U);
    CHECK(RobotControl_GetStatus().pendingShapes==1U);
    Votes(CAMERA_WALK_FORWARD,2U,2U);
    CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_EVENT);
    CHECK(MotionPlayer_GetMotion()==MotionGroups_FromQr(1U));
    WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().eventsCompleted==1U);
    puts("PASS direct shape respects independent stop, resumes without adding a walk cycle");
}
#endif
static void TestFreshContinuousCorrectionsRemainAllowed(void)
{
    uint32_t seq;
    StartOrdinary(); WaitWindow(); Votes(CAMERA_WALK_RIGHT,255U,2U);
    WaitPhase(ROBOT_WALK_TO_STAND); WaitPhase(ROBOT_WALK_MOVING);
    CHECK(MotionPlayer_GetMotion()==motion_turn_right);
    seq=RobotControl_GetStatus().activeCommandSequence;
    WaitPhase(ROBOT_WALK_MEASURING);
    Votes(CAMERA_WALK_RIGHT,255U,1U);
    CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_MEASURING);
    Votes(CAMERA_WALK_RIGHT,255U,1U);
    CHECK(MotionPlayer_GetMotion()==motion_turn_right);
    CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_MOVING);
    CHECK(RobotControl_GetStatus().activeCommandSequence!=seq);
    CHECK(RobotControl_GetStatus().targetCycles==1U);
    puts("PASS fresh confirmed RIGHT after a completed turn remains allowed; old vote cannot authorize it");
}
int main(void)
{
    printf("[TEST_TIMING] sample=%lu first_cycle=%lu next_cycle=%lu ms\n",
        (unsigned long)SampleWindowTime(), (unsigned long)WalkBatchTime(1U),
        (unsigned long)(WalkBatchTime(2U)-WalkBatchTime(1U)));
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN && ROBOT_SHAPE_STEP_BEFORE_ACTION == 0U
    TestDirectShapesSafeCycleAndRecovery();
    TestDirectShapeWaitsForCurrentTurn();
    TestDirectShapeStopPriority();
#endif
    TestLowRateTwoFrameDecision();
    TestDirectionChangeBreaksConfirmation();
    TestLowRateReturnCancellation();
    TestLatestSampleInvalidatesStationaryDecision();
    TestConfirmedForwardCancelsUnexecutedCorrection();
    TestSingleForwardDoesNotCancelCorrection();
    TestStopPreemptsReturnCancellation(CAMERA_WALK_HOLD);
    TestStopPreemptsReturnCancellation(CAMERA_WALK_BLOCKED);
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    TestShapeDuringSampling();
    TestStopAndShapeCannotResumeFromInvalidVote();
    TestShapePreemptsReturnCancellation();
#endif
    TestLateWindowAndContinuation();
#if ROBOT_WALK_DECIDE_EACH_CYCLE
    TestCycleVotesCannotLeak();
#endif
    TestCachedCorrection(CAMERA_WALK_LEFT,motion_turn_left);
    TestCachedCorrection(CAMERA_WALK_RIGHT,motion_turn_right);
    TestCachedCorrection(CAMERA_WALK_SHIFT_LEFT,motion_left_translation);
    TestCachedCorrection(CAMERA_WALK_SHIFT_RIGHT,motion_right_translation);
    TestSplitOldAndLatestVote(); TestStops(CAMERA_WALK_HOLD); TestStops(CAMERA_WALK_BLOCKED);
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN && ROBOT_SHAPE_STEP_BEFORE_ACTION != 0U
    TestShapeAndStop();
#endif
    TestSubmissionBoundary();
    TestRightTwoStepThenOrdinaryBatch();
    TestFreshContinuousCorrectionsRemainAllowed();
    CHECK(s_boundaryLogs>0U && s_sampleLogs>0U);
    puts("ALL LATE VISION TESTS PASSED; HARDWARE NOT TESTED");
    return 0;
}
