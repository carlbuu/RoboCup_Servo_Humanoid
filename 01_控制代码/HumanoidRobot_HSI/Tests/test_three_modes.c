/* Original geometry_vision7 protocol integration. Camera Python stays untouched.
 * Real parser, scheduler, player and gait tables; only hardware APIs are simulated.
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "task.h"
#include "usart.h"
#include "camera.h"
#include "camera_walk_link.h"
#include "robot_control.h"
#include "robot_walk_config.h"
#include "motion_player.h"
#include "servo.h"
#include "servo_config.h"

#define CHECK(c) do { if (!(c)) { \
    fprintf(stderr, "FAIL line %d tick %lu: %s\n", __LINE__, \
            (unsigned long)s_tick, #c); exit(1); } } while (0)

UART_HandleTypeDef huart2;
static TickType_t s_tick;
static float s_targets[SERVO_COUNT + 1];
static uint32_t s_servoSends;
static uint32_t s_uartTransmits;
static uint8_t s_servoAccepted;
static uint8_t s_rejectJoint;
static const char * const s_shapeNames[7] =
{ "NONE", "CIRCLE", "STAR", "SQUARE", "DIAMOND", "CROSS", "TRIANGLE" };
static const char * const s_commandNames[5] =
{ "HOLD", "FWD", "TURNL", "TURNR", "BLOCKED" };

TickType_t xTaskGetTickCount(void) { return s_tick; }
void vTaskDelay(TickType_t ticks) { s_tick += ticks; }
void vTaskDelayUntil(TickType_t *last, TickType_t ticks)
{ *last += ticks; s_tick = *last; }
void Debug_Printf(char *format, ...) { (void)format; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart,
                                      const uint8_t *data, uint16_t length)
{
    (void)uart; (void)data; (void)length;
    s_uartTransmits++;
    CHECK(0); /* Original camera has no ACK receiver: MCU must not transmit. */
    return HAL_ERROR;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart)
{ (void)uart; CHECK(0); return HAL_ERROR; }
uint8_t Servo_SetTarget(uint8_t id, uint16_t speed,
                        uint8_t acceleration, float angle)
{
    (void)speed; (void)acceleration;
    CHECK(id > 0U && id <= SERVO_ACTIVE_COUNT);
    if (id == s_rejectJoint) { return 0U; }
    s_targets[id] = angle;
    return 1U;
}
uint8_t Servo_SendTargets(void) { s_servoSends++; return s_servoAccepted; }

static uint16_t CommandBytes(uint8_t *bytes, uint8_t command, uint8_t arg)
{
    CHECK(command <= CAMERA_WALK_BLOCKED);
#if CAMERA_PROTOCOL_BINARY
    bytes[0] = 0xAAU; bytes[1] = 0x57U; bytes[2] = command;
    bytes[3] = arg; bytes[4] = (uint8_t)(command ^ arg); bytes[5] = 0x0AU;
    (void)s_commandNames;
    return 6U;
#else
    return (uint16_t)sprintf((char *)bytes, "CMD,%s,%u\n", s_commandNames[command], (unsigned int)arg);
#endif
}
static uint16_t ShapeBytes(uint8_t *bytes, uint8_t shape)
{
    CHECK(shape >= 1U && shape <= 6U);
#if CAMERA_PROTOCOL_BINARY
    bytes[0] = 0xAAU; bytes[1] = 0x55U; bytes[2] = shape;
    bytes[3] = (uint8_t)(shape ^ 0xA5U); bytes[4] = 0x0AU;
    (void)s_shapeNames;
    return 5U;
#else
    return (uint16_t)sprintf((char *)bytes, "SHAPE,%s,%u\n", s_shapeNames[shape], (unsigned int)shape);
#endif
}
static void Command(uint8_t command, uint8_t arg)
{
    uint8_t bytes[64];
    uint16_t length = CommandBytes(bytes, command, arg);
    Camera_Process(bytes, length);
}
static void Shape(uint8_t shape)
{
    uint8_t bytes[64];
    uint16_t length = ShapeBytes(bytes, shape);
    Camera_Process(bytes, length);
}
static void ResetLink(void)
{
    s_tick = 0U;
    s_servoSends = s_uartTransmits = 0U;
    s_servoAccepted = 1U;
    s_rejectJoint = 0U;
    huart2.gState = HAL_UART_STATE_READY;
    Camera_Init();
}
static void ResetRobot(void) { ResetLink(); RobotControl_Init(); }
static void Tick(void)
{
    s_tick += 20U;
    RobotControl_Update(20U);
    CHECK(RobotControl_GetStatus().state != ROBOT_STATE_FAULT);
}
static void WaitPhase(RobotWalkPhase_t phase)
{
    uint32_t start = s_tick;
    while (RobotControl_GetStatus().walkPhase != phase)
    { CHECK(s_tick - start < 300000U); Tick(); }
}
static void BootWalk(void)
{
    ResetRobot();
    WaitPhase(ROBOT_WALK_MOVING);
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
    CHECK(s_tick == 2000U + ROBOT_WALK_STARTUP_SETTLE_MS);
    CHECK(s_uartTransmits == 0U); /* No HELLO/FWD was sent. */
}
static uint32_t RangeTime(MotionId_t motion, uint8_t first, uint8_t last)
{
    const MotionGroup_t *group = MotionGroups_Get(motion);
    uint32_t result = 0U;
    uint8_t index;
    CHECK(first <= last && last < group->frameCount);
    for (index = first; index <= last; index++)
    {
        uint32_t duration = group->frames[index].durationMs;
        if (duration == 0U) { duration = 1U; }
        result += ((duration + 19U) / 20U) * 20U;
    }
    return result;
}
static uint32_t WalkBatchTime(uint16_t cycles)
{
    const MotionGroup_t *group = MotionGroups_Get(motion_walk_forward);
    return RangeTime(motion_walk_forward, 0U, ROBOT_WALK_CYCLE_END_FRAME) +
        (uint32_t)(cycles - 1U) * RangeTime(motion_walk_forward,
           ROBOT_WALK_LOOP_START_FRAME, (uint8_t)(group->frameCount - 1U));
}
static void CheckStand(void)
{
    uint8_t joint;
    const MotionGroup_t *stand = MotionGroups_Get(motion_stand);
    for (joint = 0U; joint < ROBOT_JOINT_COUNT; joint++)
    {
        if (ROBOT_SERVO_ID(joint) <= SERVO_ACTIVE_COUNT)
        {
            CHECK(fabsf(s_targets[ROBOT_SERVO_ID(joint)] -
                        stand->frames[0].angleDeg10[joint] * 0.1f) < 0.001f);
        }
    }
}

static void TestParserSplitMailboxAndPriority(void)
{
    uint8_t shape[64];
    uint8_t command[64];
    uint8_t joined[128];
    uint16_t a = CommandBytes(command, CAMERA_WALK_LEFT, 3U);
    uint16_t b = ShapeBytes(shape, 1U);
    uint16_t split;
    CameraWalkRequest_t request;
    memcpy(joined, command, a);
    memcpy(joined + a, shape, b);
    for (split = 0U; split <= a + b; split++)
    {
        ResetLink();
        Camera_Process(joined, split);
        Camera_Process(joined + split, (uint16_t)(a + b - split));
        CHECK(CameraWalk_TakeRequest(&request) && request.type == CAMERA_WALK_SHAPE && request.shape == 1U);
        CHECK(CameraWalk_TakeRequest(&request) && request.type == CAMERA_WALK_COMMAND && request.command == CAMERA_WALK_LEFT && request.arg == 3U);
        CHECK(!CameraWalk_TakeRequest(&request));
    }
    ResetLink();
    Command(CAMERA_WALK_LEFT, 3U);
    Command(CAMERA_WALK_FORWARD, 2U);
    Command(CAMERA_WALK_RIGHT, 6U);
    Shape(2U);
    CHECK(CameraWalk_TakeRequest(&request) && request.type == CAMERA_WALK_SHAPE && request.shape == 2U);
    CHECK(CameraWalk_TakeRequest(&request) && request.command == CAMERA_WALK_RIGHT && request.arg == 6U);
    CHECK(!CameraWalk_TakeRequest(&request));
    Shape(3U);
    Command(CAMERA_WALK_FORWARD, 2U);
    CameraWalk_ClearMotion();
    CHECK(CameraWalk_TakeRequest(&request) && request.type == CAMERA_WALK_SHAPE && request.shape == 3U);
    CHECK(!CameraWalk_TakeRequest(&request));
    Command(CAMERA_WALK_BLOCKED, 0U);
    Command(CAMERA_WALK_FORWARD, 2U);
    Command(CAMERA_WALK_RIGHT, 3U);
    CHECK(CameraWalk_TakeRequest(&request) && request.command == CAMERA_WALK_BLOCKED);
    CHECK(!CameraWalk_TakeRequest(&request)); /* BLOCKED survives later same-block advice. */
    Command(CAMERA_WALK_HOLD, 0U);
    CameraWalk_ClearMotion();
    CHECK(CameraWalk_TakeRequest(&request) && request.command == CAMERA_WALK_HOLD);
    Command(CAMERA_WALK_BLOCKED, 0U);
    CameraWalk_ClearMotion();
    CHECK(CameraWalk_TakeRequest(&request) && request.command == CAMERA_WALK_BLOCKED);
    CHECK(!CameraWalk_TakeRequest(&request)); /* High-priority RX pauses survive cleanup. */
    CHECK(s_uartTransmits == 0U);
    puts("PASS every split, sticky frames, event priority, latest-CMD mailbox and RX-only service");
}

static void TestParserInvalidNoiseAndModeIsolation(void)
{
    CameraWalkRequest_t request;
    uint8_t frame[64];
    uint16_t length;
    unsigned int shape;
    ResetLink();
#if CAMERA_PROTOCOL_BINARY
    {
        uint8_t noise[] = { 0x11U, 0xAAU, 0xAAU, 0x55U, 6U, 0xA3U, 0x0AU };
        uint8_t bad[] = { 0xAAU, 0x57U, 3U, 3U, 0xFFU, 0x0AU };
        Camera_Process(noise, sizeof(noise));
        CHECK(CameraWalk_TakeRequest(&request) && request.shape == 6U);
        Camera_Process(bad, sizeof(bad));
        bad[4] = 0U; bad[5] = 0x11U;
        Camera_Process(bad, sizeof(bad));
        CHECK(!CameraWalk_TakeRequest(&request));
        Camera_Process((uint8_t *)"CMD,FWD,2\nSHAPE,CIRCLE,1\n",
                       (uint16_t)strlen("CMD,FWD,2\nSHAPE,CIRCLE,1\n"));
        CHECK(!CameraWalk_TakeRequest(&request));
        bad[2] = 5U; bad[3] = 0U; bad[4] = 5U; bad[5] = 0x0AU;
        Camera_Process(bad, sizeof(bad));
        CHECK(!CameraWalk_TakeRequest(&request));
        {
            uint8_t overlap[] = { 0xAAU,0x57U,0xAAU,0x55U,1U,0xA4U,0x0AU };
            Camera_Process(overlap, sizeof(overlap));
            CHECK(CameraWalk_TakeRequest(&request) && request.shape == 1U);
            CHECK(!CameraWalk_TakeRequest(&request));
        }
    }
#else
    {
        static const char * const bad[] = {
            "CMD,FWD,0\n", "CMD,FWD,256\n", "CMD,HOLD,1\n", "CMD,BLOCKED,1\n",
            "CMD,TURNL,-3\n", "CMD,NOPE,3\n", "CMD,FWD,2,EXTRA\n",
            "SHAPE,CIRCLE,2\n", "SHAPE,NOPE,1\n", "SHAPE,CROSS,0\n",
            "SHAPE,TRIANGLE,7\n", "CMD,FWD,2x\n"
        };
        uint8_t binary[] = { 0xAAU,0x55U,1U,0xA4U,0x0AU,0xAAU,0x57U,1U,2U,3U,0x0AU };
        unsigned int index;
        for (index = 0U; index < sizeof(bad) / sizeof(bad[0]); index++)
        { Camera_Process((uint8_t *)bad[index], (uint16_t)strlen(bad[index])); }
        Camera_Process(binary, sizeof(binary));
        Camera_Process((uint8_t *)"$GV,CMD,1,FWD,2*00\r\n",
                       (uint16_t)strlen("$GV,CMD,1,FWD,2*00\r\n"));
        CHECK(!CameraWalk_TakeRequest(&request));
        Camera_Process((uint8_t *)"noise\nCMD,FWD,2\r\n", 17U);
        CHECK(CameraWalk_TakeRequest(&request) && request.command == CAMERA_WALK_FORWARD);
        length = ShapeBytes(frame, 5U);
        frame[length - 1U] = '\r'; frame[length++] = '\n';
        Camera_Process(frame, length);
        CHECK(CameraWalk_TakeRequest(&request) && request.shape == 5U);
        {
            char overflow[CAMERA_FRAME_MAX + 16U];
            memset(overflow, 'x', sizeof(overflow));
            Camera_Process((uint8_t *)overflow, sizeof(overflow));
            /* A corrupted line may contain CMD text; accept only a new whole line. */
            Camera_Process((uint8_t *)"CMD,FWD,2\n", 10U);
            CHECK(!CameraWalk_TakeRequest(&request));
            CHECK(Camera_GetStatistics().overflowErrors >= 1U);
            Command(CAMERA_WALK_FORWARD, 2U);
            CHECK(CameraWalk_TakeRequest(&request) && request.command == CAMERA_WALK_FORWARD);
        }
    }
#endif
    for (shape = 1U; shape <= 6U; shape++)
    {
        Shape((uint8_t)shape);
        CHECK(CameraWalk_TakeRequest(&request) && request.shape == shape);
        CHECK(Camera_GetShape() == (CameraShape_t)shape);
    }
    length = CommandBytes(frame, CAMERA_WALK_BLOCKED, 0U);
    Camera_Process(frame, length);
    CHECK(CameraWalk_TakeRequest(&request) && request.command == CAMERA_WALK_BLOCKED);
    CHECK(!CameraWalk_TakeRequest(&request));
    ResetLink();
    for (shape = 0U; shape < 17U; shape++) { Shape(1U); }
    CHECK(Camera_GetStatistics().requestQueueFull == 1U);
    for (shape = 0U; shape < 16U; shape++) { CHECK(CameraWalk_TakeRequest(&request)); }
    CHECK(!CameraWalk_TakeRequest(&request));
    puts("PASS protocol mode isolation, malformed/range/name-ID rejection, noise and bounded shape RX");
}

static void TestAutonomousBatchAndMeasurementTimeout(void)
{
    uint32_t began;
    uint16_t cycles = ROBOT_STM_WALK_BATCH_STEPS / ROBOT_WALK_STEPS_PER_CYCLE;
    BootWalk();
    began = s_tick;
    CHECK(RobotControl_GetStatus().targetCycles == cycles);
    while (RobotControl_GetStatus().walkPhase == ROBOT_WALK_MOVING)
    {
        Command(CAMERA_WALK_FORWARD, 2U); /* Original camera repeats this independently. */
        Tick();
        CHECK(s_tick - began <= WalkBatchTime(cycles));
    }
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_TO_STAND);
    CHECK(s_tick - began == WalkBatchTime(cycles));
    WaitPhase(ROBOT_WALK_MEASURING);
    CheckStand();
    began = s_tick;
    while (s_tick - began < ROBOT_STM_CORRECTION_WAIT_MS - 20U) { Tick(); }
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING);
    Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MOVING);
    CHECK(s_tick - began == ROBOT_STM_CORRECTION_WAIT_MS);
    CHECK(s_uartTransmits == 0U);
    puts("PASS autonomous startup, configured complete batch, repeated-FWD immunity and timeout continuation");
}

static void WaitStartupEventThenMeasure(void)
{
    uint32_t began = s_tick;
    uint8_t sawEvent = 0U;
    while (RobotControl_GetStatus().walkPhase != ROBOT_WALK_MEASURING)
    {
        Tick();
        CHECK(s_tick - began < 300000U);
        CHECK(MotionPlayer_GetMotion() != motion_walk_forward);
        if (RobotControl_GetStatus().walkPhase == ROBOT_WALK_EVENT) { sawEvent = 1U; }
    }
    CHECK(sawEvent && RobotControl_GetStatus().eventsStarted == 1U &&
          RobotControl_GetStatus().eventsCompleted == 1U);
    CheckStand();
}

static void TestStartupShapeAndPauseBarrier(void)
{
    uint32_t began;
    uint32_t sends;
    ResetRobot();
    Shape(1U);
    WaitStartupEventThenMeasure();
    sends = s_servoSends;
    Tick(); Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING && s_servoSends == sends);
    Command(CAMERA_WALK_FORWARD, 2U);
    Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MOVING);

    ResetRobot();
    Shape(1U);
    Command(CAMERA_WALK_HOLD, 0U);
    WaitStartupEventThenMeasure();
    sends = s_servoSends;
    began = s_tick;
    while (s_tick - began < ROBOT_STM_CORRECTION_WAIT_MS + 1000U) { Tick(); }
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING && s_servoSends == sends);
    Command(CAMERA_WALK_FORWARD, 2U);
    Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MOVING);
    puts("PASS startup-cached shape returns to measurement and startup HOLD prevents autonomous continuation");
}

static void TestHoldSafeStopAndFreshForward(void)
{
    uint32_t began;
    uint32_t sends;
    BootWalk();
    began = s_tick;
    Command(CAMERA_WALK_HOLD, 0U);
    Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MOVING);
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
    WaitPhase(ROBOT_WALK_TO_STAND);
    CHECK(s_tick - began == WalkBatchTime(1U));
    WaitPhase(ROBOT_WALK_MEASURING);
    CheckStand();
    sends = s_servoSends;
    began = s_tick;
    while (s_tick - began < ROBOT_STM_CORRECTION_WAIT_MS + 1000U) { Tick(); }
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING);
    CHECK(s_servoSends == sends);
    Command(CAMERA_WALK_FORWARD, 2U);
    Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MOVING);
    puts("PASS HOLD completes current gait cycle, remains paused and fresh FWD resumes");
}

static void TestOnlyFreshTurnAndNoBusyTurnQueue(void)
{
    uint32_t began;
    uint32_t sends;
    BootWalk();
    began = s_tick;
    while (RobotControl_GetStatus().walkPhase != ROBOT_WALK_MEASURING)
    {
        Command(CAMERA_WALK_LEFT, 3U);
        Tick();
        CHECK(MotionPlayer_GetMotion() != motion_turn_left && MotionPlayer_GetMotion() != motion_turn_right);
        CHECK(s_tick - began < 300000U);
    }
    sends = s_servoSends;
    Tick(); Tick(); Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING && s_servoSends == sends);
    Command(CAMERA_WALK_RIGHT, 3U);
    Tick();
    CHECK(MotionPlayer_GetMotion() == motion_turn_right);
    CHECK(RobotControl_GetStatus().targetCycles == 1U);
    began = s_tick;
    while (RobotControl_GetStatus().walkPhase != ROBOT_WALK_MEASURING)
    {
        Command(CAMERA_WALK_LEFT, 3U); /* Many stale suggestions while physical turning. */
        Tick();
        CHECK(MotionPlayer_GetMotion() != motion_turn_left);
        CHECK(s_tick - began < 300000U);
    }
    CheckStand();
    sends = s_servoSends;
    Tick(); Tick(); Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING);
    CHECK(s_servoSends == sends);
    Command(CAMERA_WALK_FORWARD, 2U);
    Tick();
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
    puts("PASS stale walking/turning advice discarded, fresh stable TURN executes once and FWD resumes");
}

static void TestTurnArgumentIgnoredAndLineBeforeShape(void)
{
    static const uint8_t arguments[] = { 0U, 7U, 30U, 255U };
    uint8_t index;
    uint32_t began;
    uint32_t sends;
    for (index = 0U; index < sizeof(arguments) / sizeof(arguments[0]); ++index)
    {
        ResetRobot();
        WaitPhase(ROBOT_WALK_MEASURING);
        Tick(); /* New commands must be newer than the stand-ready tick. */
        Command((index % 2U) == 0U ? CAMERA_WALK_LEFT : CAMERA_WALK_RIGHT,
                arguments[index]);
        Tick();
        CHECK(MotionPlayer_GetMotion() == ((index % 2U) == 0U
              ? motion_turn_left : motion_turn_right));
        CHECK(RobotControl_GetStatus().targetCycles == 1U);
        Shape(1U);
        began = s_tick;
        while (RobotControl_GetStatus().walkPhase != ROBOT_WALK_MEASURING)
        {
            Command(CAMERA_WALK_FORWARD, 2U); /* Busy FWD cannot prove alignment. */
            Tick();
            CHECK(RobotControl_GetStatus().eventsStarted == 0U);
            CHECK(s_tick - began < 20000U);
        }
        CHECK(RobotControl_GetStatus().pendingShapes == 1U);
        sends = s_servoSends;
        began = s_tick;
        while (s_tick - began <= ROBOT_STM_CORRECTION_WAIT_MS + 1000U) { Tick(); }
        CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING);
        CHECK(RobotControl_GetStatus().eventsStarted == 0U && s_servoSends == sends);
        Command(CAMERA_WALK_HOLD, 0U); Tick();
        CHECK(RobotControl_GetStatus().eventsStarted == 0U);
        Command(CAMERA_WALK_FORWARD, 2U); Tick();
        CHECK(RobotControl_GetStatus().eventsStarted == 1U);
        CHECK(MotionPlayer_GetMotion() == motion_qr_left_arm);
        WaitPhase(ROBOT_WALK_MEASURING);
        CHECK(RobotControl_GetStatus().eventsCompleted == 1U);
    }
    CHECK(s_uartTransmits == 0U);
    puts("PASS TURN args 0/7/30/255 execute once; busy FWD, HOLD and timeout cannot bypass fresh line before shape");
}

static void TestWalkingShapesMergeQueueAndRepeatAfterCompletion(void)
{
    uint32_t began;
    uint32_t toStandBegan;
    uint8_t index;
    RobotControlStatus_t status;
    BootWalk();
    began = s_tick;
    Shape(1U); Shape(1U);
    Tick();
    status = RobotControl_GetStatus();
    CHECK(status.walkPhase == ROBOT_WALK_MOVING && status.eventsStarted == 0U && status.pendingShapes == 1U);
    for (index = 2U; index <= 6U; index++) { Shape(index); }
    Tick();
    CHECK(RobotControl_GetStatus().pendingShapes == ROBOT_WALK_SHAPE_QUEUE_LENGTH);
    CHECK(RobotControl_GetStatus().eventsDropped == 2U);
    WaitPhase(ROBOT_WALK_TO_STAND);
    CHECK(s_tick - began == WalkBatchTime(1U));
    toStandBegan = s_tick;
    WaitPhase(ROBOT_WALK_EVENT);
    CHECK(s_tick - toStandBegan == RangeTime(motion_walk_tostand, 0U,
           (uint8_t)(MotionGroups_Get(motion_walk_tostand)->frameCount - 1U)) + ROBOT_WALK_SETTLE_MS);
    CHECK(RobotControl_GetStatus().eventsStarted == 1U);
    Shape(1U); /* Active-event same ID must merge as well. */
    Tick();
    CHECK(RobotControl_GetStatus().pendingShapes == 3U);
    began = s_tick;
    while (RobotControl_GetStatus().eventsCompleted < 4U)
    { CHECK(s_tick - began < 300000U); Tick(); }
    WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().eventsStarted == 4U && RobotControl_GetStatus().pendingShapes == 0U);
    Shape(1U); /* Finished event is not permanently deduplicated by shape ID. */
    Tick();
    CHECK(RobotControl_GetStatus().eventsStarted == 5U);
    began = s_tick;
    while (RobotControl_GetStatus().eventsCompleted < 5U)
    { CHECK(s_tick - began < 300000U); Tick(); }
    CHECK(s_uartTransmits == 0U);
    puts("PASS walking shape cache, current-cycle stop/stand/settle, ID merge, full queue and same-ID later event");
}

static void TestBlockedLatch(void)
{
    uint32_t began;
    uint32_t sends;
    BootWalk();
    began = s_tick;
    Command(CAMERA_WALK_BLOCKED, 0U);
    Command(CAMERA_WALK_RIGHT, 3U);
    Command(CAMERA_WALK_FORWARD, 2U);
    Tick();
    WaitPhase(ROBOT_WALK_TO_STAND);
    CHECK(s_tick - began == WalkBatchTime(1U));
    WaitPhase(ROBOT_WALK_BLOCKED);
    CheckStand();
    sends = s_servoSends;
    began = s_tick;
    while (s_tick - began < ROBOT_STM_CORRECTION_WAIT_MS + 2000U)
    {
        Command((s_tick % 40U == 0U) ? CAMERA_WALK_HOLD : CAMERA_WALK_BLOCKED, 0U);
        Tick();
    }
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_BLOCKED);
    CHECK(s_servoSends == sends);
    CHECK(s_uartTransmits == 0U);
    puts("PASS BLOCKED reaches safe stand; HOLD/BLOCKED and timeout do not cause blind forward motion");
}

static void TestBlockedFreshDirectionRecovery(void)
{
    uint32_t began;
    BootWalk();
    Command(CAMERA_WALK_BLOCKED, 0U); Tick();
    Shape(2U);
    began = s_tick;
    while (RobotControl_GetStatus().walkPhase != ROBOT_WALK_BLOCKED)
    {
        Command(CAMERA_WALK_RIGHT, 255U); Tick();
        CHECK(MotionPlayer_GetMotion() != motion_turn_right);
        CHECK(RobotControl_GetStatus().eventsStarted == 0U);
        CHECK(s_tick - began < 300000U);
    }
    Tick();
    Command(CAMERA_WALK_RIGHT, 255U); Tick();
    CHECK(MotionPlayer_GetMotion() == motion_turn_right);
    CHECK(RobotControl_GetStatus().targetCycles == 1U);
    CHECK(RobotControl_GetStatus().eventsStarted == 0U);
    WaitPhase(ROBOT_WALK_MEASURING); Tick();
    Command(CAMERA_WALK_FORWARD, 2U); Tick();
    CHECK(MotionPlayer_GetMotion() == MotionGroups_FromQr(2U));
    CHECK(RobotControl_GetStatus().eventsStarted == 1U);
    WaitPhase(ROBOT_WALK_MEASURING); Tick();
    Command(CAMERA_WALK_BLOCKED, 0U); Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_BLOCKED);
    Tick();
    Command(CAMERA_WALK_FORWARD, 2U); Tick();
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
    CHECK(s_uartTransmits == 0U);
    puts("PASS blocked busy-direction rejection, fresh single-turn recovery, queued star after alignment and fresh FWD recovery");
}

/* A completed walking batch must meet the calibrated recovery entry pose,
 * rather than lift the foot again and reverse back across the stride. */
static void TestWalkRecoveryContinuity(void)
{
    const MotionGroup_t *recovery = MotionGroups_Get(motion_walk_tostand);
    uint16_t cycles;
    uint32_t ticks;
    uint8_t joint;
    uint8_t previous;
    for (cycles = 1U; cycles <= 2U; ++cycles)
    {
        ResetLink();
        MotionPlayer_Init();
        CHECK(MotionPlayer_StartCycles(motion_walk_forward, cycles,
            ROBOT_WALK_LOOP_START_FRAME, ROBOT_WALK_CYCLE_END_FRAME,
            ROBOT_WALK_STOP_FRAME));
        ticks = 0U;
        while (MotionPlayer_IsActive())
        {
            CHECK(++ticks < 10000U);
            CHECK(MotionPlayer_Update(20U));
        }
        CHECK(MotionPlayer_GetCompletedCycles() == cycles);
        for (joint = 0U; joint < ROBOT_JOINT_COUNT; ++joint)
        {
            if (ROBOT_SERVO_ID(joint) <= SERVO_ACTIVE_COUNT)
            {
                CHECK(fabsf(s_targets[ROBOT_SERVO_ID(joint)] -
                    recovery->frames[0].angleDeg10[joint] * 0.1f) <= 5.0f);
            }
        }
    }
    /* After the final lift, continuous debug walking must begin with forward
     * swing, without prematurely bending the knee or retracting ID7. */
    ResetLink();
    MotionPlayer_Init();
    MotionPlayer_Start(motion_walk_forward);
    ticks = 0U;
    do
    {
        previous = MotionPlayer_GetFrameIndex();
        CHECK(++ticks < 10000U);
        CHECK(MotionPlayer_Update(20U));
    } while (MotionPlayer_GetFrameIndex() >= previous);
    CHECK(MotionPlayer_Update(20U));
    CHECK(fabsf(s_targets[3] - (-20.0f)) < 0.001f);
    CHECK(fabsf(s_targets[7]) < 0.001f);
    CHECK(s_targets[4] > 29.0f);
    puts("PASS walk recovery continuity for one/two cycles and debug forward-swing entry");
}
/* A neutral-ended turn must go directly to the existing settling barrier,
 * without replaying frame zero or a second standing motion. */
static void TestTurnSettlingLatency(void)
{
    uint8_t direction;
    uint32_t began;
    uint32_t expected;
    uint32_t sends;
    MotionId_t motion;
    const MotionGroup_t *group;
    if (ROBOT_RUN_MODE != ROBOT_MODE_FULL_RUN &&
        ROBOT_RUN_MODE != ROBOT_MODE_LINE_DEBUG) { return; }
    for (direction = 0U; direction < 2U; ++direction)
    {
        ResetRobot();
        WaitPhase(ROBOT_WALK_MEASURING);
        Tick();
        motion = direction == 0U ? motion_turn_left : motion_turn_right;
        group = MotionGroups_Get(motion);
        Command(direction == 0U ? CAMERA_WALK_LEFT : CAMERA_WALK_RIGHT, 1U);
        Tick();
        CHECK(MotionPlayer_GetMotion() == motion);
        began = s_tick;
        expected = RangeTime(motion, 0U, (uint8_t)(group->frameCount - 1U));
        while (s_tick - began < expected)
        {
            Command(CAMERA_WALK_FORWARD, 2U);
            Tick();
            CHECK(MotionPlayer_GetMotion() == motion);
        }
        CHECK(s_tick - began == expected);
        CHECK(!MotionPlayer_IsActive());
        CHECK(MotionPlayer_GetFrameIndex() == group->frameCount - 1U);
        CHECK(MotionPlayer_GetCompletedCycles() == 1U);
        CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_SETTLING);
        CheckStand();
        sends = s_servoSends;
        while (RobotControl_GetStatus().walkPhase != ROBOT_WALK_MEASURING)
        {
            Command(CAMERA_WALK_FORWARD, 2U);
            Tick();
            CHECK(s_tick - began <= expected + ROBOT_WALK_SETTLE_MS);
        }
        CHECK(s_tick - began == expected + ROBOT_WALK_SETTLE_MS);
        CHECK(s_servoSends == sends);
        Tick();
        CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING);
        Command(CAMERA_WALK_FORWARD, 2U);
        Tick();
        CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
    }
    puts("PASS both turns finish at the last frame, settle once and require fresh vision");
}

static void TestServoSubmissionFailure(void)
{
    ResetLink();
    MotionPlayer_Init();
    CHECK(MotionPlayer_StartCycles(motion_stand, 1U, 0U, 0U, 0U));
    s_servoAccepted = 0U;
    CHECK(!MotionPlayer_Update(65535U));
    CHECK(MotionPlayer_IsActive() && MotionPlayer_GetCompletedCycles() == 0U);
    s_servoAccepted = 1U;
    s_rejectJoint = 1U;
    CHECK(!MotionPlayer_Update(20U) && MotionPlayer_IsActive());
    s_rejectJoint = 0U;
    CHECK(MotionPlayer_Update(20U) && !MotionPlayer_IsActive());
    puts("PASS unsuccessful final servo target/queue submission cannot finish a frame early");
}


/* Exercise the real parser/player with every shape, repeated commands and duplicates. */
static void TestShapeOnlyCommunication(void)
{
    uint8_t shape;
    uint32_t began;
    uint32_t sends;
    static const MotionId_t expected[] = {
        motion_qr_left_arm, motion_qr_right_arm, motion_qr_left_leg,
        motion_qr_right_leg, motion_qr_both_arms, motion_qr_head_shake
    };
    ResetRobot();
    began = s_tick;
    while (RobotControl_GetStatus().walkPhase != ROBOT_WALK_MEASURING)
    {
        Command(CAMERA_WALK_FORWARD, 2U);
        Tick();
        CHECK(s_tick - began < 10000U);
        CHECK(MotionPlayer_GetMotion() == motion_stand);
    }
    CheckStand();
    sends = s_servoSends;
    began = s_tick;
    while (s_tick - began < ROBOT_STM_CORRECTION_WAIT_MS + 2000U)
    {
        uint8_t cmd = (uint8_t)((s_tick / 20U) % 5U);
        Command(cmd, (cmd == CAMERA_WALK_HOLD || cmd == CAMERA_WALK_BLOCKED) ? 0U : 3U);
        Tick();
        CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING);
        CHECK(s_servoSends == sends);
    }
    for (shape = 1U; shape <= 6U; shape++)
    {
        Shape(shape); Shape(shape);
        Tick();
        CHECK(RobotControl_GetStatus().eventsStarted == shape);
        CHECK(MotionPlayer_GetMotion() == expected[shape - 1U]);
        CHECK(RobotControl_GetStatus().pendingShapes == 0U);
        Shape(shape); /* Same active event must not restart or enqueue itself. */
        began = s_tick;
        while (RobotControl_GetStatus().eventsCompleted < shape)
        {
            uint8_t cmd = (uint8_t)((s_tick / 20U) % 5U);
            Command(cmd, (cmd == CAMERA_WALK_HOLD || cmd == CAMERA_WALK_BLOCKED) ? 0U : 3U);
            Tick();
            CHECK(s_tick - began < 30000U);
            CHECK(RobotControl_GetStatus().eventsStarted == shape);
            CHECK(MotionPlayer_GetMotion() == expected[shape - 1U] ||
                  MotionPlayer_GetMotion() == motion_stand);
        }
        CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING);
        CheckStand();
    }
    Shape(6U); Tick(); /* A new event after completion may repeat the same shape. */
    CHECK(RobotControl_GetStatus().eventsStarted == 7U);
    WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().eventsCompleted == 7U);
    sends = s_servoSends;
    began = s_tick;
    while (s_tick - began < ROBOT_STM_CORRECTION_WAIT_MS + 2000U) { Tick(); }
    CHECK(s_servoSends == sends && s_uartTransmits == 0U);
    CheckStand();
    ResetRobot();
    Shape(1U); Command(CAMERA_WALK_FORWARD, 2U);
    WaitStartupEventThenMeasure();
    CHECK(s_uartTransmits == 0U);
    puts("PASS shape-only: no automatic/CMD/timeout walking, six mappings, duplicate merge, recovery and repeat");
}


static void TestActionDebugIsolation(void)
{
    const MotionGroup_t *group = MotionGroups_Get(ROBOT_DEBUG_MOTION);
    uint32_t began;
    uint32_t duration;
    uint32_t sends;
    ResetRobot();
    while (s_tick + 20U < ROBOT_ACTION_START_DELAY_MS)
    {
        Shape(1U); Command(CAMERA_WALK_FORWARD, 2U); Tick();
        CHECK(MotionPlayer_GetMotion() == motion_stand);
    }
    Tick();
    CHECK(MotionPlayer_GetMotion() == ROBOT_DEBUG_MOTION);
    CHECK(RobotControl_GetStatus().eventsStarted == 0U);
    duration = RangeTime(ROBOT_DEBUG_MOTION, 0U, (uint8_t)(group->frameCount - 1U));
    began = s_tick;
    while (s_tick - began < duration + 2000U)
    {
        Shape(2U); Command(CAMERA_WALK_RIGHT, 3U); Tick();
        CHECK(MotionPlayer_GetMotion() == ROBOT_DEBUG_MOTION);
        CHECK(RobotControl_GetStatus().eventsStarted == 0U);
    }
    if (group->loop == 0U)
    {
        CHECK(RobotControl_GetStatus().state == ROBOT_STATE_FINISHED);
        sends = s_servoSends;
        for (began = 0U; began < 100U; began++) { Tick(); }
        CHECK(s_servoSends == sends);
    }
    else { CHECK(MotionPlayer_IsActive()); }
    CHECK(s_uartTransmits == 0U);
    puts("PASS action mode ignores vision, selects configured action, preserves loop and single completion");
}


static void TestLineDebugBatchGate(void)
{
    uint32_t began, sends;
    uint16_t cycles = ROBOT_STM_WALK_BATCH_STEPS / ROBOT_WALK_STEPS_PER_CYCLE;
    uint8_t cmd;
    ResetRobot();
    CHECK(strcmp(RobotControl_GetModeName(), "line_debug") == 0);
    while (RobotControl_GetStatus().walkPhase != ROBOT_WALK_MOVING)
    {
        Command(CAMERA_WALK_BLOCKED, 0U); Shape(1U); Tick();
        CHECK(s_tick <= 2000U + ROBOT_WALK_STARTUP_SETTLE_MS);
    }
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
    CHECK(RobotControl_GetStatus().targetCycles == cycles);
    began = s_tick;
    while (RobotControl_GetStatus().walkPhase == ROBOT_WALK_MOVING)
    {
        cmd = (uint8_t)((s_tick / 20U) % 5U);
        Command(cmd, (cmd == CAMERA_WALK_HOLD || cmd == CAMERA_WALK_BLOCKED) ? 0U : 3U);
        Shape(2U); Tick();
        CHECK(s_tick - began <= WalkBatchTime(cycles));
        CHECK(RobotControl_GetStatus().eventsStarted == 0U);
    }
    CHECK(s_tick - began == WalkBatchTime(cycles));
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_TO_STAND);
    while (RobotControl_GetStatus().walkPhase != ROBOT_WALK_MEASURING)
    { Command(CAMERA_WALK_RIGHT, 3U); Tick(); }
    CheckStand();
    CHECK(RobotControl_GetStatus().pendingShapes == 0U);
    /* A request received at the settling boundary is not a fresh decision. */
    Command(CAMERA_WALK_FORWARD, 2U); Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING);
    sends = s_servoSends; began = s_tick;
    while (s_tick - began <= ROBOT_STM_CORRECTION_WAIT_MS + 1000U) { Tick(); }
    CHECK(s_servoSends == sends);
    Command(CAMERA_WALK_LEFT, 3U); Tick();
    CHECK(MotionPlayer_GetMotion() == motion_turn_left);
    began = s_tick;
    while (RobotControl_GetStatus().walkPhase != ROBOT_WALK_MEASURING)
    {
        Command(CAMERA_WALK_RIGHT, 3U); Shape(3U); Tick();
        CHECK(MotionPlayer_GetMotion() != motion_turn_right);
        CHECK(s_tick - began < 300000U);
    }
    CheckStand(); Tick();
    Command(CAMERA_WALK_HOLD, 0U); Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING);
    Command(CAMERA_WALK_RIGHT, 3U); Tick();
    CHECK(MotionPlayer_GetMotion() == motion_turn_right);
    WaitPhase(ROBOT_WALK_MEASURING); Tick();
    Command(CAMERA_WALK_FORWARD, 2U); Tick();
    CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
    CHECK(RobotControl_GetStatus().targetCycles == cycles);
    WaitPhase(ROBOT_WALK_MEASURING); Tick();
    Command(CAMERA_WALK_BLOCKED, 0U); Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_BLOCKED);
    sends = s_servoSends;
    Command(CAMERA_WALK_FORWARD, 2U); Shape(4U); Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_BLOCKED);
    CHECK(s_servoSends == sends && RobotControl_GetStatus().eventsStarted == 0U);
    puts("PASS line debug: complete batch, busy CMD/shape isolation, fresh-only gate, no timeout walk, turns, HOLD and BLOCKED");
}

int main(void)
{
    TestParserSplitMailboxAndPriority();
    TestParserInvalidNoiseAndModeIsolation();
    if (ROBOT_RUN_MODE == ROBOT_MODE_ACTION_DEBUG)
    {
        TestActionDebugIsolation();
    }
    else if (ROBOT_RUN_MODE == ROBOT_MODE_VISION_DEBUG)
    {
        TestShapeOnlyCommunication();
    }
    else if (ROBOT_RUN_MODE == ROBOT_MODE_LINE_DEBUG)
    {
        TestLineDebugBatchGate();
    }
    else
    {
        TestAutonomousBatchAndMeasurementTimeout();
        TestStartupShapeAndPauseBarrier();
        TestHoldSafeStopAndFreshForward();
        TestOnlyFreshTurnAndNoBusyTurnQueue();
        TestTurnArgumentIgnoredAndLineBeforeShape();
        TestWalkingShapesMergeQueueAndRepeatAfterCompletion();
        TestBlockedLatch();
        TestBlockedFreshDirectionRecovery();
    }
    TestWalkRecoveryContinuity();
    TestTurnSettlingLatency();
    TestServoSubmissionFailure();
    puts("ALL MODE TESTS PASSED");
    return 0;
}
