/* Diagnostic timing audit. This does not replace the 5-degree safety regression.
 * Hardware is stubbed; source durations and scheduler transitions are observed.
 */
#define main original_three_modes_main
#include "test_three_modes.c"
#undef main

static uint32_t NominalTime(MotionId_t id, uint8_t first, uint8_t last)
{
    const MotionGroup_t *g = MotionGroups_Get(id);
    uint32_t total = 0U;
    uint8_t f;
    for (f = first; f <= last; ++f) { total += g->frames[f].durationMs; }
    return total;
}
static void AuditGroups(void)
{
    MotionId_t id;
    for (id = motion_stand; id < motion_group_count; ++id)
    {
        const MotionGroup_t *g = MotionGroups_Get(id);
        printf("GROUP %s frames=%u nominal_ms=%lu scheduled_ms=%lu\n", g->name,
            (unsigned int)g->frameCount,
            (unsigned long)NominalTime(id, 0U, (uint8_t)(g->frameCount - 1U)),
            (unsigned long)RangeTime(id, 0U, (uint8_t)(g->frameCount - 1U)));
    }
    printf("WALK finite_cycle nominal_ms=%lu scheduled_ms=%lu two_cycles_ms=%lu\n",
        (unsigned long)NominalTime(motion_walk_forward, 0U, ROBOT_WALK_STOP_FRAME),
        (unsigned long)WalkBatchTime(1U), (unsigned long)WalkBatchTime(2U));
}
static void ReadyForAudit(void)
{
    ResetRobot(); WaitPhase(ROBOT_WALK_MEASURING); Tick();
}
static void AuditExtraAndResume(void)
{
    uint32_t began, returned, action, finished;
    ReadyForAudit(); Shape(3U); Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_SHAPE_STEP);
    began = s_tick;
    WaitPhase(ROBOT_WALK_SHAPE_STEP_RETURN); returned = s_tick;
    WaitPhase(ROBOT_WALK_EVENT); action = s_tick;
    CHECK(returned - began == WalkBatchTime(1U));
    CHECK(action - returned == RangeTime(motion_walk_tostand, 0U, 1U));
    CHECK(MotionPlayer_GetMotion() == motion_qr_left_leg);
    WaitPhase(ROBOT_WALK_MEASURING); finished = s_tick;
    CHECK(finished - action == RangeTime(motion_qr_left_leg, 0U, 8U));
    CHECK(RobotControl_GetStatus().eventsCompleted == 1U);
    Command(CAMERA_WALK_FORWARD, 2U); /* Same ready timestamp must be rejected. */
    Tick(); CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MEASURING);
    Command(CAMERA_WALK_FORWARD, 2U); Tick();
    CHECK(RobotControl_GetStatus().walkPhase == ROBOT_WALK_MOVING);
    printf("PATH extra_walk_ms=%lu return_ms=%lu extra_to_shape_ms=%lu shape_ms=%lu resume_fresh_only=PASS\n",
        (unsigned long)(returned - began), (unsigned long)(action - returned),
        (unsigned long)(action - began), (unsigned long)(finished - action));
}
static void AuditTurns(void)
{
    uint8_t right;
    for (right = 0U; right < 2U; ++right)
    {
        uint32_t began, finished;
        MotionId_t id = right ? motion_turn_right : motion_turn_left;
        ReadyForAudit(); Command(right ? CAMERA_WALK_RIGHT : CAMERA_WALK_LEFT, 255U); Tick();
        began = s_tick;
        CHECK(RobotControl_GetStatus().targetCycles == 1U);
        WaitPhase(ROBOT_WALK_MEASURING); finished = s_tick;
        CHECK(finished - began == RangeTime(id, 0U,
            (uint8_t)(MotionGroups_Get(id)->frameCount - 1U)));
        CHECK(MotionPlayer_GetMotion() == id); CheckStand();
        Tick(); Command(CAMERA_WALK_FORWARD, 2U); Tick();
        CHECK(MotionPlayer_GetMotion() == motion_walk_forward);
        printf("PATH turn_%s_ms=%lu arg255_cycles=1 duplicate_stand=0 fresh_FWD=PASS\n",
            right ? "right" : "left", (unsigned long)(finished - began));
    }
}
static void AuditStop(void)
{
    uint8_t stop;
    const uint8_t commands[2] = { CAMERA_WALK_HOLD, CAMERA_WALK_BLOCKED };
    for (stop = 0U; stop < 2U; ++stop)
    {
        uint32_t began, requested, returned, finished;
        ReadyForAudit(); Command(CAMERA_WALK_FORWARD, 2U); Tick(); began = s_tick;
        while (MotionPlayer_GetFrameIndex() < 9U) { Tick(); }
        Command(commands[stop], 0U); Tick(); requested = s_tick;
        WaitPhase(ROBOT_WALK_TO_STAND); returned = s_tick;
        WaitPhase(stop ? ROBOT_WALK_BLOCKED : ROBOT_WALK_MEASURING); finished = s_tick;
        CHECK(returned - began == WalkBatchTime(1U));
        CHECK(finished - returned == RangeTime(motion_walk_tostand, 0U, 1U));
        CHECK(MotionPlayer_GetMotion() == motion_walk_tostand);
        printf("PATH stop_%s request_to_ready_ms=%lu remaining_cycle_ms=%lu return_ms=%lu duplicate_stand=0\n",
            stop ? "BLOCKED" : "HOLD", (unsigned long)(finished - requested),
            (unsigned long)(returned - requested), (unsigned long)(finished - returned));
    }
}
static void AuditRecoveryBoundary(void)
{
    const MotionGroup_t *walk = MotionGroups_Get(motion_walk_forward);
    const MotionGroup_t *recovery = MotionGroups_Get(motion_walk_tostand);
    uint8_t joint;
    float before;
    CHECK(memcmp(recovery->frames[0].angleDeg10, walk->frames[3].angleDeg10,
        sizeof(recovery->frames[0].angleDeg10)) == 0);
    CHECK(memcmp(recovery->frames[0].angleDeg10, walk->frames[12].angleDeg10,
        sizeof(recovery->frames[0].angleDeg10)) == 0);
    for (joint = 0U; joint < ROBOT_JOINT_COUNT; ++joint)
    {
        int delta = recovery->frames[0].angleDeg10[joint] - walk->frames[11].angleDeg10[joint];
        printf("BOUNDARY id=%u from_deg=%.1f to_deg=%.1f delta_deg=%+.1f\n",
            (unsigned int)ROBOT_SERVO_ID(joint), walk->frames[11].angleDeg10[joint] * 0.1,
            recovery->frames[0].angleDeg10[joint] * 0.1, delta * 0.1);
    }
    ResetLink(); MotionPlayer_Init();
    CHECK(MotionPlayer_StartCycles(motion_walk_forward, 1U,
        ROBOT_WALK_LOOP_START_FRAME, ROBOT_WALK_CYCLE_END_FRAME, ROBOT_WALK_STOP_FRAME));
    while (MotionPlayer_IsActive()) { CHECK(MotionPlayer_Update(20U)); }
    before = s_targets[4];
    CHECK(MotionPlayer_StartCycles(motion_walk_tostand, 1U, 0U, 1U, 1U));
    CHECK(MotionPlayer_Update(20U));
    printf("INTERPOLATION id4_first_20ms_delta_deg=%.3f (50.5deg is endpoint change, not one-tick jump)\n",
        (double)(s_targets[4] - before));
}
int main(void)
{
    AuditGroups(); AuditExtraAndResume(); AuditTurns(); AuditStop(); AuditRecoveryBoundary();
    puts("TIMING/GATING AUDIT PASSED; RECOVERY SAFETY IS NOT CERTIFIED; RUN ORIGINAL 5-DEGREE TEST");
    return 0;
}
