#define main existing_mode_test_main
#include "test_three_modes.c"
#undef main

#ifndef TEST_NONZERO_SHAPE_END
#define TEST_NONZERO_SHAPE_END 0
#endif

static uint8_t NeedsRecovery(uint8_t shape, MotionId_t motion)
{
    return (uint8_t)((shape == 6U && TEST_NONZERO_SHAPE_END) ||
        (ROBOT_SKIP_DUPLICATE_SHAPE_STAND == 0U && motion != motion_qr_left_leg));
}

static void TestEveryImageBody(void)
{
    uint8_t shape;
    for (shape=1U; shape<=6U; ++shape)
    {
        uint32_t began, duration, expected;
        uint8_t recover;
        MotionId_t motion=MotionGroups_FromQr(shape);
        const MotionGroup_t *group=MotionGroups_Get(motion);
        ResetRobot(); WaitPhase(ROBOT_WALK_MEASURING); Shape(shape); Tick();
        WaitPhase(ROBOT_WALK_EVENT);
        CHECK(MotionPlayer_GetMotion()==motion);
        CHECK(MotionPlayer_GetFrameIndex()==0U); /* All group frames are preserved. */
        began=s_tick;
        duration=RangeTime(motion,0U,(uint8_t)(group->frameCount-1U));
        recover=NeedsRecovery(shape,motion);
        while(RobotControl_GetStatus().walkPhase==ROBOT_WALK_EVENT)
        {
            Command(CAMERA_WALK_FORWARD,2U); Tick();
            CHECK(s_tick-began<=duration);
        }
        CHECK(s_tick-began==duration);
        if(recover)
        {
            CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_RECOVER);
            CHECK(MotionPlayer_GetMotion()==motion_stand);
            CHECK(RobotControl_GetStatus().eventsCompleted==0U);
        }
        else if(ROBOT_WALK_SETTLE_MS==0U)
        {
            CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_MEASURING);
            CHECK(MotionPlayer_GetMotion()==motion);
            CHECK(RobotControl_GetStatus().eventsCompleted==1U);
        }
        else
        {
            CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_SETTLING);
            CHECK(MotionPlayer_GetMotion()==motion);
            CHECK(RobotControl_GetStatus().eventsCompleted==0U);
        }
        WaitPhase(ROBOT_WALK_MEASURING);
        expected=duration+ROBOT_WALK_SETTLE_MS+(recover?RangeTime(motion_stand,0U,0U):0U);
        CHECK(s_tick-began==expected);
        CHECK(RobotControl_GetStatus().eventsCompleted==1U);
        CheckStand();
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
        began=s_tick;
        while(s_tick-began<ROBOT_STM_CORRECTION_WAIT_MS+500U)
        { Tick(); CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_MEASURING); }
        Command(CAMERA_WALK_FORWARD,2U); Tick();
        CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_MOVING);
#endif
    }
    puts("PASS all six image groups preserve every frame; neutral ends skip duplicate stand, nonneutral ends recover, fresh vision remains required");
}

static void TestImageFinalTargetRetry(void)
{
    const MotionGroup_t *group=MotionGroups_Get(motion_qr_head_shake);
    uint8_t last=(uint8_t)(group->frameCount-1U);
    uint32_t i;
    ResetRobot(); WaitPhase(ROBOT_WALK_MEASURING); Shape(6U); Tick();
    WaitPhase(ROBOT_WALK_EVENT);
    while(MotionPlayer_GetFrameIndex()!=last){Tick();}
    s_servoAccepted=0U;
    for(i=0U;i<RangeTime(motion_qr_head_shake,last,last)+100U;i+=20U){Tick();}
    CHECK(MotionPlayer_IsActive());
    CHECK(RobotControl_GetStatus().walkPhase==ROBOT_WALK_EVENT);
    CHECK(RobotControl_GetStatus().eventsCompleted==0U);
    s_servoAccepted=1U;Tick();
    WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(RobotControl_GetStatus().eventsCompleted==1U);
    puts("PASS unsuccessful final image target cannot complete the event early");
}

static void TestStartupAndWalkingWait(void)
{
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    uint32_t began;
#endif
    ResetRobot();WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(s_tick==RangeTime(motion_stand,0U,0U)+ROBOT_WALK_STARTUP_SETTLE_MS);
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    Tick();Command(CAMERA_WALK_FORWARD,2U);Tick();
    WaitPhase(ROBOT_WALK_TO_STAND);
    began=s_tick;
    WaitPhase(ROBOT_WALK_MEASURING);
    CHECK(s_tick-began==RangeTime(motion_walk_tostand,0U,
          (uint8_t)(MotionGroups_Get(motion_walk_tostand)->frameCount-1U))+ROBOT_WALK_SETTLE_MS);
    CHECK(MotionPlayer_GetMotion()==motion_walk_tostand);
#endif
    puts("PASS startup and walking have exactly group durations plus configured wait, no extra control-period wait");
}

int main(void)
{
    TestEveryImageBody();TestImageFinalTargetRetry();TestStartupAndWalkingWait();
    puts("ALL GROUP ONLY STAND TESTS PASSED");
    return 0;
}
