package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.GameMap.RobotLength;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.linearslides.IntakeSlide;

@Config
public class Intake {
    public static double MaxDistance = 12.5 + RobotLength/2;
    public static double MinDistance = 2.5 + RobotLength/2;

    public IntakeSlide linear_slide;
    public Claw claw;

    public Intake(HardwareMap hwmap){
        this.linear_slide = new IntakeSlide(hwmap);
        this.claw = new Claw(hwmap);
    }

    public void readyTransfer(){
        claw.grab();
        claw.toTransferPos();
        linear_slide.moveIn();
    }

    public void readyGrab(double linear_slide_to_in, double claw_rotation){
        claw.open();
        claw.toReadyGrabPos();
        claw.rotate_turret(claw_rotation);
        linear_slide.goTo(linear_slide.inToTicks(linear_slide_to_in));
    }

    public Action pickUpAction(){
        return new SequentialAction(
                new InstantAction(claw::toGrabPos),
                new InstantAction(claw::grab),
                new SleepAction(0.4),
                new InstantAction(claw::toReadyGrabPos));
    }
    public Action readyGrabAction(double linear_slide_to_in, double claw_rotation){
        return new ParallelAction(
                    new SleepAction(0.5), // TODO: Get better estimate of servo movement time (maybe even calculated at runtime)
                    new InstantAction(() -> readyGrab(linear_slide_to_in, claw_rotation)),
                    linear_slide.loopUntilDone()
                );
    }

    public Action readyTransferAction(){
        return new ParallelAction(
                new SleepAction(0.5), // TODO: Get better estimate of servo movement time (maybe even calculated at runtime)
                new InstantAction(this::readyTransfer),
                linear_slide.loopUntilDone()
        );
    }
    public Action fullTransferAction(){
        return new SequentialAction(
                readyTransferAction(),
                new InstantAction(claw::open),
                new SleepAction(0.4) // TODO: Get better estimate of servo movement time (maybe even calculated at runtime)
        );
    }
}
