package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.linearslides.IntakeSlide;

public class Intake {
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
        claw.rotate(claw_rotation);
        linear_slide.goTo(linear_slide.inToTicks(linear_slide_to_in));
    }

    public Action pickUpAction(){
        return new SequentialAction(
                new InstantAction(claw::toGrabPos),
                new InstantAction(claw::grab),
                new SleepAction(0.2),
                new InstantAction(claw::toReadyGrabPos));
    }
    public Action readyGrabAction(double linear_slide_to_in, double claw_rotation){
        return new ParallelAction(
                    new SleepAction(1), // TODO: Get better estimate of servo movement time (maybe even calculated at runtime)
                    new InstantAction(() -> readyGrab(linear_slide_to_in, claw_rotation)),
                    linear_slide.loopUntilDone()
                );
    }

    public Action readyTransferAction(){
        return new ParallelAction(
                new SleepAction(1), // TODO: Get better estimate of servo movement time (maybe even calculated at runtime)
                new InstantAction(this::readyTransfer),
                linear_slide.loopUntilDone()
        );
    }
    public Action fullTransferAction(){
        return new SequentialAction(
                readyTransferAction(),
                new InstantAction(claw::open),
                new SleepAction(0.2) // TODO: Get better estimate of servo movement time (maybe even calculated at runtime)
        );
    }
}
