package org.firstinspires.ftc.teamcode.util.mechanisms.intake;

import static org.firstinspires.ftc.teamcode.util.GameMap.RobotLength;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.mechanisms.linearslides.IntakeSlide;

@Config
public class Intake {
    public static double MaxDistance = 11.25 + RobotLength/2;
    public static double MinDistance = 2.75 + RobotLength/2;

    public IntakeSlide slide;
    public Claw claw;

    public Intake(HardwareMap hwmap){
        this.slide = new IntakeSlide(hwmap);
        this.claw = new Claw(hwmap);
    }

    public void readyTransfer(){
        claw.loose();
        claw.toTransferPos();
        slide.moveIn();
    }

    public void readyGrab(double linear_slide_to_in){
        claw.toReadyGrabPos();
        claw.wrist_ready();
        slide.goTo(slide.inToTicks(linear_slide_to_in));
    }
    public Action pickUpAction(){
        return new SequentialAction(
                new InstantAction(() -> claw.toGrabPos()),
                new SleepAction(0.25),
                new InstantAction(claw::grab),
                new SleepAction(0.15),
                new InstantAction(claw::toReadyGrabPos));
    }
    public Action readyGrabAction(double linear_slide_to_in, double claw_rotation){
        return new ParallelAction(
                    new SleepAction(0.5), // TODO: Get better estimate of servo movement time (maybe even calculated at runtime)
                    new InstantAction(
                        () -> {
                            claw.turret_angle = claw_rotation;
                            readyGrab(linear_slide_to_in);
                        }
                    ),
                    new InstantAction(claw::open),
                    slide.loopUntilDone()
                );
    }

    public Action readyTransferAction(){
        return new SequentialAction(
                new InstantAction(this::readyTransfer),
                new ParallelAction(
                        new SequentialAction(
                                new SleepAction(0.4), // TODO: Get better estimate of servo movement time (maybe even calculated at runtime)
                                new InstantAction(this.claw::grab),
                                new SleepAction(0.1)
                        ),
                        slide.loopUntilDone()
                )


        );
    }
}
