package org.firstinspires.ftc.teamcode.util.mechanisms.outtake;

import static org.firstinspires.ftc.teamcode.util.CustomActions.foreverAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.triggerAction;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.mechanisms.linearslides.MirrorMotor;
import org.firstinspires.ftc.teamcode.util.mechanisms.linearslides.OuttakeSlide;

@Config
public class Outtake {
    public static int HANG_UP = 1600;
    public static int HANG_DOWN = 1300;
    public static int READY_SPECIMEN = 175;
    public final MirrorMotor mirror_slide;
    public OuttakeSlide slide;
    public OuttakeClaw claw;
    public Outtake(HardwareMap hwmap) {
        this.slide = new OuttakeSlide(hwmap);
        this.mirror_slide = new MirrorMotor(
                hwmap.get(DcMotor.class, "outtake_slide_right"),
                hwmap.get(DcMotor.class, "outtake_slide_left"),
                DcMotorSimple.Direction.FORWARD
        );
        this.claw = new OuttakeClaw(hwmap);
    }

    public void backgroundIter(){
        slide.movementLoop();
        mirror_slide.update();
    }

    public void readyTransfer(){
        slide.down();
        claw.toTransferPos();
    }

    public void readySpecTransfer(){
        slide.down();
        claw.toSpecTransferPos();
    }

    public void readySample(){
        slide.up();
        claw.toSamplePos();
    }

    public void readyHang(){
        claw.toTransferPos();
        slide.goTo(HANG_UP);
    }
    public void hang(){
        slide.goTo(HANG_DOWN);
        claw.wrist_servo.close();
        claw.left_servo.close();
        claw.right_servo.close();
        claw.claw_servo.close();
    }
    public void readySpecimen(){
        slide.goTo(READY_SPECIMEN);
        claw.readySpecimen();
    }

    public Action slideWaitAction(){
        return new RaceAction(
                foreverAction(this::backgroundIter),
                triggerAction(() -> slide.within_error)
        );
    }
    public Action readySpecimenAction(){
        return new SequentialAction(
                new InstantAction(this::readySpecimen),
                new ParallelAction(
                    new SleepAction(0.5),
                    slideWaitAction()
                )

        );
    }
    public Action readyTransferAction(){
        return new SequentialAction(
                new InstantAction(this::readyTransfer),
                slideWaitAction()
        );
    }
    public Action readySpecTransferAction(){
        return new SequentialAction(
                new InstantAction(this::readySpecTransfer),
                slideWaitAction()
        );
    }
    public Action standbyAction(){
        return new SequentialAction(
                new InstantAction(this::readyTransfer),
                slideWaitAction()
        );
    }
    public Action readySampleAction(){
        return new SequentialAction(
                new InstantAction(this::readySample),
                slideWaitAction()
        );
    }
    public Action openClawAction(){
        return new ParallelAction(new SleepAction(0.2), new InstantAction(this.claw::open));
    }
    public Action closeClawAction(){
        return new ParallelAction(new SleepAction(0.2), new InstantAction(this.claw::grab));
    }

    public Action scoreAction(){
        return new SequentialAction(
                openClawAction(),
                new InstantAction(this::readyTransfer)
        );
    }
    public Action fullScoreAction(){
        return new SequentialAction(
                readySampleAction(),
                openClawAction(),
                readyTransferAction()
        );
    }
}

