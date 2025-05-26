package org.firstinspires.ftc.teamcode.util;

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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.linearslides.MirrorMotor;
import org.firstinspires.ftc.teamcode.util.linearslides.OuttakeSlide;
@Config
public class Outtake {
    public static  double LOW_HANG = 700;
    public static final double SERVO_CLOSE_MIN_TICKS = 1000;
    public static final double SERVO_CLOSE_MAX_TICKS = 1500;
    private final MirrorMotor mirror_slide;
    public OuttakeSlide slide;
    public OuttakeClaw claw;
    public Outtake(HardwareMap hwmap) {
        this.slide = new OuttakeSlide(hwmap);
        this.mirror_slide = new MirrorMotor(
                hwmap.get(DcMotor.class, "outtake_slide_left"),
                hwmap.get(DcMotor.class, "outtake_slide_right"),
                DcMotorSimple.Direction.REVERSE
        );
        this.claw = new OuttakeClaw(hwmap);
    }

    public void backgroundIter(){
        mirror_slide.update();
        slide.movementLoop();
    }

    public void readyTransfer(){
        slide.down();
        claw.toTransferPos();
    }
    public void readyHang(){
        claw.toTransferPos();
        slide.up();
    }
    public void readyHang2(){
        claw.toTransferPos();
        slide.goTo(LOW_HANG);
    }
    public void hang(){
        slide.down();
        claw.wrist_servo.close();
        claw.left_servo.close();
        claw.right_servo.close();
        claw.claw_servo.close();
    }
    public void readySample(){
        slide.up();
        claw.toSamplePos();
    }

    public Action slideWaitAction(){
        return new RaceAction(
                slide.loopUntilDone()
        );
    }
    public Action readyTransferAction(){
        return new SequentialAction(
                new InstantAction(this::readyTransfer),
                slideWaitAction()
        );
    }
    public Action readySampleAction(){
        return new SequentialAction(
                new InstantAction(this::readySample),
                slideWaitAction());
    }
    public Action openClawAction(){
        return new ParallelAction(new SleepAction(0.2), new InstantAction(this.claw::open));
    }
    public Action closeClawAction(){
        return new ParallelAction(new SleepAction(0.2), new InstantAction(this.claw::grab));
    }

    public Action scoreAction(){
        return new SequentialAction(
                readySampleAction(),
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

