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

import org.firstinspires.ftc.teamcode.util.customactions.ForeverAction;
import org.firstinspires.ftc.teamcode.util.customactions.TriggerAction;
import org.firstinspires.ftc.teamcode.util.linearslides.MirrorMotor;
import org.firstinspires.ftc.teamcode.util.linearslides.OuttakeSlide;
@Config
public class Outtake {
    public static  double HANG = 700;
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
        slide.movementLoop();
        mirror_slide.update();
    }

    public void readyTransfer(){
        slide.down();
        claw.toTransferPos();
    }
    public void standby(){
        slide.down();
        claw.toStandbyPos();
    }
    public void readyHang(){
        claw.toTransferPos();
        slide.goTo(HANG);
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
    public void readySpecimen(){
        slide.goTo(300);
        claw.toSpecimenPose();
    }
    public void scoreSpecimen(){
        slide.goTo(150);
        claw.toStandbyPos();
    }

    public Action slideWaitAction(){
        return new RaceAction(
                new ForeverAction(this::backgroundIter),
                new TriggerAction(() -> slide.within_error)
        );
    }
    public Action readyTransferAction(){
        return new SequentialAction(
                new InstantAction(this::readyTransfer),
                slideWaitAction()
        );
    }
    public Action standbyAction(){
        return new SequentialAction(
                new InstantAction(this::standby),
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
                readySampleAction(),
                openClawAction(),
                new InstantAction(this::standby)
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

