package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.linearslides.OuttakeSlide;

public class Outtake {
    Servo outtake_servo;
    public OuttakeSlide linear_slide;
    public Outtake(HardwareMap hwmap) {
        this.outtake_servo = hwmap.get(Servo.class, "outtake_servo");
        this.linear_slide = new OuttakeSlide(hwmap);
    }
    public void open(){
        outtake_servo.setPosition(0);
    }
    public void close(){
        outtake_servo.setPosition(0.66);
    }
    public Action slideDownAction(){
        return new ParallelAction(new TriggerAction(() -> linear_slide.within_error), new InstantAction(linear_slide::down));
    }
    public Action slideUpAction(){
        return new ParallelAction(new TriggerAction(() -> linear_slide.within_error), new InstantAction(linear_slide::up));
    }
    public Action openGateAction(){
        return new ParallelAction(new SleepAction(1), new InstantAction(this::open));
    }
    public Action closeGateAction(){
        return new InstantAction(this::close);
    }
    public Action scoreAction(){
        return new SequentialAction(
                slideUpAction(),
                openGateAction(),
                new InstantAction(linear_slide::down)
        );
    }
    public Action fullScoreAction(){
        return new SequentialAction( slideUpAction(), openGateAction(), slideDownAction());
    }
    public Action outtakeSlideUpAction(){
        return new ParallelAction(slideUpAction(), new SequentialAction(new SleepAction(0.3), closeGateAction()));
    }
    public Action outtakeSlideDownAction(){
        return new ParallelAction(slideDownAction(), closeGateAction());
    }
}
