package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.linearslides.OuttakeSlide;

public class Outtake {
    public static final double SERVO_CLOSE_MIN_TICKS = 1000;
    public static final double SERVO_CLOSE_MAX_TICKS = 4000;
    Servo servo;
    public OuttakeSlide slide;
    public Outtake(HardwareMap hwmap) {
        this.servo = hwmap.get(Servo.class, "outtake_servo");
        this.slide = new OuttakeSlide(hwmap);
    }
    public void open(){
        servo.setPosition(0);
    }
    public void close(){
        servo.setPosition(0.66);
    }

    public void safeClose(){
        Thread t = new Thread(() -> {
                // While the slide position and destination makes movement possible
                while (
                    Math.min(slide.getPosition(),  slide.target_pos)  <  SERVO_CLOSE_MAX_TICKS
                    && SERVO_CLOSE_MIN_TICKS < Math.max(slide.getPosition(), slide.target_pos)
                ){
                    if (SERVO_CLOSE_MIN_TICKS < slide.getPosition() && slide.getPosition() < SERVO_CLOSE_MAX_TICKS) {
                        servo.close();
                        break;
                    }
                };
            }
        );
        t.start();
    }

    public void down(){
        slide.down();
        safeClose();
    }
    public Action slideDownAction(){
        return new ParallelAction(
                new InstantAction(this::down),
                new TriggerAction(() -> slide.within_error)
        );
    }
    public Action slideUpAction(){
        return new ParallelAction(new TriggerAction(() -> slide.within_error), new InstantAction(slide::up));
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
                new InstantAction(this::down)
        );
    }
    public Action fullScoreAction(){
        return new SequentialAction(
                slideUpAction(),
                openGateAction(),
                new ParallelAction(
                        new SequentialAction(
                                closeGateAction(),
                                new SleepAction(0.5)
                        ),
                        slideDownAction()
                )
        );
    }
}

