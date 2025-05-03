package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.customactions.ForeverAction;
import org.firstinspires.ftc.teamcode.util.linearslides.OuttakeSlide;

public class Outtake {
    public static final double SERVO_CLOSE_MIN_TICKS = 1000;
    public static final double SERVO_CLOSE_MAX_TICKS = 4000;
    Servo servo;
    Boolean close_plan = false;
    public OuttakeSlide slide;
    public Outtake(HardwareMap hwmap) {
        this.servo = hwmap.get(Servo.class, "outtake_servo");
        this.slide = new OuttakeSlide(hwmap);
    }
    public void open(){
        servo.setPosition(0.33);
    }
    public void close(){
        servo.setPosition(1);
    }

    public void safeCloseIter(){
        if (SERVO_CLOSE_MIN_TICKS < slide.getPosition() && slide.getPosition() < SERVO_CLOSE_MAX_TICKS && close_plan) {
            close();
            close_plan = false;
        }

        // While the slide position and destination makes movement possible
        close_plan = close_plan &&(
                Math.min(slide.getPosition(),  slide.target_pos)  <  SERVO_CLOSE_MAX_TICKS
                        && SERVO_CLOSE_MIN_TICKS < Math.max(slide.getPosition(), slide.target_pos)
        );
    }

    public void backgroundIter(){
        safeCloseIter();
        slide.movementLoop();
    }

    public void safeClose(){
        close_plan = true;
    }

    public void down(){
        slide.down();
        safeClose();
    }

    public Action slideWaitAction(){
        return new RaceAction(
                new ForeverAction(this::safeCloseIter),
                slide.loopUntilDone()
        );
    }
    public Action slideDownAction(){
        return new SequentialAction(
                new InstantAction(this::down),
                slideWaitAction()
        );
    }
    public Action slideUpAction(){
        return new SequentialAction(new InstantAction(slide::up), slideWaitAction());
    }
    public Action openGateAction(){
        return new ParallelAction(new SleepAction(1), new InstantAction(this::open));
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
                                new InstantAction(this::close),
                                new SleepAction(0.5)
                        ),
                        slideDownAction()
                )
        );
    }
}

