package org.firstinspires.ftc.teamcode.OldRobot.Util;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.OldRobot.Util.linearslides.IntakeSlide;

public class Intake {
    Servo intake_servo_a1;
    Servo intake_servo_b;
    Servo intake_servo_a2;
    public IntakeSlide linear_slide;
    public Intake(Servo intake_servo_a1, Servo intake_servo_a2, Servo intake_servo_b, DcMotor motor){
        this.intake_servo_a1 = intake_servo_a1;
        this.intake_servo_a2 = intake_servo_a2;
        this.intake_servo_b  = intake_servo_b;
        this.linear_slide = new IntakeSlide(motor);
        intake_servo_a1.setPosition(0.83);
        intake_servo_a2.setPosition(1-0.83);
    }
    public double moveUp(){
        intake_servo_a1.setPosition(0.83);
        intake_servo_a2.setPosition(1-0.83);
        return Math.abs((0.83 - intake_servo_a1.getPosition())) + Math.abs((1-0.83 - intake_servo_a2.getPosition()));
    }
    public double moveDown(){
        intake_servo_a1.setPosition(0.05);
        intake_servo_a2.setPosition(1 - 0.05);
        return Math.abs((0.05 - intake_servo_a1.getPosition())) + Math.abs((1-0.05 - intake_servo_a2.getPosition()));
    }
    public void grab(){
        intake_servo_b.setPosition(0);
    }
    public void hold(){
        intake_servo_b.setPosition(0.44);
    }
    public void stop(){
        intake_servo_b.setPosition(0.5);
    }
    public void release(){
        intake_servo_b.setPosition(0.7);
    }
    public void slideOut(){
        linear_slide.extendToBreaking(1000, 50);
    }
    public void slideIn(){linear_slide.extendToBreaking(-50, 50);}

    public Thread slideInAsync(){
        return linear_slide.extendToAsync(0, 50);
    }

    public void transferSample(){
        this.hold();
        this.moveUp();
        this.slideIn();
        while (intake_servo_a1.getPosition() < 0.83 && intake_servo_a2.getPosition() > 1-0.83){}
        this.release();
        try {Thread.sleep(1000);} catch(InterruptedException e) {
            e.printStackTrace();
        }
        this.stop();
    }
    public void readyIntake(){
        this.moveDown();
        this.slideOut();
        while (intake_servo_a1.getPosition() > 0.05){}
        this.grab();
    }
    public Thread slideOutAsync(){
        Thread thread = new Thread(this::slideOut);
        thread.start();
        return thread;
    }
    public void slideStop(){
        linear_slide.stop();
    }

    public static class GameMap {
        // -------OBSERVATION CORNERS-------
        // The very corner of the field in which the observation zone is
        public static final Vector2d ObservationBlueCorner = new Vector2d(-70.5, 70.5);
        public static final Vector2d ObservationRedCorner = new Vector2d(70.5, -70.5);
        // -------NET CORNERS---------------
        // The very corner of the field in which the net zone is
        public static final Vector2d NetRedCorner = new Vector2d(-70.5, -70.5);
        public static final Vector2d NetBlueCorner = new Vector2d(70.5, 70.5);
        // -------ASCENT ZONE EDGES---------
        // The center of the edge of the submersible zone that touches the ascent zone
        public static final Vector2d AscentZoneEdgeBlue = new Vector2d(27.5/2, 0);
        public static final Vector2d AscentZoneEdgeRed = new Vector2d(-27.5/2, 0);
        // -------SUBMERSIBLE EDGES----------
        // The center of the edge of the edge of the submersible zone that doesn't touch the ascent zone
        public static final Vector2d SubmersibleBlueEdge = new Vector2d(0, 42.75/2);
        public static final Vector2d SubmersibleRedEdge = new Vector2d(0, -42.75/2);
        // -------SPIKE MARKS----------------
        // The center of the spike mark
        // RED
        public static final Vector2d SpikeMarkRedInner = new Vector2d(47, -23.5-3.5/2);
        public static final Vector2d SpikeMarkRedCenter = new Vector2d(58, -23.5-3.5/2);
        public static final Vector2d SpikeMarkRedOuter = new Vector2d(68, -23.5-3.5/2);
        // BLUE
        public static final Vector2d SpikeMarkBlueInner = new Vector2d(-47, 23.5+3.5/2);
        public static final Vector2d SpikeMarkBlueCenter = new Vector2d(-58, 23.5+3.5/2);
        public static final Vector2d SpikeMarkBlueOuter = new Vector2d(-68, 23.5+3.5/2);
        // NEUTRAL RIGHT
        // right = +x
        public static final Vector2d SpikeMarkNeutralRightInner = new Vector2d(47, 23.5+3.5/2);
        public static final Vector2d SpikeMarkNeutralRightCenter = new Vector2d(58, 23.5+3.5/2);
        public static final Vector2d SpikeMarkNeutralRightOuter = new Vector2d(68, 23.5+3.5/2);
        // NEUTRAL LEFT
        // left = -x
        public static final Vector2d SpikeMarkNeutralLeftInner = new Vector2d(-47, -23.5-3.5/2);
        public static final Vector2d SpikeMarkNeutralLeftCenter = new Vector2d(-58, -23.5-3.5/2);
        public static final Vector2d SpikeMarkNeutralLeftOuter = new Vector2d(-68, -23.5-3.5/2);
        // -------APRIL TAGS-----------------
        // the center of the april tag in x/y
        // BLUE
        public static final Vector2d AprilTagBlueCenter = new Vector2d(0, 70.5);
        public static final Vector2d AprilTagBlueLeft = new Vector2d(-70.5, 47);
        public static final Vector2d AprilTagBlueRight = new Vector2d(70.5, 47);
        public static final int AprilTagBlueCenterID = 12;
        public static final int AprilTagBlueLeftID = 11;
        public static final int AprilTagBlueRightID = 13;
        // RED
        public static final Vector2d AprilTagRedCenter = new Vector2d(0, -70.5);
        public static final Vector2d AprilTagRedLeft = new Vector2d(-70.5, -47);
        public static final Vector2d AprilTagRedRight = new Vector2d(70.5, -47);
        public static final int AprilTagRedCenterID = 15;
        public static final int AprilTagRedLeftID = 16;
        public static final int AprilTagRedRightID = 14;

        //------ROBOT CONSTANTS-------------
        public static final double RobotWidth = 18.0;
        public static final double RobotLength = 17.4;
        public static final double OuttakeDistance = 6.7 + RobotLength/2;
        public static final double MaxIntakeDistance = 10.2 + RobotLength/2;
    }
}
