package org.firstinspires.ftc.teamcode.util.mechanisms.outtake;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.vision.Camera;

@Config
public class OuttakeClaw {
    ServoImplEx claw_servo;
    ServoImplEx left_servo;
    ServoImplEx right_servo;
    ServoImplEx wrist_servo;

    public static double LEFT_SAMPLE = 1;
    public static double LEFT_SPECIMEN = 1;
    public static double LEFT_TRANSFER = 0.72;
    public static double LEFT_SPECIMEN_TRANSFER = 0.57;

    public static double RIGHT_TRANSFER = 0.28;
    public static double RIGHT_SAMPLE = 0;
    public static double RIGHT_SPECIMEN = 0;
    public static double RIGHT_SPECIMEN_TRANSFER = 0.43;

    public static double WRIST_TRANSFER = 0.65;
    public static double WRIST_SPECIMEN_TRANSFER = 0;
    public static double WRIST_READY_SPECIMEN = 0.3;
    public static double WRIST_SAMPLE = 0.4;

    public static double CLAW_OPEN = 0.35;
    public static double CLAW_CLOSED = 0.45;

    public OuttakeClaw(HardwareMap hardwareMap){
        claw_servo = hardwareMap.get(ServoImplEx.class, "outtake_claw_servo");
        claw_servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        right_servo = hardwareMap.get(ServoImplEx.class, "outtake_right_servo");
        right_servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        left_servo = hardwareMap.get(ServoImplEx.class, "outtake_left_servo");
        left_servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        wrist_servo = hardwareMap.get(ServoImplEx.class, "outtake_wrist_servo");
        wrist_servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void grab(){
        claw_servo.setPosition(CLAW_CLOSED);
    }

    public void open(){
        claw_servo.setPosition(CLAW_OPEN);
    }

    public void toggleGrab(){
        if (claw_servo.getPosition() > 0.5) {
            open();
        } else {
            grab();
        }
    }

    public void toTransferPos(){
        left_servo.setPosition(LEFT_TRANSFER);
        right_servo.setPosition(RIGHT_TRANSFER);
        wrist_servo.setPosition(WRIST_TRANSFER);
        claw_servo.setPosition(CLAW_CLOSED);
    }

    public void toSpecTransferPos(){
        left_servo.setPosition(LEFT_SPECIMEN_TRANSFER);
        right_servo.setPosition(RIGHT_SPECIMEN_TRANSFER);
        wrist_servo.setPosition(WRIST_SPECIMEN_TRANSFER);
        claw_servo.setPosition(CLAW_CLOSED);
    }

    public void toSamplePos(){
        left_servo.setPosition(LEFT_SAMPLE);
        right_servo.setPosition(RIGHT_SAMPLE);
        wrist_servo.setPosition(WRIST_SAMPLE);
    }

    public void readySpecimen(){
        left_servo.setPosition(LEFT_SPECIMEN);
        right_servo.setPosition(RIGHT_SPECIMEN);
        wrist_servo.setPosition(WRIST_READY_SPECIMEN);
    }

    public void disableClaw(){
        claw_servo.setPwmDisable();
        left_servo.setPwmDisable();
        right_servo.setPwmDisable();
        wrist_servo.setPwmDisable();
    }
}
