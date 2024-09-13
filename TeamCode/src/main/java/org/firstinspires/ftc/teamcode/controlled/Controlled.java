package org.firstinspires.ftc.teamcode.controlled;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Mechanisms;

/**
 * TODO:
 *  give pose to next OpMode/take pose from previous
 *  assisted aim if necessary
 *  dpad mode if necessary
 */
@TeleOp(name = "Controller")
public class Controlled extends LinearOpMode{
    public void runOpMode(){
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        DcMotor linear;
        Servo servo1;
        Servo servo2;
        double servopos = 0.4;
        Mechanisms mech = new Mechanisms(hardwareMap);
        linear = hardwareMap.get(DcMotor.class, "linear");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        double power = 0;
        double realpower = 0;
        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // y and x ARE correct here, as "forward" is represented by the X coordinate for roadrunner and the Y coordinate on the controller
        waitForStart();
        while (opModeIsActive()){
            Vector2d linearVelocity = new Vector2d(gamepad1.left_stick_y, -gamepad1.left_stick_x);
            double angularVelocity = -gamepad1.right_stick_x;
            drive.setDrivePowers(new PoseVelocity2d(linearVelocity, angularVelocity));
            if (gamepad1.dpad_down){
                power = 0.5;
            }
            else if (gamepad1.dpad_up){
                power = -0.5;
            } else {
                if (linear.getCurrentPosition() > -810){
                    power = -0.14;
                }
                else if (linear.getCurrentPosition() < -980){
                    power = 0.14;
                } else {
                    power = 0;
                }
            }
            if (gamepad1.a) {
                servo2.setPosition(0.0);
            } else if (gamepad1.b) {
                servo2.setPosition(1.0);
            } else {
                servo2.setPosition(0.5);
            }
            if (gamepad1.dpad_left){
                servopos -= 0.007;
            }
            if (gamepad1.dpad_right){
                servopos += 0.007;
            }
            if (servopos < 0.4){
                servopos = 0.4;
            }
            if (servopos > 1){
                servopos = 1;
            }
            servo1.setPosition(servopos);
            telemetry.addData("servo", servopos);
            telemetry.addData("currentPos", linear.getCurrentPosition());
            telemetry.update();
            realpower = realpower + 0.1 * (power-realpower);
            linear.setPower(realpower);
        }
    }
}
