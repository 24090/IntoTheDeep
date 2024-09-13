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

@TeleOp(name = "linear")
public class Linearslidetest extends LinearOpMode{
    public void runOpMode(){
        DcMotor linear;
        DcMotor linear1;
        Servo servo1;
        Servo servo2;
        double servopos = 0.4;
        Mechanisms mech = new Mechanisms(hardwareMap);
        linear = hardwareMap.get(DcMotor.class, "linear");
        linear1 = hardwareMap.get(DcMotor.class, "armMotor");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linear1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double midPos = 500;
        double power =0 ;
        waitForStart();
        while (opModeIsActive()){
            linear1.setPower(gamepad1.left_stick_y/2);
            telemetry.addData("currentPos", linear.getCurrentPosition());
            if (gamepad1.dpad_up){
                power += 0.01;
                while (gamepad1.dpad_up){
                    sleep(1);
                }
            }

            if (gamepad1.dpad_down){
                power -= 0.01;
                while (gamepad1.dpad_down){
                    sleep(1);
                }
            }
            linear.setPower(power);
            telemetry.addData("currentPow", power);
            telemetry.update();
        }

    }
}
