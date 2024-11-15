package org.firstinspires.ftc.teamcode.controlled;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.util.ColorDistance;
import org.firstinspires.ftc.teamcode.util.Mechanisms;

@TeleOp(name = "linear")
public class Linearslidetest extends LinearOpMode{
    public void runOpMode(){
        DcMotor linear;
        DcMotor linear1;
        Servo servo3;
        Servo servo4;
        double servopos = 0.4;
        Mechanisms mech = new Mechanisms(hardwareMap);
        linear = hardwareMap.get(DcMotor.class, "linear");
        linear1 = hardwareMap.get(DcMotor.class, "armMotor");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");
        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linear1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //ColorDistance colorDistance = new ColorDistance(hardwareMap);
        double midPos = 500;
        double power =0 ;
        waitForStart();
        while (opModeIsActive()){
            linear1.setPower(gamepad1.left_stick_y/2);
            if (gamepad1.a){
                servo3.setPosition(1.0);
            }
            if (gamepad1.b){
                servo3.setPosition(0.0);
            }
            if (gamepad1.y){
                servo4.setPosition(1.0);
            }
            if (gamepad1.x){
                servo4.setPosition(0.0);
            }
            telemetry.addData("currentPos", linear.getCurrentPosition());
            telemetry.addData("currentPow", power);
//            telemetry.addData("distance", colorDistance.getDistance(DistanceUnit.CM));
//            telemetry.addData("RGB", colorDistance.getRGB().toString());
            telemetry.update();
        }


    }
}
