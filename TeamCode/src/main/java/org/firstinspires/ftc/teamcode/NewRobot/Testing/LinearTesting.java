package org.firstinspires.ftc.teamcode.NewRobot.Testing;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.NewRobot.Util.IntegratedMechs.NewMechActions;
import org.firstinspires.ftc.teamcode.NewRobot.Util.IntegratedMechs.NewOuttake;
import org.firstinspires.ftc.teamcode.NewRobot.Util.LinearSlides.NewIntakeSlide;
import org.firstinspires.ftc.teamcode.NewRobot.Util.LinearSlides.NewOuttakeSlide;

@TeleOp
public class LinearTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        NewOuttakeSlide outtakeSlide1 = new NewOuttakeSlide(hardwareMap, "outtake1", 2500, 0);
        NewOuttakeSlide outtakeSlide2 = new NewOuttakeSlide(hardwareMap, "outtake2", 0, -2500);
        NewOuttake outtake = new NewOuttake(outtakeSlide1, outtakeSlide2);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.y){
                outtake.outtakeSlideTo(2000);
            }
            else if  (gamepad1.a){
                outtake.outtakeSlideTo(0);
            }
            telemetry.addData("pos1", outtakeSlide1.getPosition());
            telemetry.addData("pos2", outtakeSlide2.getPosition());
            telemetry.update();
        }
    }
}
