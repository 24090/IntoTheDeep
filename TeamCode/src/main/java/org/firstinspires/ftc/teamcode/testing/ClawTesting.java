package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Claw;
@TeleOp
public class ClawTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(this, hardwareMap);
        double i = 0.13;
        double j = 0.72;
        claw.elbow1ServoSetPosition(i);
        claw.elbow2ServoSetPosition(j);
        waitForStart();
        while(opModeIsActive()){
            /*if (gamepad1.dpad_up){
                if (i<1){
                    i = i+0.01;
                    sleep(50);
                }
            }
            if (gamepad1.dpad_down){
                if (0<i){
                    i = i-0.01;
                    sleep(50);
                }
            }
            if (gamepad1.y){
                if (j<1){
                    j = j+0.01;
                    sleep(50);
                }
            }
            if (gamepad1.a){
                if (0<j){
                    j = j-0.01;
                    sleep(50);
                }
            }*/
            if (gamepad1.a){
                claw.elbow1ServoSetPosition(0.13);
                claw.elbow2ServoSetPosition(0.72);
            }
            if (gamepad1.b){
                claw.elbow1ServoSetPosition(0.9);
                claw.elbow2ServoSetPosition(0.03);
            }
            telemetry.addData("pos1", i);
            telemetry.addData("pos2", j);
            telemetry.update();
        }
    }
}
