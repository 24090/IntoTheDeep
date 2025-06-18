package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Outtake;

@TeleOp(group = "testing", name = "Outtake Hold Testing")
public class OuttakeHoldTesting extends LinearOpMode {
    Outtake outtake;
    public void runOpMode() throws InterruptedException {
        outtake = new Outtake(hardwareMap);
        waitForStart();
        outtake.slide.goTo(outtake.slide.getPosition());
        double power = 0;
        double last_time = time;
        double dt = 0;
        while (opModeIsActive()){
            dt = time-last_time;
            last_time = time;
            if (gamepad1.dpad_up){
                power += dt;
            } else if (gamepad1.dpad_down) {
                power -= dt;
            }
            outtake.slide.setMotorPower(power);
            outtake.mirror_slide.update();
            telemetry.addData("pos", outtake.slide.getPosition());
            telemetry.addData("power", power);

            telemetry.update();

        }
    }


}
