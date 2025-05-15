package org.firstinspires.ftc.teamcode.vision;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Claw;

import java.util.ArrayList;

import kotlin.Triple;

@TeleOp
public class TurretVision extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        Claw claw = new Claw(hardwareMap);
        Vision vision = new Vision(telemetry, hardwareMap);
        double angle = 0;
        waitForStart();
        while (opModeIsActive()) {
            ArrayList<Triple<Double, Double, Double>> data = vision.pipeline.getAnalysis();
            if (!data.isEmpty()) {
                angle = data.get(0).getThird();
            }
            claw.rotate(angle);
        }

    }
}
