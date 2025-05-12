package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.RobotActions;
import org.firstinspires.ftc.teamcode.util.customactions.ForeverAction;
import org.firstinspires.ftc.teamcode.util.customactions.FutureAction;
import org.firstinspires.ftc.teamcode.util.customactions.TriggerAction;

@TeleOp(group = "testing")
public class VisionTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Intake intake = new Intake(hardwareMap);
        Vision vision = new Vision(
                telemetry,
                hardwareMap
        );
        waitForStart();
        while (opModeIsActive()){
            Sample sample = new Sample();
            Actions.runBlocking(
                    new ParallelAction(
                        new SequentialAction(
                            vision.findSample(sample),
                            new FutureAction( () ->
                                RobotActions.reachSample(sample.pose, intake, drive)
                            ),
                            intake.pickUpAction(),
                            intake.fullTransferAction(),
                            new TriggerAction(() -> gamepad1.a)
                        ),
                        new ForeverAction(() -> {
                            if (sample.pose != null) {
                                telemetry.addData("sample x", sample.pose.position.x);
                                telemetry.addData("sample y", sample.pose.position.y);
                                telemetry.addData("sample Θ", sample.pose.heading);
                                telemetry.update();
                            }

                        })
                    )

            );
        }
    }
}
