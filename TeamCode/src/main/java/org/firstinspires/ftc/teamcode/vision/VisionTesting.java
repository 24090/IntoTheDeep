package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.RobotActions;
import org.firstinspires.ftc.teamcode.util.customactions.FutureAction;
import org.firstinspires.ftc.teamcode.util.customactions.TriggerAction;

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
            Pose2d[] sample_out = new Pose2d[1];
            Actions.runBlocking(
                    new SequentialAction(
                            vision.findSample(sample_out),
                            new FutureAction( () ->
                                    RobotActions.reachSample(sample_out[0], intake, drive)
                            ),
                            intake.pickUpAction(),
                            intake.fullTransferAction(),
                            new TriggerAction(() -> gamepad1.a)
                    )
            );
        }
    }
}
