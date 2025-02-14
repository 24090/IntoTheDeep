package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.OpenCVTesting.STREAM_HEIGHT;
import static org.firstinspires.ftc.teamcode.vision.OpenCVTesting.STREAM_WIDTH;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.GameMap;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.PoseStorer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Vision {
    OpenCvWebcam webcam;
    SampleLocationPipeline pipeline;
    Intake intake;
    MecanumDrive drive;
    public Vision(MecanumDrive drive, Intake intake, Telemetry telemetry, HardwareMap hwmap){
        this.intake = intake;
        this.drive = drive;
        WebcamName webcamName = hwmap.get(WebcamName.class, "Webcam");
        webcamName.getCameraCharacteristics();
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        webcam.setViewportRenderingPolicy(
                OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW
        );
        pipeline = new SampleLocationPipeline(telemetry);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        webcam.startStreaming(
                                STREAM_WIDTH,
                                STREAM_HEIGHT,
                                OpenCvCameraRotation.UPRIGHT
                        );
                        telemetry.addData("Camera Opened", "");
                        telemetry.update();
                    }

                    @Override
                    public void onError(int errorCode) {
                        telemetry.addData("Camera Failed", "");
                        telemetry.update();
                    }
                }
        );
    }
    public class LookAtSample implements Action {
        Pose2d sample = null;
        boolean turned = false;
        Action turn_action;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (pipeline.getAnalysis().isEmpty()) {
                return true;
            } else if (sample == null) {
                sample = pipeline.getAnalysis().get(0);
                turn_action = drive.actionBuilder(drive.pose).turn(new Rotation2d(sample.position.y, sample.position.x).toDouble()).build();
                intake.moveDown();
            }
            if (!turned) {
                turned = !turn_action.run(telemetryPacket);
                return true;
            }
            boolean extended = intake.linear_slide.extendToIter(intake.linear_slide.inToTicks(Math.max(Math.min(sample.position.norm() - 4, GameMap.MinIntakeDistance), GameMap.MaxIntakeDistance)), 50);
            return !extended;
        }
    }
    public Action LookAtSample(){
        return new LookAtSample();
    }
}
