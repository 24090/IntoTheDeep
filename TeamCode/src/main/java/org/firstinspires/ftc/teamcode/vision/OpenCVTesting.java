package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.GameMap;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.PoseStorer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(group = "testing", name = "OpenCV Testing")
public class OpenCVTesting extends LinearOpMode {

	static final int STREAM_WIDTH = 640;
	static final int STREAM_HEIGHT = 480;
	OpenCvWebcam webcam;
	SampleLocationPipeline pipeline;
	Intake intake;
	MecanumDrive drive;
	@Override
	public void runOpMode() {
		drive = new MecanumDrive(hardwareMap, PoseStorer.pose);
		intake = new Intake(
				hardwareMap.get(Servo.class, "intake_servo_a1"),
				hardwareMap.get(Servo.class, "intake_servo_a2"),
				hardwareMap.get(Servo.class, "intake_servo_b"),
				hardwareMap.get(DcMotor.class, "intake_motor")
		);
		WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
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
		waitForStart();
		LookAtSample(this);
	}
	public void LookAtSample(LinearOpMode opMode){
		while (pipeline.getAnalysis().isEmpty() && opMode.opModeIsActive()){}
		if (opModeIsActive()){
			Pose2d sample = pipeline.getAnalysis().get(0);
			intake.moveDown();
			Actions.runBlocking(drive.actionBuilder(drive.pose).turn(new Rotation2d(sample.position.y, sample.position.x).toDouble()).build());
			intake.linear_slide.extendToBreaking(intake.linear_slide.inToTicks(Math.max(Math.min(sample.position.norm() - 4, GameMap.MinIntakeDistance), GameMap.MaxIntakeDistance)), 50);
			while (opModeIsActive()) {
				telemetry.addData("Image Analysis:", pipeline.getAnalysis());
				telemetry.update();
			}
		}
	}
}
