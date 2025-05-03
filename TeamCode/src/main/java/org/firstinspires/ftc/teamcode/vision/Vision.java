package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import kotlin.Triple;

public class Vision {
    int STREAM_WIDTH= 640;
    int STREAM_HEIGHT = 480;
    OpenCvWebcam webcam;
    SampleLocationPipeline pipeline;
    public Vision(Telemetry telemetry, HardwareMap hwmap){
        WebcamName webcamName = hwmap.get(WebcamName.class, "Webcam");
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
                                OpenCvCameraRotation.UPSIDE_DOWN
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
    Action findSample(Pose2d[] sample_out) {
        return (telemetry_packet) -> {
            if (pipeline.getAnalysis().isEmpty()) {
                return true;
            }
            Triple<Double, Double, Double>  sample_triple = pipeline.getAnalysis().get(0);
            sample_out[0] = new Pose2d(sample_triple.getFirst(), sample_triple.getSecond(), sample_triple.getThird());
            return false;
        };
    }
}
