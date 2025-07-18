package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Intake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.NoSuchElementException;
import java.util.stream.Stream;

import kotlin.Triple;

public class Vision {
    public interface SampleChecker {
        boolean run(Triple<Double, Double, Double> sample);
    }
    int STREAM_WIDTH = 640;
    int STREAM_HEIGHT = 480;
    OpenCvWebcam webcam;
    SampleLocationPipeline pipeline;
    public Vision(Telemetry telemetry, HardwareMap hwmap){
        WebcamName webcamName = hwmap.get(WebcamName.class, "Webcam");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
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

    public Action findSample(Sample out) {
        return findSample(out, (a) -> true);
    }

    public Action findSample(Sample out, SampleChecker sample_checker) {
        return (telemetry_packet) -> {
            Triple<Double, Double, Double> sample_triple;
            List<Triple<Double, Double, Double>> analysis = pipeline.getAnalysis();
            if (analysis.isEmpty()) {
                return true;
            }
            Stream<Triple<Double, Double, Double>> filtered_samples = analysis.stream().filter(
                (a) ->
                    Intake.MinDistance - 3 < (a.getSecond()) &&
                    Intake.MaxDistance - 1 > (a.getSecond()) &&
                    sample_checker.run(a)
            );
            try {
                sample_triple = filtered_samples.findFirst().get();
            } catch(NoSuchElementException e) {
                return true;
            }
            out.pose = new Pose(sample_triple.getSecond() - 2, sample_triple.getFirst(), sample_triple.getThird());
            telemetry_packet.addLine("Sample X" + sample_triple.getFirst());
            telemetry_packet.addLine("Sample Y" + (sample_triple.getSecond()));
            telemetry_packet.addLine("Sample Heading" + sample_triple.getThird());
            return false;
        };
    }
}
