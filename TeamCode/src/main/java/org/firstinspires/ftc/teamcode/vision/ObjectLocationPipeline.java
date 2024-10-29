package org.firstinspires.ftc.teamcode.vision;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.Features2d;
import org.opencv.features2d.SimpleBlobDetector;
import org.opencv.features2d.SimpleBlobDetector_Params;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Vector;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ObjectLocationPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    public Scalar color_min;
    public Scalar color_max;

    public int stage = 0;
    public Mat input_hsv = new Mat();
    public Mat color_mask = new Mat();
    public Mat color_filtered_image = new Mat();
    public Mat keypoints_drawn_image = new Mat();
    public Mat output = new Mat();
    public Vector<Double>[] object_field_coords = new Vector[]{};

    public ObjectLocationPipeline(CameraInfo.Colors color) {
        switch (color){
            case RED:
                this.color_min = CameraInfo.red_min;
                this.color_max = CameraInfo.red_max;
                break;
            case BLUE:
                this.color_min = CameraInfo.blue_min;
                this.color_max = CameraInfo.blue_max;
                break;
            case YELLOW:
                this.color_min = CameraInfo.yellow_min;
                this.color_max = CameraInfo.yellow_max;
                break;
        }

    }
    public ObjectLocationPipeline(Telemetry telemetry){
        this.color_min = CameraInfo.blue_min;
        this.color_max = CameraInfo.blue_max;
        this.telemetry = telemetry;
    }

    public Mat processFrame(Mat input) {
        input_hsv = new Mat();
        color_mask = new Mat();
        color_filtered_image = new Mat();
        keypoints_drawn_image = new Mat();
        output = new Mat();
        // Input RGB -> HSV
        Imgproc.cvtColor(input, input_hsv, Imgproc.COLOR_RGB2HSV);
        // Gets Colors From Image (Binary)
        //Imgproc.blur(input_hsv, color_mask, new Size(3,3));
        Core.inRange(input_hsv, color_min, color_max, color_mask);
        //   Erode and dilate
        Imgproc.erode(color_mask, color_mask, new Mat(), new Point(-1, -1), 5);
        Imgproc.dilate(color_mask, color_mask, new Mat(), new Point(-1, -1), 10);

        //   add color when mask is white, add white when mask is black
        Core.bitwise_and(input_hsv, input_hsv, color_filtered_image, color_mask);
        Core.bitwise_not(color_mask, color_mask);
        Imgproc.cvtColor(color_mask, color_mask, Imgproc.COLOR_GRAY2RGB);
        Imgproc.cvtColor(color_mask, color_mask, Imgproc.COLOR_RGB2HSV);
        Core.add(color_mask, color_filtered_image, color_filtered_image);
        // Get points from colors
        //     set blob detector params
        SimpleBlobDetector_Params blob_detector_params = new SimpleBlobDetector_Params();
        blob_detector_params.set_filterByCircularity(false);
        blob_detector_params.set_filterByConvexity(false);
        blob_detector_params.set_filterByColor(true);
        blob_detector_params.set_filterByArea(true);
        blob_detector_params.set_minArea(1000);
        blob_detector_params.set_maxArea(300000);
        SimpleBlobDetector blob_detector = SimpleBlobDetector.create(blob_detector_params);
        //     detect + draw keypoints
        MatOfKeyPoint keypoints = new MatOfKeyPoint();
        blob_detector.detect(color_mask, keypoints);
        Features2d.drawKeypoints(color_filtered_image, keypoints, keypoints_drawn_image);
        // undistort image
        telemetry.addData("distorted points", keypoints);

        MatOfPoint2f points = new MatOfPoint2f();
        keypoints.convertTo(points, CvType.CV_32FC2);

        telemetry.addData("size", String.valueOf(keypoints.size()), keypoints.empty());
        telemetry.addData("empty", keypoints.empty());
//        if (points.size().area() != 0) {
//            Calib3d.undistortPoints(points, points, CameraInfo.getCameraMatrix(), CameraInfo.getDistortionCoefficients());
//        }
        //telemetry.addData("undistorted points", points);
        // camera points -> world points
        //      See: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
        //      (Y = 0)

        List<Vector<Double>> world_points = new ArrayList<>();
        List<Mat> mats = new ArrayList<>();
            mats.add(CameraInfo.getRotationMatrix().col(0));
            mats.add(CameraInfo.getRotationMatrix().col(2));
            mats.add(CameraInfo.getTranslationMatrix());
        Mat transformation_matrix = new Mat();
        Core.hconcat(mats, transformation_matrix);
        Mat multiplication = new Mat();
        Core.gemm( transformation_matrix.inv(), CameraInfo.getCameraMatrix().inv(), 1, new Mat(), 0, multiplication);
        for (int i = 0; i < points.rows(); i++) {
            Mat screen_coords = new Mat(3, 1, 6);
                screen_coords.put(0,0, points.get(i, 0)[0]);
                screen_coords.put(1,0, points.get(i, 0)[1]);
                screen_coords.put(2,0, 1);
            Mat result = new Mat();
            Core.gemm(multiplication, screen_coords, 1, new Mat(), 0, result);
            Vector<Double> result_vector = new Vector<>(3);
                    result_vector.add(result.get(0,0)[0]);
                    result_vector.add(result.get(1,0)[0]);
                    result_vector.add(result.get(2,0)[0]);
            telemetry.addData("field_coords", result_vector);
            world_points.add(result_vector);

        }
        object_field_coords = world_points.toArray(new Vector[]{});
        switch(stage){
            case 3:
                output = keypoints_drawn_image;
                break;
            case 2:
                output = color_filtered_image;
                break;
            case 1:
                output = color_mask;
                break;
            default:
                output = input_hsv;
                break;
        }
        // HSV -> RGB

        Imgproc.cvtColor(output, output, Imgproc.COLOR_HSV2RGB);
        telemetry.update();
        return output;
    }

    public Vector[] getAnalysis(){
        return object_field_coords;
    };
}
