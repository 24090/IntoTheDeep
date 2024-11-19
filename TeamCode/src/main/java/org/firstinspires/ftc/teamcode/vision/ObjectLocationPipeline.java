package org.firstinspires.ftc.teamcode.vision;
import static java.util.Arrays.asList;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
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
import java.util.stream.Stream;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ObjectLocationPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    public Scalar color_min;
    public Scalar color_max;

    public int stage = 4;
    public Mat input_undistort = new Mat();
    public Mat input_hsv  = new Mat();
    public Mat color_mask = new Mat();
    public Mat color_filtered_image = new Mat();
    public Mat keypoints_drawn_image = new Mat();
    public Mat output = new Mat();
    Mat empty_mat = new Mat();
    Mat transformation_matrix = new Mat();
    Mat multiplication = new Mat();
    public Vector<Double>[] object_field_coords = new Vector[]{};
    public ObjectLocationPipeline(CameraInfo.Colors color, Telemetry telemetry) {
        this.telemetry = telemetry;
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
        this.color_min = CameraInfo.yellow_min;
        this.color_max = CameraInfo.yellow_max;
        this.telemetry = telemetry;
    }

    public Mat processFrame(Mat input) {
        empty_mat.copyTo(color_filtered_image);
//        keypoints_drawn_image = new Mat();
//        output = new Mat();
        // Undistort
        Calib3d.undistort(input, input_undistort, CameraInfo.getCameraMatrix(), CameraInfo.getDistortionCoefficients());
        // Input RGB -> HSV
        Imgproc.cvtColor(input_undistort, input_undistort, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, input_hsv, Imgproc.COLOR_RGB2HSV);
        // Gets Colors From Image (Binary)
        //Imgproc.blur(input_hsv, color_mask, new Size(3,3));
        Core.inRange(input_undistort, color_min, color_max, color_mask);
        //   Erode and dilate
        Imgproc.erode(color_mask, color_mask, empty_mat, new Point(-1, -1), 5);
        Imgproc.dilate(color_mask, color_mask, empty_mat, new Point(-1, -1), 10);

        //   add color when mask is white, add white when mask is black
        Core.bitwise_and(input_undistort, input_undistort, color_filtered_image, color_mask);
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
        // detect + draw keypoints
        MatOfKeyPoint keypoints = new MatOfKeyPoint();
        blob_detector.detect(color_mask, keypoints);
        Features2d.drawKeypoints(color_filtered_image, keypoints, keypoints_drawn_image);
        // only do if points exist
        if (keypoints.size().area() != 0) {
            MatOfPoint2f points = new MatOfPoint2f();
            List<Point> point_list = new ArrayList<>();
            for (KeyPoint keypoint: keypoints.toList()){
                point_list.add(keypoint.pt);
            }
            points.fromList(point_list);
            // camera points -> world points
            //      See: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
            //      (Z = 0)
            List<Vector<Double>> world_points = new ArrayList<>();
            List<Mat> mats = new ArrayList<>();
            mats.add(CameraInfo.getRotationMatrix().col(0));
            mats.add(CameraInfo.getRotationMatrix().col(1));
            mats.add(CameraInfo.getTranslationMatrix());
            Core.hconcat(mats, transformation_matrix);
            // transformation_matrix = [ R_1 | R_2 | T ]
            Core.gemm(CameraInfo.getCameraMatrix(), transformation_matrix, 1, empty_mat, 0, multiplication);
            multiplication = multiplication.inv();
            for (int i = 0; i < points.rows(); i++) {
                Mat screen_coords = new Mat(3, 1, 6);
                screen_coords.put(0,0, points.get(i, 0)[0]);
                screen_coords.put(1,0, points.get(i, 0)[1]);
                screen_coords.put(2,0, 1);
                // screen_coords = [u, v, 1]
                Mat result = new Mat();
                Core.gemm(multiplication, screen_coords, 1, empty_mat, 0, result);
                double scale = 1.0;//result.get(2,0)[0];
                Vector<Double> result_vector = new Vector<>(3);
                result_vector.add(result.get(0,0)[0] * scale);
                result_vector.add(result.get(1,0)[0] * scale);
                result_vector.add(result.get(2,0)[0] * scale);
                telemetry.addData("field_coords", result_vector);
                world_points.add(result_vector);
            }
            object_field_coords = world_points.toArray(new Vector[]{});
        }
        switch(stage){
            case 4:
                keypoints_drawn_image.copyTo(output);
                break;
            case 3:
                color_filtered_image.copyTo(output);
                break;
            case 2:
                color_mask.copyTo(output);
                break;
            case 1:
                input_undistort.copyTo(output);
                break;
            default:
                input_hsv.copyTo(output);
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
