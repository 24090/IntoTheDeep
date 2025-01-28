package org.firstinspires.ftc.teamcode.vision;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SampleLocationPipeline extends OpenCvPipeline {

    Telemetry telemetry;
    public Scalar color_min;
    public Scalar color_max;
    public int stage = 4;
    public Mat input_undistort = new Mat();
    public Mat input_hsv  = new Mat();
    public Mat color_mask = new Mat();
    public Mat color_filtered_image = new Mat();
    public Mat output = new Mat();
    final Mat empty_mat = new Mat();
    Mat hierarchy = new Mat();
    Mat greyscale = new Mat();
    Mat reprojection_matrix = new Mat();
    Mat top_reprojection_matrix = new Mat();
    public List<Vector<Double>> object_field_coords = new ArrayList<>();
    public SampleLocationPipeline(Camera.Colors color, Telemetry telemetry) {
        this.reprojection_matrix = Camera.getReprojectionMatrix(false);
        this.top_reprojection_matrix = Camera.getReprojectionMatrix(true);
        this.telemetry = telemetry;
        switch (color){
            case RED:
                this.color_min = Camera.red_min;
                this.color_max = Camera.red_max;
                break;
            case BLUE:
                this.color_min = Camera.blue_min;
                this.color_max = Camera.blue_max;
                break;
            case YELLOW:
                this.color_min = Camera.yellow_min;
                this.color_max = Camera.yellow_max;
                break;
        }
    }
    public SampleLocationPipeline(Telemetry telemetry){
        this(Camera.Colors.BLUE, telemetry);
    }

    public Mat processFrame(Mat input) {
        empty_mat.copyTo(color_filtered_image);
        // Undistort
        //Calib3d.undistort(input, input_undistort, Camera.getCameraMatrix(), Camera.getDistortionCoefficients());
        input.copyTo(input_undistort);
        // Input RGB -> HSV
        Imgproc.cvtColor(input_undistort, input_undistort, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, input_hsv, Imgproc.COLOR_RGB2HSV);
        // Gets Colors From Image (Binary)
        //Imgproc.blur(input_hsv, color_mask, new Size(3,3));
        Core.inRange(input_undistort, color_min, color_max, color_mask);
        //   Erode and dilate
        Imgproc.erode(color_mask, color_mask, empty_mat, new Point(-1, -1), 6);
        Imgproc.dilate(color_mask, color_mask, empty_mat, new Point(-1, -1), 6);

        //   add color when mask is white, add white when mask is black
        Core.bitwise_and(input_undistort, input_undistort, color_filtered_image, color_mask);
        Core.bitwise_not(color_mask, color_mask);
        Imgproc.cvtColor(color_mask, color_mask, Imgproc.COLOR_GRAY2RGB);
        Imgproc.cvtColor(color_mask, color_mask, Imgproc.COLOR_RGB2HSV);
        Core.add(color_mask, color_filtered_image, color_filtered_image);
        // Get points from colors
        //     set blob detector params
        Imgproc.cvtColor(color_filtered_image,greyscale, Imgproc.COLOR_HSV2RGB);
        Imgproc.cvtColor(greyscale,greyscale, Imgproc.COLOR_RGB2GRAY);
        Core.bitwise_not(greyscale, greyscale);
        List<MatOfPoint> contours = new ArrayList<>();
        List<RotatedRect> rectangles = new ArrayList<>();
        Imgproc.findContours(greyscale, contours, hierarchy, Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_SIMPLE );
        Imgproc.drawContours(color_filtered_image, contours, -1, new Scalar(0,0,0), 1);
        List<Point> world_points = new ArrayList<>();
        int n = 0;
        for (MatOfPoint points:contours){
            n+= 1;
            double top_x = 0;
            double max_y = Double.NEGATIVE_INFINITY;
            double min_y = Double.POSITIVE_INFINITY;
            Point top_point = null;
            Point bottom_point = null;
            double center_x = 0;
            double size = 0;
            for(Point point: points.toList()){
                size += 1;
                center_x += point.x;
            };
            center_x /= size;
            for(Point point: points.toList()){
                // bias to be far on the x axis
                if (point.y + Math.abs(center_x -point.x)/8 > max_y){
                    max_y = point.y + Math.abs(center_x -point.x)/8;
                    top_x = point.x;
                    top_point = point;
                }
            };
            for(Point point: points.toList()){
                // bias to be far on the x axis
                if (point.y - Math.abs(point.x-top_x)/8 < min_y){
                    min_y = point.y - Math.abs(top_x-point.x)/8;
                    bottom_point = point;
                }
            };
            Imgproc.drawMarker(color_filtered_image, top_point, new Scalar(0,255,255));
            Imgproc.drawMarker(color_filtered_image, bottom_point, new Scalar(120+5*n,255,255));
            Point top_world_point = uvToWorld(top_point, true);
            Point bottom_world_point = uvToWorld(bottom_point, false);
            world_points.add(new Point((top_world_point.x + bottom_world_point.x)/2,(top_world_point.y + bottom_world_point.y)/2));
        double angle = Math.atan((top_world_point.y - bottom_world_point.y)/(bottom_world_point.x - top_world_point.x)) + 0.40489178629;
            Vector<Double> v = new Vector<>(3,0);
            v.add(0, (top_world_point.x + bottom_world_point.x)/2 * 1/25.4);
            v.add(1, (top_world_point.y + bottom_world_point.y)/2 * 1/25.4);
            // 0.40489178629 = difference in radians from
            v.add(angle);
            telemetry.addData("x, y, θ", String.format("%.1f, %.1f, %.2f", (top_world_point.x + bottom_world_point.x)/2 * 1/25.4, (top_world_point.y + bottom_world_point.y)/2 * 1/25.4, angle/2/Math.PI));
            object_field_coords.add(v);
        }
        switch(stage){
            case 4:
                color_filtered_image.copyTo(output);
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
    public Point uvToWorld(Point screen_point, boolean top){
        Mat screen_coords = new Mat(3, 1, 6);
        screen_coords.put(0,0, screen_point.x);
        screen_coords.put(1,0, screen_point.y);
        screen_coords.put(2,0, 1);
        // screen_coords = [u, v, 1]
        Mat result = top ? top_reprojection_matrix.matMul(screen_coords)
                         : reprojection_matrix.matMul(screen_coords);
        double scale = 1.0/result.get(2,0)[0];
        return new Point(result.get(0,0)[0] * scale, result.get(1,0)[0] * scale);
    }
    public List<Vector<Double>> getAnalysis(){
        return object_field_coords;
    };
}
