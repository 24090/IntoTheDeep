package org.firstinspires.ftc.teamcode.vision;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
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
    //     Calib3d.undistort(input, input_undistort, Camera.getCameraMatrix(), Camera.getDistortionCoefficients());
        input.copyTo(input_undistort);
        // Input RGB -> HSV
        Imgproc.cvtColor(input_undistort, input_undistort, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input_undistort, input_hsv, Imgproc.COLOR_RGB2HSV);
        // Gets Colors From Image (Binary)
        //Imgproc.blur(input_hsv, color_mask, new Size(3,3));
        Core.inRange(input_undistort, color_min, color_max, color_mask);
        //   Erode and dilate
        Imgproc.erode(color_mask, color_mask, empty_mat, new Point(-1, -1), 3);
        Imgproc.dilate(color_mask, color_mask, empty_mat, new Point(-1, -1), 3);
        //   add color when mask is white, add white when mask is black
        Core.bitwise_and(input_undistort, input_undistort, color_filtered_image, color_mask);
        Core.bitwise_not(color_mask, color_mask);
        Imgproc.cvtColor(color_mask, color_mask, Imgproc.COLOR_GRAY2RGB);
        Imgproc.cvtColor(color_mask, color_mask, Imgproc.COLOR_RGB2HSV);
        Core.add(color_mask, color_filtered_image, color_filtered_image);
        // Find contours
        Imgproc.cvtColor(color_filtered_image,greyscale, Imgproc.COLOR_HSV2RGB);
        Imgproc.cvtColor(greyscale,greyscale, Imgproc.COLOR_RGB2GRAY);
        Core.bitwise_not(greyscale, greyscale);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(greyscale, contours, hierarchy, Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_TC89_L1 );
        List<Point> world_points = new ArrayList<>();
        // Split contours using the watershed algorithm
        // Get coords + rotation for each contour
        int n = 0;
        for (MatOfPoint points:contours){
            n += 1;
            MatOfPoint2f points2f = new MatOfPoint2f(points.toArray());
            Imgproc.approxPolyDP(points2f, points2f, 3, true);
            List<MatOfPoint > list_points2f = new ArrayList<>();
            list_points2f.add(new MatOfPoint(points2f.toArray()));
            Imgproc.drawContours(color_filtered_image, list_points2f, -1, new Scalar(60,255,255), 1);
            double top_x = 0;
            double max_top_score = Double.NEGATIVE_INFINITY;
            double max_top2_score = Double.NEGATIVE_INFINITY;
            Point top_point = null;
            Point top2_point = null;
            double center_x = 0;
            for(Point point: points2f.toList()){
                center_x += point.x;
            };
            center_x /= points2f.size().area() ;
            for(Point point: points2f.toList()){
                // bias to be far on the x axis
                if (-point.y + Math.abs(center_x -point.x)/24 > max_top_score){
                    max_top_score = -point.y + Math.abs(center_x -point.x)/24;
                    top_x = point.x;
                    top_point = point;
                }
            };
            for(Point point: points2f.toList()){
                // bias to be far on the x axis
                if (-point.y/1.5 + Math.abs(top_x-point.x) > max_top2_score){
                    max_top2_score = -point.y/1.5 + Math.abs(top_x-point.x);
                    top2_point = point;
                }
            };
            Point top_world_point = Camera.uvToWorld(top_point, true);
            Point top2_world_point = Camera.uvToWorld(top2_point, true);
            telemetry.addLine(String.format("%.0f, %.0f", top_world_point.x, top_world_point.y));
            telemetry.addLine(String.format("%.0f, %.0f", top2_world_point.x, top2_world_point.y));

            double distance = Math.sqrt(Math.pow(top2_world_point.x - top_world_point.x,2) + Math.pow(top2_world_point.y - top_world_point.y,2));
            if (distance > 120) {
                telemetry.addData("too long", distance);
                break;
            }
            Imgproc.drawMarker(color_filtered_image, top_point, new Scalar(0,255,255));
            Imgproc.drawMarker(color_filtered_image, top2_point, new Scalar(120+5*n,255,255));
            boolean short_side = (distance - (2 * 25.4)) < 0;
            double angle = Math.atan((top_world_point.y - top2_world_point.y)/(top2_world_point.x - top_world_point.x)) + (short_side ? Math.PI/2: 0);
            double multiplier = (short_side ? 1.5*25.4: 3.5* 25.4);
            double world_x = (top_world_point.x + top2_world_point.x)/2 + Math.abs((top_world_point.y - top2_world_point.y)/distance) * multiplier;
            double world_y = (top_world_point.y + top2_world_point.y)/2 - Math.abs((top_world_point.x - top2_world_point.x)/distance) * multiplier;
            Imgproc.putText(color_filtered_image, (short_side? "short": "long"), new Point((top_point.x + top2_point.x)/2, (top_point.y + top2_point.y)/2), Imgproc.FONT_HERSHEY_PLAIN, 0.6, new Scalar(80,255,20), 1);
            Imgproc.putText(color_filtered_image,String.format("%.1f", distance), top_point, Imgproc.FONT_HERSHEY_PLAIN, 0.6, new Scalar(80,255,20), 1);

            world_points.add(new Point(world_x, world_y));
            Vector<Double> v = new Vector<>(3,0);
                v.add(0, (world_x) * 1/25.4);
                v.add(1, (world_y) * 1/25.4);
                v.add(angle);
            object_field_coords.add(v);
            telemetry.addData("x, y, θ", String.format("%.1f, %.1f, %.2f", world_x, world_y, angle));

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

    public List<Vector<Double>> getAnalysis(){
        return object_field_coords;
    };
}
