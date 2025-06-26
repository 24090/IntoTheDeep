package org.firstinspires.ftc.teamcode.vision;

import static java.lang.Math.PI;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import kotlin.Pair;
import kotlin.Triple;

public class SampleLocationPipeline extends OpenCvPipeline {
    final Camera.Colors[] allowed_colors;
    Telemetry telemetry;
    public int stage = 4;
    Mat input_undistort = new Mat();
    Mat color_mask_upper = new Mat();
    Mat color_mask = new Mat();
    Mat color_filtered_image = new Mat();
    Mat output = new Mat();
    final Mat empty_mat = new Mat();
    Mat hierarchy = new Mat();
    Mat greyscale = new Mat();
    public List<Triple<Double, Double, Double>> object_field_coords = new ArrayList<>();
    final double top_distance_weight  = 0.05;
    final double top2_distance_weight = 1;
    public SampleLocationPipeline(Telemetry telemetry) {
        Camera.init();
        this.allowed_colors = new Camera.Colors[]{Camera.Colors.RED, Camera.Colors.YELLOW, Camera.Colors.BLUE};
        this.telemetry = telemetry;
    }
    public SampleLocationPipeline(Camera.Colors[] allowed_colors, Telemetry telemetry) {
        Camera.init();
        this.allowed_colors = allowed_colors;
        this.telemetry = telemetry;
    }
    void initialProcessing(Mat input, Mat input_undistort){
        // Undistort
        Calib3d.undistort(input, input_undistort, Camera.camera_matrix, Camera.distortion_coefficients);
        // Input RGB -> HSV
        Imgproc.cvtColor(input_undistort, input_undistort, Imgproc.COLOR_RGB2HSV);
    }
    void filterColors(Camera.Colors color, Mat input_undistort, Mat color_mask){
        empty_mat.copyTo(color_mask);
        switch (color) {
            case RED:
                Core.inRange(input_undistort, new Scalar(0,0,0), Camera.ColorValues.red_max, color_mask);
                Core.inRange(input_undistort, Camera.ColorValues.red_min, new Scalar(255,255,255), color_mask_upper);
                Core.bitwise_or(color_mask, color_mask_upper, color_mask);
                break;
            case BLUE:
                Core.inRange(input_undistort, Camera.ColorValues.blue_min, Camera.ColorValues.blue_max, color_mask);
                break;
            case YELLOW:
                Core.inRange(input_undistort, Camera.ColorValues.yellow_min, Camera.ColorValues.yellow_max, color_mask);
                break;
        }
    }
    void getContours(Mat color_mask, Mat input_undistort, Mat color_filtered_image, List<MatOfPoint> dst){
        empty_mat.copyTo(color_filtered_image);
        //   Erode and dilate
        Imgproc.erode(color_mask, color_mask, empty_mat, new Point(-1, -1), 5);
        Imgproc.dilate(color_mask, color_mask, empty_mat, new Point(-1, -1), 10);
        Imgproc.erode(color_mask, color_mask, empty_mat, new Point(-1, -1), 5);
        //   add color when mask is white, add white when mask is black
        Core.bitwise_and(input_undistort, input_undistort, color_filtered_image, color_mask);
        Core.bitwise_not(color_mask, color_mask);
        Imgproc.cvtColor(color_mask, color_mask, Imgproc.COLOR_GRAY2RGB);
        Imgproc.cvtColor(color_mask, color_mask, Imgproc.COLOR_RGB2HSV);
        Core.add(color_mask, color_filtered_image, color_filtered_image);
        // Find contours
        Imgproc.cvtColor(color_filtered_image, greyscale, Imgproc.COLOR_HSV2RGB);
        Imgproc.cvtColor(greyscale, greyscale, Imgproc.COLOR_RGB2GRAY);
        Core.bitwise_not(greyscale, greyscale);
        Imgproc.findContours(greyscale, dst, hierarchy, Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_NONE );
    }

    Pair<Point, Point> getTopPoints(MatOfPoint2f points2f) {
        double top_x = 0;
        double max_primary_score = Double.NEGATIVE_INFINITY;
        double max_secondary_score = Double.NEGATIVE_INFINITY;
        Point primary_top_point = null;
        Point secondary_top_point = null;
        double center_x = 0;
        for(Point point: points2f.toList()){
            center_x += point.x;
        }
        center_x /= points2f.size().area();
        for(Point point: points2f.toList()){
            // bias to be far on the x axis
            double score = -point.y + Math.abs(center_x - point.x) * top_distance_weight;
            if (score > max_primary_score){
                max_primary_score = score;
                top_x = point.x;
                primary_top_point = point;
            }
        }
        for(Point point: points2f.toList()){
            // bias to be far on the x axis
            double score = -point.y + Math.abs(top_x - point.x) * top2_distance_weight;
            if (score > max_secondary_score){
                max_secondary_score = score;
                secondary_top_point = point;
            }
        }
        return new Pair<>(primary_top_point, secondary_top_point);
    }

    Triple<Double, Double, Double> calculateSamplePose(Point world_point_a, Point world_point_b){
        double distance = Math.sqrt(
                Math.pow(world_point_b.x - world_point_a.x,2)
                + Math.pow(world_point_b.y - world_point_a.y,2)
        );

        if (Math.abs(distance - 1.5) > 0.5 && Math.abs(distance - 3.5) > 0.75) {
            telemetry.addData("too long or short", distance);
            return null;
        }

        boolean short_side = (distance < 2);
        double angle = Math.atan((world_point_a.y - world_point_b.y)/(world_point_b.x - world_point_a.x)) + (short_side ? PI/2: 0);
        double multiplier = (short_side ? 3.5/2: 1.5/2) * (world_point_a.x < world_point_b.x ? -1: 1);
        double world_x = (world_point_a.x + world_point_b.x)/2 + (world_point_a.y - world_point_b.y)/distance * multiplier;
        double world_y = (world_point_a.y + world_point_b.y)/2 - (world_point_a.x - world_point_b.x)/distance * multiplier;
        telemetry.addData("detected_length", distance);
        return new Triple<>(world_x, world_y, angle);
    }

    @SuppressLint("DefaultLocale")
    public Mat processFrame(Mat input) {
        List<Triple<Double, Double, Double>> new_objects = new ArrayList<>();
        List<MatOfPoint> contours = new ArrayList<>();

        initialProcessing(input, input_undistort);

        for (Camera.Colors color: allowed_colors){
            List<MatOfPoint> new_contours = new ArrayList<>();
            filterColors(color, input_undistort, color_mask);
            getContours(color_mask, input_undistort, color_filtered_image, new_contours);
            contours.addAll(new_contours);
        }
        // Get coords + rotation for each contour
        int n = 0;
        for (MatOfPoint points: contours){
            n += 1;
            MatOfPoint2f points2f = new MatOfPoint2f(points.toArray());
            Imgproc.approxPolyDP(points2f, points2f, 2, true);
            List<MatOfPoint > list_points2f = new ArrayList<>();
            list_points2f.add(new MatOfPoint(points2f.toArray()));

            Imgproc.drawContours(input_undistort, list_points2f, -1, new Scalar(60,255,255), 1);

            Pair<Point, Point> top_points = getTopPoints(points2f);
            Point point_a = top_points.component1();
            Point point_b = top_points.component2();
            Point world_point_a = Camera.uvToWorld(point_a);
            Point world_point_b = Camera.uvToWorld(point_b);
            Imgproc.drawMarker(input_undistort, point_a, new Scalar(0,255,255));
            Imgproc.drawMarker(input_undistort, point_b, new Scalar(120+5*n,255,255));

            Triple<Double, Double, Double> pose = calculateSamplePose(world_point_a, world_point_b);

            if (pose == null) {
                continue;
            }
            Point screen_point = Camera.worldToUV(new Point(pose.component1(), pose.component2()));
            Imgproc.drawMarker(input_undistort, screen_point, new Scalar(120+5*n,255,255));
            new_objects.add(pose);
            telemetry.addData("x, y, θ", String.format("%.1f, %.1f, %.2f", pose.getFirst(), pose.getSecond(), pose.getThird()));
        }
        switch(stage){
            case 4:
                Imgproc.cvtColor(color_filtered_image, output, Imgproc.COLOR_HSV2RGB);
                break;
            case 3:
                Imgproc.cvtColor(greyscale, output, Imgproc.COLOR_GRAY2RGB);
                break;
            case 2:
                Imgproc.cvtColor(color_mask, output, Imgproc.COLOR_HSV2RGB);
                break;
            default:
                Imgproc.cvtColor(input_undistort, output, Imgproc.COLOR_HSV2RGB);
                break;
        }
        object_field_coords = new ArrayList<>(new_objects);
        telemetry.update();
        return output;
    }

    public List<Triple<Double, Double, Double>> getAnalysis(){
        return object_field_coords;
    }
}
