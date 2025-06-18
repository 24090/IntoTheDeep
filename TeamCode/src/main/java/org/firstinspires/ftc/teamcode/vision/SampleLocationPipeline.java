package org.firstinspires.ftc.teamcode.vision;

import static java.lang.Math.PI;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
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
    Telemetry telemetry;
    public Scalar color_min;
    public Scalar color_max;
    public int stage = 5;
    Mat input_undistort = new Mat();
    Mat color_mask = new Mat();
    Mat color_filtered_image = new Mat();
    Mat output = new Mat();
    final Mat empty_mat = new Mat();
    Mat hierarchy = new Mat();
    Mat lines = new Mat();
    Mat greyscale = new Mat();
    Mat horizontal_edges = new Mat();
    Mat vertical_edges = new Mat();
    public ArrayList<Triple<Double, Double, Double>> object_field_coords = new ArrayList<>();
    final double top_distance_weight  = 0.05;
    final double top2_distance_weight = 2.4;

    public SampleLocationPipeline(Camera.Colors color, Telemetry telemetry) {
        Camera.init();
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
        this(Camera.Colors.YELLOW, telemetry);
    }

    private void getContours(Mat input, List<MatOfPoint> dst){
        empty_mat.copyTo(color_filtered_image);
        // Undistort
        Calib3d.undistort(input, input_undistort, Camera.camera_matrix, Camera.distortion_coefficients);
        // Input RGB -> HSV
        Imgproc.cvtColor(input_undistort, input_undistort, Imgproc.COLOR_RGB2HSV);
        // Gets Colors From Image (Binary)
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
        Imgproc.Sobel(greyscale, horizontal_edges, CvType.CV_8U, 0, 1);
        Core.subtract(greyscale, horizontal_edges, greyscale);
        Imgproc.findContours(greyscale, dst, hierarchy, Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_NONE );
    }

    private Pair<Point, Point> getTopPoints(MatOfPoint2f points2f) {
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

    private Triple<Double, Double, Double> calculateSamplePose(Point world_point_a, Point world_point_b){
        double distance = Math.sqrt(
                Math.pow(world_point_b.x - world_point_a.x,2)
                        + Math.pow(world_point_b.y - world_point_a.y,2)
        );

        if (Math.abs(distance - 1.5) > 0.5 && Math.abs(distance - 3.5) > 0.5) {
            telemetry.addData("out of range", distance);
            return null;
        }
        boolean short_side = (distance < 2);
        double angle = Math.atan((world_point_a.y - world_point_b.y)/(world_point_b.x - world_point_a.x)) + (short_side ? PI/2: 0);
        double multiplier = (short_side ? 1.5/2: 3.5/2) * (world_point_a.x < world_point_b.x ? 1: -1);
        double world_x = (world_point_a.x + world_point_b.x)/2 + (world_point_a.y - world_point_b.y)/distance * multiplier;
        double world_y = (world_point_a.y + world_point_b.y)/2 - (world_point_a.x - world_point_b.x)/distance * multiplier;

        return new Triple<>(world_x, world_y, angle);
    }

    @SuppressLint("DefaultLocale")
    public Mat processFrame(Mat input) {
    	object_field_coords.clear();
        List<MatOfPoint> contours = new ArrayList<>();
        getContours(input, contours);

        // Get coords + rotation for each contour
        int n = 0;
        for (MatOfPoint points: contours){
            n += 1;
            MatOfPoint2f points2f = new MatOfPoint2f(points.toArray());
            Imgproc.approxPolyDP(points2f, points2f, 2, true);
            List<MatOfPoint > list_points2f = new ArrayList<>();
            list_points2f.add(new MatOfPoint(points2f.toArray()));

            Imgproc.drawContours(color_filtered_image, list_points2f, -1, new Scalar(60,255,255), 1);

            Pair<Point, Point> top_points = getTopPoints(points2f);
            Point point_a = top_points.component1();
            Point point_b = top_points.component2();
            Point world_point_a = Camera.uvToWorld(point_a);
            Point world_point_b = Camera.uvToWorld(point_b);

            telemetry.addLine(String.format("w1 %.2f, %.2f", world_point_a.x, world_point_a.y));
            telemetry.addLine(String.format("c1 %.2f, %.2f", point_a.x, point_a.y));
            telemetry.addLine(String.format("w2 %.2f, %.2f", world_point_b.x, world_point_b.y));
            telemetry.addLine(String.format("c2 %.2f, %.2f", point_b.x, point_b.y));

            Imgproc.drawMarker(color_filtered_image, point_a, new Scalar(0,255,255));
            Imgproc.drawMarker(color_filtered_image, point_b, new Scalar(120+5*n,255,255));

            Triple<Double, Double, Double> pose = calculateSamplePose(world_point_a, world_point_b);
            if (pose == null) {
                continue;
            }
            object_field_coords.add(pose);
            telemetry.addData("x, y, θ", String.format("%.1f, %.1f, %.2f", pose.getFirst(), pose.getSecond(), pose.getThird()));
        }
        switch(stage){
            case 5:
                Imgproc.cvtColor(color_filtered_image, output, Imgproc.COLOR_HSV2RGB);
                break;
            case 4:
                Imgproc.cvtColor(greyscale, output, Imgproc.COLOR_GRAY2RGB);
                break;
            case 3:
                Imgproc.cvtColor(horizontal_edges, output, Imgproc.COLOR_GRAY2RGB);
                break;
            case 2:
                Imgproc.cvtColor(color_mask, output, Imgproc.COLOR_HSV2RGB);
                break;
            default:
                Imgproc.cvtColor(input_undistort, output, Imgproc.COLOR_HSV2RGB);
                break;
        }
        telemetry.update();
        return output;
    }

    public ArrayList<Triple<Double, Double, Double>> getAnalysis(){
        return object_field_coords;
    }
}
