package org.firstinspires.ftc.teamcode.vision;
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
import java.util.Vector;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SampleLocationPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    public Scalar color_min;
    public Scalar color_max;
    public int stage = 4;
    Mat input_undistort = new Mat();
    Mat color_mask = new Mat();
    Mat color_filtered_image = new Mat();
    Mat output = new Mat();
    final Mat empty_mat = new Mat();
    Mat hierarchy = new Mat();
    Mat greyscale = new Mat();
    public List<double[]> object_field_coords = new ArrayList<>();
    final double top_distance_weight = 1/10;
    final double top2_distance_weight = 3;
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
        this(Camera.Colors.BLUE, telemetry);
    }

    public Mat processFrame(Mat input) {
    	object_field_coords.clear();
        empty_mat.copyTo(color_filtered_image);
        // Undistort
        //Calib3d.undistort(input, input_undistort, Camera.getCameraMatrix(), Camera.getDistortionCoefficients());
        input.copyTo(input_undistort);
        // Input RGB -> HSV
        Imgproc.cvtColor(input_undistort, input_undistort, Imgproc.COLOR_RGB2HSV);
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
        Imgproc.findContours(greyscale, contours, hierarchy, Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_NONE );
        
        // Get coords + rotation for each contour
        int n = 0;
        for (MatOfPoint points:contours){
            n += 1;
            MatOfPoint2f points2f = new MatOfPoint2f(points.toArray());
            Imgproc.approxPolyDP(points2f, points2f, 2, true);
            List<MatOfPoint > list_points2f = new ArrayList<>();
            list_points2f.add(new MatOfPoint(points2f.toArray()));
            Imgproc.drawContours(color_filtered_image, list_points2f, -1, new Scalar(60,255,255), 1);
            double top_x = 0;
            double max_top_score = Double.NEGATIVE_INFINITY;
            double max_top2_score = Double.NEGATIVE_INFINITY;
            Point top_point = null;
            Point top2_point = null;
            double center_x = 0;
            double center_y = 0;
            for(Point point: points2f.toList()){
                center_x += point.x;
                center_y += point.y;
            };
            center_x /= points2f.size().area() ;
            center_y /= points2f.size().area() ;
            for(Point point: points2f.toList()){
                // bias to be far on the x axis
                double score = -point.y + Math.abs(center_x - point.x) * top_distance_weight;
                if (score > max_top_score){
                    max_top_score = score;
                    top_x = point.x;
                    top_point = point;
                }
            };
            for(Point point: points2f.toList()){
                // bias to be far on the x axis
                double score = -point.y + Math.abs(top_x - point.x) * top2_distance_weight;
                if (score > max_top2_score){
                    max_top2_score = score;
                    top2_point = point;
                }
            };
            Point top_world_point = Camera.uvToWorld(top_point);
            Point top2_world_point = Camera.uvToWorld(top2_point);
            telemetry.addLine(String.format("w1 %.2f, %.2f", top_world_point.x, top_world_point.y));
            telemetry.addLine(String.format("c1 %.2f, %.2f", top_point.x, top_point.y));
            telemetry.addLine(String.format("w2 %.2f, %.2f", top2_world_point.x, top2_world_point.y));
            telemetry.addLine(String.format("c2 %.2f, %.2f", top2_point.x, top2_point.y));

            double distance = Math.sqrt(
                    Math.pow(top2_world_point.x - top_world_point.x,2)
                  + Math.pow(top2_world_point.y - top_world_point.y,2)
            );
            
            Imgproc.drawMarker(color_filtered_image, top_point, new Scalar(0,255,255));
            Imgproc.drawMarker(color_filtered_image, top2_point, new Scalar(120+5*n,255,255));
            if (Math.abs(distance - 1.5) > 0.3 && Math.abs(distance - 3.5) > 0.3) {
                telemetry.addData("out of range", distance);
                break;
            }
            boolean short_side = (distance < 2);
            double angle = Math.atan((top_world_point.y - top2_world_point.y)/(top2_world_point.x - top_world_point.x)) + (short_side ? Math.PI/2: 0);
            double multiplier =
                      (short_side ? 1.5: 3.5)
                  * (top_world_point.x < top2_world_point.x ? 1: -1)
                  * (center_y < (top_point.y-top2_point.y/top_point.x-top2_point.x) * (center_x-top_point.x) + top_point.y ? 1: -1);
            double world_x = (top_world_point.x + top2_world_point.x)/2 + (top_world_point.y - top2_world_point.y)/distance * multiplier;
            double world_y = (top_world_point.y + top2_world_point.y)/2 - (top_world_point.x - top2_world_point.x)/distance * multiplier;
            Imgproc.putText(color_filtered_image, (short_side? "short": "long"), new Point((top_point.x + top2_point.x)/2, (top_point.y + top2_point.y)/2), Imgproc.FONT_HERSHEY_PLAIN, 1, new Scalar(80,255,20), 1);
            Imgproc.putText(color_filtered_image, String.format("%.1f", distance), top_point, Imgproc.FONT_HERSHEY_PLAIN, 1, new Scalar(80,255,20), 1);
            double[] pose = {world_x, world_y, angle};
            object_field_coords.add(pose);
            telemetry.addData("x, y, θ", String.format("%.1f, %.1f, %.2f", world_x, world_y, angle));

        }
        switch(stage){
            case 4:
                color_filtered_image.copyTo(output);
                break;
            case 3:
                greyscale.copyTo(output);
                break;
            case 2:
                color_mask.copyTo(output);
                break;
            case 1:
                input_undistort.copyTo(output);
                break;
            default:
                input_undistort.copyTo(output);
                break;
        }
        // HSV -> RGB

        Imgproc.cvtColor(output, output, Imgproc.COLOR_HSV2RGB);
        telemetry.update();
        return output;
    }

    public List<double[]> getAnalysis(){
        return object_field_coords;
    };
}
