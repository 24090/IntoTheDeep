package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Scalar;

import kotlin.Pair;

public class CameraInfo {
    public static Scalar red_min;
    public static Scalar red_max ;
    public static Scalar blue_min = new Scalar(105,105,0);
    public static Scalar blue_max = new Scalar(125,255, 225);
    public static Scalar yellow_min;
    public static Scalar yellow_max;
    public enum Colors {
        RED,
        BLUE,
        YELLOW,
    }
    public static Mat getRotationMatrix() {
        Mat mat = new Mat(3,3,6);
        mat.put(0,0,1); mat.put(0,1,0); mat.put(0,2,0);
        mat.put(1,0,0); mat.put(1,1,1); mat.put(1,2,0);
        mat.put(2,0,0); mat.put(2,1,0); mat.put(2,2,1);
        return mat;
    }
    public static Mat getTranslationMatrix() {
        Mat mat = new Mat(3,1,6);
        mat.put(0,0,0);
        mat.put(1,0,10);
        mat.put(2,0,0);
        return mat;
    }
    public static Mat getCameraMatrix() {
        double focal_length_x = 822.317;
        double focal_length_y = 822.317;
        double principal_point_x = 319.495;
        double principal_point_y = 242.502;
        Mat mat = new Mat(3,3,6);
        mat.put(0,0,focal_length_x); mat.put(0,1,0);       mat.put(0,2,principal_point_x);
        mat.put(1,0,0);       mat.put(1,1,focal_length_y); mat.put(1,2,principal_point_y);
        mat.put(2,0,0);       mat.put(2,1,0);       mat.put(2,2,1);
        return mat;
    }
    public static Mat getDistortionCoefficients() {
        Mat mat = new Mat(1, 8, 6);
        mat.put(0, 0, -0.0449369);
        mat.put(0, 1, 1.17277);
        mat.put(0, 2, 0);
        mat.put(0, 3, 0);
        mat.put(0, 4, -3.63244);
        mat.put(0, 5, 0);
        mat.put(0, 6, 0);
        mat.put(0, 7, 0);
        return mat;
    }

}