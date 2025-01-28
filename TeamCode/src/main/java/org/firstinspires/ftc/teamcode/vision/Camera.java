package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

public class Camera {
    public static Scalar red_min;
    public static Scalar red_max ;
    public static Scalar blue_min = new Scalar(105,105,0);
    public static Scalar blue_max = new Scalar(125,255, 225);
    public static Scalar yellow_min = new Scalar(10, 145, 0);
    public static Scalar yellow_max = new Scalar(30, 255, 255);
    public enum Colors {
        RED,
        BLUE,
        YELLOW,
    }
    public static Mat getRotationMatrix() {
        //  [  1.0000000,  0.0000000,  0.0000000;
        //   0.0000000, 0.7071068, -0.7071068;
        //   0.0000000,  0.7071068, 0.7071068 ]
        // 60º X rotation
        Mat mat = new Mat(3,3,6);
        mat.put(0,0,1); mat.put(0,1,0); mat.put(0,2,0);
        mat.put(1,0,0); mat.put(1,1,0.7071068); mat.put(1,2,-0.7071068);
        mat.put(2,0,0); mat.put(2,1,0.7071068); mat.put(2,2, 0.7071068);
        return mat;
    }
    public static Mat getTranslationVector(boolean top) {
        // measurements in millimeters
        Mat mat = new Mat(3,1,6);
        mat.put(0,0,0);
        mat.put(1,0,0);
        if (!top) {mat.put(2,0,300);}
        else {mat.put(2,0,300 - 15);}
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
    public static Mat getExtrinsicsMatrix(boolean top){
        Mat extrinsics_matrix = new Mat();
        List<Mat> mats = new ArrayList<>();
            mats.add(getRotationMatrix());
            mats.add(getTranslationVector(top));
        Core.hconcat(mats, extrinsics_matrix);
        Mat bottom_row = new Mat(1, 4, 6);
            bottom_row.put(0, 0, 0);
            bottom_row.put(0, 1, 0);
            bottom_row.put(0, 2, 0);
            bottom_row.put(0, 3, 1);
        mats = new ArrayList<>();
            mats.add(extrinsics_matrix);
            mats.add(bottom_row);
        Core.vconcat(mats, extrinsics_matrix);
        extrinsics_matrix = extrinsics_matrix.inv();
        return extrinsics_matrix.rowRange(0, 3);
    }
    public static Mat getReprojectionMatrix(boolean top){
        Mat extrinsics_matrix = getExtrinsicsMatrix(top);
        List<Mat> mats = new ArrayList<>();
        mats.add(extrinsics_matrix.col(0));
        mats.add(extrinsics_matrix.col(1));
        mats.add(extrinsics_matrix.col(3));
        Core.hconcat(mats, extrinsics_matrix);
        return (Camera.getCameraMatrix().matMul(extrinsics_matrix)).inv();
    }
}