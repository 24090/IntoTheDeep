package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

public class Camera {

	public static Scalar red_min;
	public static Scalar red_max;
	public static Scalar blue_min = new Scalar(105, 85, 0);
	public static Scalar blue_max = new Scalar(125, 255, 255);
	public static Scalar yellow_min = new Scalar(10, 115, 0);
	public static Scalar yellow_max = new Scalar(40, 255, 255);
    public static Mat reprojection_matrix = new Mat();
    public static Mat camera_matrix = new Mat();
    public static Mat distortion_coefficients = new Mat();

    public static void init(){
        // Matrix Creation
        Mat m1 = new Mat(3, 3, 6 );
		
		// generated with ReprojectionMatrixCalculator.py
		m1.put(0, 0,  1.216076e-03); m1.put(0, 1,  6.948339e-20); m1.put(0, 2, -3.885302e-01);
		m1.put(1, 0,  0.000000e+00); m1.put(1, 1,  7.088603e-04); m1.put(1, 2, -9.844383e-01);
		m1.put(2, 0, -0.000000e+00); m1.put(2, 1, -8.592246e-05); m1.put(2, 2, -2.985127e-02);
        m1.copyTo(reprojection_matrix);

        double focal_length_x = 822.317;
        double focal_length_y = 822.317;
        double principal_point_x = 319.495;
        double principal_point_y = 242.502;
        Mat m2 = new Mat(3, 3, 6);
        m2.put(0, 0, focal_length_x); m2.put(0, 1, 0);     m2.put(0, 2, principal_point_x);
        m2.put(1, 0, 0);     m2.put(1, 1, focal_length_y); m2.put(1, 2, principal_point_y);
        m2.put(2, 0, 0);     m2.put(2, 1, 0);     m2.put(2, 2, 1);
        m2.copyTo(camera_matrix);

        Mat m3 = new Mat(1, 8, 6);
        m3.put(0, 0, -0.0449369);
        m3.put(0, 1, 1.17277);
        m3.put(0, 2, 0);
        m3.put(0, 3, 0);
        m3.put(0, 4, -3.63244);
        m3.put(0, 5, 0);
        m3.put(0, 6, 0);
        m3.put(0, 7, 0);
        m3.copyTo(distortion_coefficients);

    }

	public enum Colors {
		RED,
		BLUE,
		YELLOW,
	}
    
	public static Point uvToWorld(Point screen_point) {
		Mat screen_coords = new Mat(3, 1, 6);
		screen_coords.put(0, 0, screen_point.x);
		screen_coords.put(1, 0, screen_point.y);
		screen_coords.put(2, 0, 1);
		// screen_coords = [u, v, 1]
		Mat result = reprojection_matrix.matMul(screen_coords);
		double scale = 1.0 / result.get(2, 0)[0];
		return new Point(
			result.get(0, 0)[0] * scale,
			result.get(1, 0)[0] * scale
		);
	}
}
