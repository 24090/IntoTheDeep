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
        Mat m2 = new Mat(3, 3, 6);
		Mat m3 = new Mat(1, 5, 6);

		// generated with ReprojectionMatrixCalculator.py
		m1.put(0, 0,  1.053050e-03); m1.put(0, 1,  1.690219e-20); m1.put(0, 2, -3.571787e-01);
		m1.put(1, 0,  0.000000e+00); m1.put(1, 1,  6.709920e-04); m1.put(1, 2, -9.126883e-01);
		m1.put(2, 0, -0.000000e+00); m1.put(2, 1, -6.025893e-05); m1.put(2, 2, -3.526842e-02);

		m2.put(0, 0,  9.496228e+02); m2.put(0, 1,  0.000000e+00); m2.put(0, 2,  3.391850e+02);
		m2.put(1, 0,  0.000000e+00); m2.put(1, 1,  9.579662e+02); m2.put(1, 2,  2.185479e+02);
		m2.put(2, 0,  0.000000e+00); m2.put(2, 1,  0.000000e+00); m2.put(2, 2,  1.000000e+00);

		m3.put(0, 0, 0.1947389099586247);
		m3.put(0, 1, -0.4378239199371739);
		m3.put(0, 2, -0.006919984229621114);
		m3.put(0, 3, 0.005882513339448832);
		m3.put(0, 4, -0.8125853302687356);

		m1.copyTo(reprojection_matrix);
		m2.copyTo(camera_matrix);
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
