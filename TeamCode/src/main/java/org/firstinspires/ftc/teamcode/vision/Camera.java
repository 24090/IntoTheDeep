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

	public static void init() {
		// Matrix Creation
		Mat rm = new Mat(3, 3, 6);
		Mat cm = new Mat(3, 3, 6);
		Mat dc = new Mat(1, 5, 6);

		// generated with ReprojectionMatrixCalculator.py
		rm.put(0, 0,  1.049424e-03); rm.put(0, 1,  2.504268e-21); rm.put(0, 2, -3.546058e-01);
		rm.put(1, 0,  0.000000e+00); rm.put(1, 1,  6.695704e-04); rm.put(1, 2, -8.765833e-01);
		rm.put(2, 0, -0.000000e+00); rm.put(2, 1, -6.013126e-05); rm.put(2, 2, -3.851086e-02);

		cm.put(0, 0,  9.529036e+02); cm.put(0, 1,  0.000000e+00); cm.put(0, 2,  3.379052e+02);
		cm.put(1, 0,  0.000000e+00); cm.put(1, 1,  9.600001e+02); cm.put(1, 2,  1.650892e+02);
		cm.put(2, 0,  0.000000e+00); cm.put(2, 1,  0.000000e+00); cm.put(2, 2,  1.000000e+00);

		dc.put(0, 0,  8.863863e-04);
		dc.put(0, 1,  1.895324e+00);
		dc.put(0, 2, -2.824227e-02);
		dc.put(0, 3, -7.429107e-03);
		dc.put(0, 4, -1.116401e+01);
		// end

		rm.copyTo(reprojection_matrix);
		cm.copyTo(camera_matrix);
		dc.copyTo(distortion_coefficients);
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
