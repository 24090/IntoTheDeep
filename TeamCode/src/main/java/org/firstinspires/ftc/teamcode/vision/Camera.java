package org.firstinspires.ftc.teamcode.vision;


import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

public class Camera {
	@Config
	static class ColorValues {
		public static Scalar red_min = new Scalar(0, 150, 80);
		public static Scalar red_max = new Scalar(255, 255, 255);
		public static Scalar blue_min =  new Scalar(0, 0, 180);
		public static Scalar blue_max = new Scalar(255, 255, 255);
		public static Scalar yellow_min =  new Scalar(0, 0, 0);
		public static Scalar yellow_max = new Scalar(255, 200, 95);
	}
	public static Mat projection_matrix = new Mat();
	public static Mat reprojection_matrix = new Mat();
	public static Mat camera_matrix = new Mat();
	public static Mat distortion_coefficients = new Mat();

	public static void init() {
		// Matrix Creation
		Mat pm = new Mat(3, 3, 6);
		Mat rm = new Mat(3, 3, 6);
		Mat cm = new Mat(3, 3, 6);
		Mat dc = new Mat(1, 5, 6);

		// generated with ReprojectionMatrixCalculator.py
		pm.put(0, 0,  9.600476e+02); pm.put(0, 1, -2.190011e+02); pm.put(0, 2,  8.856987e+02);
		pm.put(1, 0,  0.000000e+00); pm.put(1, 1,  4.156337e+02); pm.put(1, 2, -1.474207e+04);
		pm.put(2, 0,  0.000000e+00); pm.put(2, 1, -7.660444e-01); pm.put(2, 2, -5.801016e+00);

		rm.put(0, 0,  1.041615e-03); rm.put(0, 1,  1.481313e-04); rm.put(0, 2, -2.174110e-01);
		rm.put(1, 0,  0.000000e+00); rm.put(1, 1,  4.233028e-04); rm.put(1, 2, -1.075735e+00);
		rm.put(2, 0, -0.000000e+00); rm.put(2, 1, -5.589862e-05); rm.put(2, 2, -3.032899e-02);

		cm.put(0, 0,  9.600476e+02); cm.put(0, 1,  0.000000e+00); cm.put(0, 2,  2.858857e+02);
		cm.put(1, 0,  0.000000e+00); cm.put(1, 1,  9.556607e+02); cm.put(1, 2,  2.593233e+02);
		cm.put(2, 0,  0.000000e+00); cm.put(2, 1,  0.000000e+00); cm.put(2, 2,  1.000000e+00);

		dc.put(0, 0,  4.841244e-02);
		dc.put(0, 1,  1.121194e+00);
		dc.put(0, 2,  7.827670e-03);
		dc.put(0, 3, -4.267660e-04);
		dc.put(0, 4, -8.512152e+00);
		// end

		rm.copyTo(reprojection_matrix);
		cm.copyTo(camera_matrix);
		dc.copyTo(distortion_coefficients);
		pm.copyTo(projection_matrix);
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

	public static Point worldToUV(Point world_point) {
		Mat screen_coords = new Mat(3, 1, 6);
		screen_coords.put(0, 0, world_point.x);
		screen_coords.put(1, 0, world_point.y);
		screen_coords.put(2, 0, 1);
		// world_coords = [u, v, 1]
		Mat result = projection_matrix.matMul(screen_coords);
		double scale = 1.0 / result.get(2, 0)[0];
		return new Point(
				result.get(0, 0)[0] * scale,
				result.get(1, 0)[0] * scale
		);
	}
}
