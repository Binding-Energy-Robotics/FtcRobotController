package org.firstinspires.ftc.teamcode.Roadrunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.checkerframework.checker.units.qual.K;
import org.firstinspires.ftc.teamcode.Roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.gyrationConstant;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->
 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
	public static double TICKS_PER_REV = 8192;
	public static double WHEEL_RADIUS = 1; // in
	public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

	public static double PARALLEL_X = 0; // X is the up and down direction
	public static double PARALLEL_Y = -3.97; // Y is the strafe direction

	public static double PERPENDICULAR_X = 0;
	public static double PERPENDICULAR_Y = 0;

	public static double X_MULTIPLIER = 0.6866;
	public static double Y_MULTIPLIER = 0.7027;

	// Parallel/Perpendicular to the forward axis
	// Parallel wheel is parallel to the forward axis
	// Perpendicular is perpendicular to the forward axis
	private Encoder parallelEncoder, perpendicularEncoder;

	private SampleMecanumDrive drive;

	private KalmanFilter kalmanFilter;
	private ProcessModel processModel;
	private MeasurementModel measurementModel;

	private int headingWraps = 0;
	private double previousHeading = 0;

	public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
		super(Arrays.asList(
				new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
				new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
		));

		this.drive = drive;

		parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
		perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));

		double kV_r = kV / TRACK_WIDTH;
		double kA_r = kA / (TRACK_WIDTH * gyrationConstant);
		RealMatrix Ac = new Array2DRowRealMatrix(new double[][] {
				{ 0, 0, 0,   1, 0, 0,   0, 0, 0 },
				{ 0, 0, 0,   0, 1, 0,   0, 0, 0 },
				{ 0, 0, 0,   0, 0, 1,   0, 0, 0 },

				{ 0, 0, 0,     -kV / kA, 0,        0,                1 / kA, 0,      0 },
				{ 0, 0, 0,     0,        -kV / kA, 0,                0,      1 / kA, 0 },
				{ 0, 0, 0,     0,        0,        -kV_r / kA_r,     0,      0,      1 / kA_r },

				{ 0, 0, 0,   0, 0, 0,   0, 0, 0 },
				{ 0, 0, 0,   0, 0, 0,   0, 0, 0 },
				{ 0, 0, 0,   0, 0, 0,   0, 0, 0 }
		});
		RealMatrix Bc = new Array2DRowRealMatrix(new double[][] {
				{ 0, 0, 0 },
				{ 0, 0, 0 },
				{ 0, 0, 0 },

				{ 1 / kA,  0,      0 },
				{ 0,       1 / kA, 0 },
				{ 0,       0,      1 / kA_r },

				{ 0, 0, 0 },
				{ 0, 0, 0 },
				{ 0, 0, 0 }
		});
		RealMatrix Q = new Array2DRowRealMatrix(new double[][] {
				{ .05, 0,   0,      0, 0, 0,   0, 0, 0 },
				{ 0,   .05, 0,      0, 0, 0,   0, 0, 0 },
				{ 0,   0,   .005,   0, 0, 0,   0, 0, 0 },

				{ 0, 0, 0,   .5, 0,  0,     0, 0, 0 },
				{ 0, 0, 0,   0,  .5, 0,     0, 0, 0 },
				{ 0, 0, 0,   0,  0,  .05,   0, 0, 0 },

				{ 0, 0, 0,   0, 0, 0,   .03, 0,   0 },
				{ 0, 0, 0,   0, 0, 0,   0,   .03, 0 },
				{ 0, 0, 0,   0, 0, 0,   0,   0,   .03 }
		});

		RealMatrix C = new Array2DRowRealMatrix(new double[][] {
				{ 1, 0, 0,   0, 0, 0,   0, 0, 0 },
				{ 0, 1, 0,   0, 0, 0,   0, 0, 0 },
				{ 0, 0, 1,   0, 0, 0,   0, 0, 0 },

				{ 0, 0, 0,   1, 0, 0,   0, 0, 0 },
				{ 0, 0, 0,   0, 1, 0,   0, 0, 0 },
				{ 0, 0, 0,   0, 0, 1,   0, 0, 0 }
		});
		RealMatrix R = new Array2DRowRealMatrix(new double[][] {
				{ 0.001, 0,     0,        0, 0, 0 },
				{ 0,     0.001, 0,        0, 0, 0 },
				{ 0,     0,     0.0003,   0, 0, 0 },

				{ 0, 0, 0,   0.01, 0,    0 },
				{ 0, 0, 0,   0,    0.01, 0 },
				{ 0, 0, 0,   0,    0,    0.0003 }
		});

		RealVector startPose = new ArrayRealVector(new double[] {
				0, 0, 0,   0, 0, 0,   0, 0, 0
		});
		RealMatrix startCovariance = new Array2DRowRealMatrix(new double[][] {
				{ 0, 0, 0,   0, 0, 0,   0, 0, 0 },
				{ 0, 0, 0,   0, 0, 0,   0, 0, 0 },
				{ 0, 0, 0,   0, 0, 0,   0, 0, 0 },

				{ 0, 0, 0,   0, 0, 0,   0, 0, 0 },
				{ 0, 0, 0,   0, 0, 0,   0, 0, 0 },
				{ 0, 0, 0,   0, 0, 0,   0, 0, 0 },

				{ 0, 0, 0,   0, 0, 0,   0, 0, 0 },
				{ 0, 0, 0,   0, 0, 0,   0, 0, 0 },
				{ 0, 0, 0,   0, 0, 0,   0, 0, 0 }
		});

		double dt = 0.01;

		RealMatrix taylor = taylorSeries(Ac, dt, 10);
		RealMatrix Bd = taylor.multiply(Bc);
		RealMatrix Ad = Ac.multiply(taylor).add(MatrixUtils.createRealIdentityMatrix(9));

		processModel = new DefaultProcessModel(
				Ad, Bd, Q, startPose, startCovariance
		);
		measurementModel = new DefaultMeasurementModel(
				C, R
		);
		kalmanFilter = new KalmanFilter(processModel, measurementModel);
	}

	private String printMat(RealMatrix mat) {
		StringBuilder toPrint = new StringBuilder("\n{\n");
		for (int i = 0; i < mat.getRowDimension(); i++) {
			for (int j = 0; j < mat.getColumnDimension(); j++) {
				if (j == 0) {
					toPrint.append("\t{").append(mat.getEntry(i, j));
				}
				else {
					toPrint.append(", ").append(mat.getEntry(i, j));
				}
			}
			toPrint.append("}\n");
		}
		toPrint.append("}");
		return toPrint.toString();
	}

	private RealMatrix taylorSeries(RealMatrix A, double dt, int terms) {
		RealMatrix exponentialA = MatrixUtils.createRealIdentityMatrix(9);
		RealMatrix acc = MatrixUtils.createRealIdentityMatrix(9).scalarMultiply(dt);
		double k = 1;
		double T = dt;
		for (int i = 2; i <= terms; i++) {
			exponentialA = exponentialA.multiply(A);
			T *= dt;
			k /= i;
			RealMatrix term = exponentialA.scalarMultiply(T * k);
			acc = acc.add(term);
		}
		return acc;
	}

	public static double encoderTicksToInches(double ticks) {
		return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
	}

	@Override
	public double getHeading() {
		return drive.getRawExternalHeading();
	}

	@Override
	public Double getHeadingVelocity() {
		return drive.getExternalHeadingVelocity();
	}

	@NonNull
	@Override
	public List<Double> getWheelPositions() {
		return Arrays.asList(
				encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
				encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
		);
	}

	@NonNull
	@Override
	public List<Double> getWheelVelocities() {
		// TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
		//  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
		//  compensation method

		return Arrays.asList(
				encoderTicksToInches(parallelEncoder.getCorrectedVelocity()),
				encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity())
		);
	}

	public double[] measurementVector(Pose2d position, Pose2d velocity) {
		if (velocity == null) velocity = new Pose2d();

		double currentHeading = position.getHeading();
		if (previousHeading > Math.PI / 2 && currentHeading <= -Math.PI / 2) {
			headingWraps += 1;
		}
		else if (previousHeading < -Math.PI / 2 && currentHeading >= Math.PI / 2) {
			headingWraps -= 1;
		}
		previousHeading = currentHeading;
		currentHeading += Math.PI * 2 * headingWraps;

		double[] vector = new double[6];
		vector[0] = position.getX();
		vector[1] = position.getY();
		vector[2] = currentHeading;
		vector[3] = velocity.getX();
		vector[4] = velocity.getY();
		vector[5] = velocity.getHeading();
		return vector;
	}

	@Override
	public void setPoseEstimate(@NonNull Pose2d value) {
		super.setPoseEstimate(value);
		headingWraps = 0;
		previousHeading = getPoseEstimate().getHeading();
		double[] partialState = measurementVector(getPoseEstimate(), null);
		double[] state = new double[9];
		for (int i = 0; i < 9; i++) {
			if (i < partialState.length) {
				state[i] = partialState[i];
				continue;
			}
			state[i] = 0;
		}
		processModel = new DefaultProcessModel(
				processModel.getStateTransitionMatrix(),
				processModel.getControlMatrix(),
				processModel.getProcessNoise(),
				new ArrayRealVector(state),
				kalmanFilter.getErrorCovarianceMatrix()
		);
		kalmanFilter = new KalmanFilter(processModel, measurementModel);
	}

	public void update(double[] previousPowers) { // implement extended kalman filter with adrc here
		super.update();
		kalmanFilter.predict(previousPowers);
		double[] measurement = measurementVector(getPoseEstimate(), getPoseVelocity());
		kalmanFilter.correct(measurement);
	}

	public Pose2d getDisturbanceRejectionPower() {
		double[] state = kalmanFilter.getStateEstimation();
		return new Pose2d(
				state[6],
				state[7],
				state[8]
		);
	}
}