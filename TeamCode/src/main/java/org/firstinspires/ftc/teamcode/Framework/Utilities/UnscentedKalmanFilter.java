package org.firstinspires.ftc.teamcode.Framework.Utilities;

import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.CholeskyDecomposition;
import org.apache.commons.math3.linear.EigenDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class UnscentedKalmanFilter {
	public interface NonlinearSystem {
		RealVector calculate(RealVector x, RealVector u);
	}
	public interface NonlinearSensor {
		RealVector measure(RealVector x, RealVector u);
		RealVector getData();
	}

	private NonlinearSystem f;
	private RealMatrix Q;
	private RealVector weights;
	private int n;

	private RealMatrix sigmaPoints; // definitely a reference to 22377
	private RealMatrix P;
	private RealVector xHat;

	public UnscentedKalmanFilter(NonlinearSystem f, RealMatrix Q,
								 double w0, RealVector x0, RealMatrix P0) {
		this.f = f;
		this.Q = Q;
		this.n = this.Q.getColumnDimension();
		double w = (1 - w0) / (2 * n);
		this.weights = new ArrayRealVector(2 * this.n + 1, w);
		this.weights.setEntry(0, w0);

		this.P = P0;
		this.xHat = x0;
		this.sigmaPoints = new Array2DRowRealMatrix(n, 2 * n + 1);

		calculateSigmaPoints();
	}

	private void calculateSigmaPoints() {
		RealMatrix squared = P.scalarMultiply(n / (1 - weights.getEntry(0)));
		EigenDecomposition decomposition = new EigenDecomposition(squared);
		RealMatrix squareRoot = decomposition.getSquareRoot();

		sigmaPoints.setColumnVector(0, xHat);
		for (int i = 1; i <= n; i++) {
			RealVector offset = squareRoot.getColumnVector(i);

			sigmaPoints.setColumnVector(i, xHat.add(offset));
			sigmaPoints.setColumnVector(i + n, xHat.subtract(offset));
		}
	}

	private void computeStateCovariance() {
		xHat = sigmaPoints.operate(weights);

		P = new Array2DRowRealMatrix(n, n);
		for (int i = 0; i < 2 * n + 1; i++) {
			RealVector variance = sigmaPoints.getColumnVector(i).subtract(xHat);
			P = P.add(variance.outerProduct(variance).scalarMultiply(weights.getEntry(i)));
		}

		P = P.add(Q);
	}

	private RealMatrix computeMeasurementCovariance(RealVector predictedMeasurement,
													RealMatrix measurementPoints, RealMatrix R) {
		RealMatrix S = new Array2DRowRealMatrix(R.getRowDimension(), R.getColumnDimension());
		for (int i = 0; i < 2 * n + 1; i++) {
			RealVector variance =
					measurementPoints.getColumnVector(i).subtract(predictedMeasurement);
			S = S.add(variance.outerProduct(variance).scalarMultiply(weights.getEntry(i)));
		}

		return S.add(R);
	}

	private RealMatrix computeCrossCovariance(RealVector predictedMeasurement,
													RealMatrix measurementPoints, RealMatrix R) {
		RealMatrix N = new Array2DRowRealMatrix(n, R.getColumnDimension());
		for (int i = 0; i < 2 * n + 1; i++) {
			RealVector stateVariance = sigmaPoints.getColumnVector(i).subtract(xHat);
			RealVector measurementVariance =
					measurementPoints.getColumnVector(i).subtract(predictedMeasurement);
			N = N.add(stateVariance.outerProduct(measurementVariance)
					.scalarMultiply(weights.getEntry(i)));
		}

		return N;
	}

	public void predict(RealVector u) {
		calculateSigmaPoints();

		for (int i = 0; i < 2 * n + 1; i++) {
			RealVector point = sigmaPoints.getColumnVector(i);
			point = f.calculate(point, u);
			sigmaPoints.setColumnVector(i, point);
		}

		computeStateCovariance();
	}

	public void correct(RealVector u, NonlinearSensor sensor, RealMatrix R) {
		RealMatrix measurementPoints = new Array2DRowRealMatrix(n, 2 * n + 1);

		for (int i = 0; i < 2 * n + 1; i++) {
			RealVector point = sigmaPoints.getColumnVector(i);
			point = sensor.measure(point, u);
			measurementPoints.setColumnVector(i, point);
		}

		RealVector predictedMeasurement = measurementPoints.operate(weights);
		RealMatrix measurementCovariance =
				computeMeasurementCovariance(predictedMeasurement, measurementPoints, R);
		RealMatrix crossCovariance =
				computeCrossCovariance(predictedMeasurement, measurementPoints, R);

		RealMatrix K = crossCovariance.multiply(MatrixUtils.inverse(measurementCovariance));

		xHat = xHat.add(K.operate(sensor.getData().subtract(predictedMeasurement)));
		P = P.subtract(K.multiply(measurementCovariance.multiply(K.transpose())));
	}
}
