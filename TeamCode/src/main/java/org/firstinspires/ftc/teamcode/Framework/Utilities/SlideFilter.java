package org.firstinspires.ftc.teamcode.Framework.Utilities;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class SlideFilter {
	private RealMatrix A;
	private RealMatrix B;
	private RealMatrix C;

	private RealMatrix Q;
	private RealMatrix R;

	private RealVector x;
	private RealMatrix P;

	public SlideFilter(RealMatrix A, RealMatrix B, RealMatrix Q, RealMatrix R) {
		this.A = A;
		this.B = B;
		C = new Array2DRowRealMatrix(new double[][] {
				{ 1, 0 },
				{ 0, 1 }
		});

		this.Q = Q;
		this.R = R;

		x = new ArrayRealVector(new double[] {
				0, 0
		});
		P = new Array2DRowRealMatrix(new double[][] {
				{ 0, 0 },
				{ 0, 0 }
		});
	}

	public void predict(double power) {
		RealVector u = new ArrayRealVector(new double[] { power });

		x = A.operate(x).add(B.operate(u));
		P = A.multiply(P).multiply(A.transpose()).add(Q);
		P = P.add(P.transpose()).scalarMultiply(0.5); // helps with numerical stability
	}

	public void correct(double position, double velocity) {
		RealVector y = new ArrayRealVector(new double[] { position, velocity });

		double speed = Math.abs(x.getEntry(1));

		RealVector residual = y.subtract(C.operate(x));
		RealMatrix S = C.multiply(P).multiply(C.transpose()).add(R.scalarMultiply(speed));
		RealMatrix K = P.multiply(C.transpose()).multiply(MatrixUtils.inverse(S));
		x = x.add(K.operate(residual));
		P = MatrixUtils.createRealIdentityMatrix(2).subtract(K.multiply(C)).multiply(P);
	}

	public double getPosition() {
		return x.getEntry(0);
	}

	public double getVelocity() {
		return x.getEntry(1);
	}
}
