package frc.robot.util;


import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;

public class KinematicPredictor {
    // State: [position, velocity, acceleration]
    // Output: [position, velocity]
    private final KalmanFilter<N3, N1, N2> m_filter; // 3 states, 1 input, 2 outputs
    double kDt = 0.020;


    public KinematicPredictor() {
        // 1. Build A matrix (3x3)
        // MatBuilder.fill(rows, cols, values...)
        Matrix<N3, N3> aMatrix = MatBuilder.fill(Nat.N3(), Nat.N3(),
            0, 1, 0,  // dp/dt = v
                    0, 0, 1,  // dv/dt = a
                    0, 0, 0   // da/dt = 0
        );

        // 2. Build B matrix (3x1)
        Matrix<N3, N1> bMatrix = MatBuilder.fill(Nat.N3(), Nat.N1(),
            0, 0, 0 // Acceleration directly controlled by input
        );

        // 3. Build C matrix (2x3)
        Matrix<N2, N3> cMatrix = MatBuilder.fill(Nat.N2(), Nat.N3(),
            1, 0, 0, // Observe position
            0, 1, 0  // Observe velocity
        );

        // 4. Build D matrix (2x1)
        Matrix<N2, N1> dMatrix = MatBuilder.fill(Nat.N2(), Nat.N1(),
            0, 0
        );

        // 5. Build linear system
        LinearSystem<N3, N1, N2> system = new LinearSystem<>(aMatrix, bMatrix, cMatrix, dMatrix);

        // Increase responsiveness: raise Q (process noise), lower R (measurement noise)
        // Process noise Q: model trust (higher uncertainty on acceleration)
        var stateStdDevs = VecBuilder.fill(0.01, 0.05, 1.0);
        // Measurement noise R: sensor trust (position and velocity are accurate, set low)
        var measurementStdDevs = VecBuilder.fill(0.01, 0.02);

        m_filter = new KalmanFilter<>(Nat.N3(), Nat.N2(), system, stateStdDevs, measurementStdDevs, kDt);
    }

    public void update(double posObs, double velObs) {
        m_filter.predict(VecBuilder.fill(0.0), kDt);
        m_filter.correct(VecBuilder.fill(0.0), VecBuilder.fill(posObs, velObs));
    }

    public double getPredictedVelocity(double lookaheadTime) {
        // Predict future velocity from current estimated v and a
        // v_future = v_curr + a_curr * t
        double v = m_filter.getXhat(1);
        double a = m_filter.getXhat(2);
        return v + a * lookaheadTime;
    }

    public double getEstimatedAcceleration(){
        return m_filter.getXhat(2);
    }

    public double getPredictedPosition(double lookaheadTime) {
        // Predict future position from current estimated x and v
        // x_future = x_curr + v_curr * t
        double x = m_filter.getXhat(0);
        double v = m_filter.getXhat(1);
        return x + v * lookaheadTime;
    }
}
