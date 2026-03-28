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
    // 状态: [位置, 速度, 加速度]
    // 输出: [位置, 速度]
    private final KalmanFilter<N3, N1, N2> m_filter;//状态3，输入1，观测2
    double kDt = 0.020;


    public KinematicPredictor() {
        // ... 在方法内部 ...


        // 1. 创建 A 矩阵 (3x3)
        // MatBuilder.fill(行维度, 列维度, 具体的数值...)
        Matrix<N3, N3> aMatrix = MatBuilder.fill(Nat.N3(), Nat.N3(),
            0, 1, 0,  // dp/dt = v
                    0, 0, 1,  // dv/dt = a
                    0, 0, 0   // da/dt = 0
        );

        // 2. 创建 B 矩阵 (3x1)
        Matrix<N3, N1> bMatrix = MatBuilder.fill(Nat.N3(), Nat.N1(),
            0, 0, 0 // 加速度直接受输入控制
        );

        // 3. 创建 C 矩阵 (2x3)
        Matrix<N2, N3> cMatrix = MatBuilder.fill(Nat.N2(), Nat.N3(),
            1, 0, 0, // 观测位置
            0, 1, 0  // 观测速度
        );

        // 4. 创建 D 矩阵 (2x1)
        Matrix<N2, N1> dMatrix = MatBuilder.fill(Nat.N2(), Nat.N1(),
            0, 0
        );

        // 5. 构建线性系统
        LinearSystem<N3, N1, N2> system = new LinearSystem<>(aMatrix, bMatrix, cMatrix, dMatrix);

        //增大响应，调大q，调小r
        // 过程噪声 Q: 相信模型程度 (对加速度的不确定性设高一点)
        var stateStdDevs = VecBuilder.fill(0.01, 0.05, 1.0);
        // 测量噪声 R: 相信传感器程度 (位置和速度都很准，设小一点)
        var measurementStdDevs = VecBuilder.fill(0.01, 0.02);

        m_filter = new KalmanFilter<>(Nat.N3(), Nat.N2(), system, stateStdDevs, measurementStdDevs, kDt);
    }

    public void update(double posObs, double velObs) {
        m_filter.predict(VecBuilder.fill(0.0), kDt);
        m_filter.correct(VecBuilder.fill(0.0), VecBuilder.fill(posObs, velObs));
    }

    public double getPredictedVelocity(double lookaheadTime) {
        // 根据当前估计的 v 和 a，预测未来时刻的速度
        // v_future = v_curr + a_curr * t
        double v = m_filter.getXhat(1);
        double a = m_filter.getXhat(2);
        return v + a * lookaheadTime;
    }

    public double getEstimatedAcceleration(){
        return m_filter.getXhat(2);
    }

    public double getPredictedPosition(double lookaheadTime) {
        // 根据当前估计的 x 和 v，预测未来时刻的位置
        // x_future = x_curr + v_curr * t
        double x = m_filter.getXhat(0);
        double v = m_filter.getXhat(1);
        return x + v * lookaheadTime;
    }
}