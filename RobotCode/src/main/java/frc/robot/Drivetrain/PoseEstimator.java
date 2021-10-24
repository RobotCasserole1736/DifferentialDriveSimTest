package frc.robot.Drivetrain;


import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;
import edu.wpi.first.wpiutil.math.numbers.N5;
import frc.robot.PoseTelemetry;
import frc.robot.Vision;
import frc.wrappers.ADXRS453.ADXRS453Factory;
import frc.wrappers.ADXRS453.CasseroleADXRS453;

public class PoseEstimator {
    // Sensors used as part of the Pose Estimation
    private final CasseroleADXRS453 gyro = ADXRS453Factory.makeNewGyro();
    private Vision cam = Vision.getInstance();
    // Note - drivetrain encoders are also used. The Drivetrain class must pass us
    // the relevant readings.

    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much you trust your
    // various sensors. Smaller numbers will cause the filter to "trust" the estimate from that particular
    // component more than the others. This in turn means the particular component will have a stronger
    // influence on the final pose estimate.
    Matrix<N5, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05);
    Matrix<N3, N1> localMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));
    Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));

    private final DifferentialDrivePoseEstimator m_poseEstimator =
            new DifferentialDrivePoseEstimator(
                    gyro.getRotation2d(),
                    new Pose2d(),
                    stateStdDevs,
                    localMeasurementStdDevs,
                    visionMeasurementStdDevs);

    public PoseEstimator() {}

    /**
    * Perform all periodic pose estimation tasks.
    *
    * @param actWheelSpeeds Current Speeds (in m/s) of the drivetrain wheels
    * @param leftWheelDeltaDist Distance (in m) the left wheel has traveled since the last call to update
    * @param rightWheelDeltaDist Distance (in m) the right wheel has traveled since the last call to update
    */
    public void update(
            DifferentialDriveWheelSpeeds actWheelSpeeds, double leftWheelDeltaDist, double rightWheelDeltaDist) {

        // Incorporate any available camera data
        if(cam.targetVisble()){
            m_poseEstimator.addVisionMeasurement(cam.getDtPoseEst(), cam.getCaptureTime());
        }

        // Step the estimator forward based on gyro/drivetrain sensor readings
        m_poseEstimator.update(gyro.getRotation2d().unaryMinus(), actWheelSpeeds, leftWheelDeltaDist, rightWheelDeltaDist);

        // Update telemetry
        PoseTelemetry.getInstance().setEstimatedPose(getPoseEst());
    }

    /**
    * Force the pose estimator to a particular pose. This is useful for indicating to the software
    * when you have manually moved your robot in a particular position on the field (EX: when you
    * place it on the field at the start of the match).
    *
    * @param pose
    */
    public void resetToPose(Pose2d pose) {
        m_poseEstimator.resetPosition(pose, gyro.getRotation2d());
    }

    /** @return The current best-guess at drivetrain position on the field. */
    public Pose2d getPoseEst() {
        return m_poseEstimator.getEstimatedPosition();
    }
    
}
