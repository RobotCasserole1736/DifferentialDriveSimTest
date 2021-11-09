package frc.sim.Drivetrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import frc.Constants;
import frc.robot.PoseTelemetry;
import frc.robot.Drivetrain.DtUtils;
import frc.sim.Drivetrain.Vision.Vision;
import frc.wrappers.SimCANDeviceBank;
import frc.wrappers.ADXRS453.ADXRS453Factory;
import frc.wrappers.ADXRS453.SimADXRS453;
import frc.wrappers.TalonFX.SimTalonFX;

public class DrivetrainPlant {

    // Simulated Sensors
    SimADXRS453 gyroSim = ADXRS453Factory.getSimGyro();
    Vision simVision = new Vision();

    // Simulated Motor Controllers
    SimTalonFX leftLeader    = (SimTalonFX) SimCANDeviceBank.get(Constants.kDtLeftLeaderCAN_ID   );
    SimTalonFX leftFollower  = (SimTalonFX) SimCANDeviceBank.get(Constants.kDtLeftFollowerCAN_ID );
    SimTalonFX rightLeader   = (SimTalonFX) SimCANDeviceBank.get(Constants.kDtRightLeaderCAN_ID  );
    SimTalonFX rightFollower = (SimTalonFX) SimCANDeviceBank.get(Constants.kDtRightFollowerCAN_ID);


    // Simulation Physics
    // Configure these to match your drivetrain's physical dimensions
    // and characterization results.
    DifferentialDrivetrainSim drivetrainSimulator =
            new DifferentialDrivetrainSim(
                    Constants.drivetrainSystem,
                    DCMotor.getCIM(2),
                    8,
                    Constants.kTrackWidth_m,
                    Constants.kWheelRadius_m,
                    null);



    public DrivetrainPlant() {
    }

    /**
    * Perform all periodic drivetrain simulation related tasks to advance our simulation of robot
    * physics forward by a single 20ms step.
    */
    public void update() {

        double leftMotorVoltage = 0;
        double rightMotorVoltage = 0;

        if (DriverStation.getInstance().isEnabled() && !RobotController.isBrownedOut()) {
            // If the motor controllers are enabled...
            // Roughly model the effect of leader and follower motor pushing on the same gearbox.
            // Note if the software is incorrect and drives them against each other, speed will be
            // roughly matching the physical situation, but current draw will _not_ be accurate.
            leftMotorVoltage = (leftLeader.sim_getWindingVoltage() + leftFollower.sim_getWindingVoltage()) / 2.0;
            rightMotorVoltage = (rightLeader.sim_getWindingVoltage() + rightFollower.sim_getWindingVoltage()) / 2.0;
        }

        // Update the physics simulation
        drivetrainSimulator.setInputs(leftMotorVoltage,rightMotorVoltage);
        drivetrainSimulator.update(Constants.Ts);

        // Update our sensors based on the simulated motion of the robot
        var leftVel = drivetrainSimulator.getLeftVelocityMetersPerSecond();
        var leftDist = drivetrainSimulator.getLeftPositionMeters();
        var rightVel = drivetrainSimulator.getRightVelocityMetersPerSecond();
        var rightDist = drivetrainSimulator.getRightPositionMeters();

        leftLeader.sim_setActualVelocity(DtUtils.DtLinM_to_MotorRotationRad(leftVel));
        leftFollower.sim_setActualVelocity(DtUtils.DtLinM_to_MotorRotationRad(leftVel));
        rightLeader.sim_setActualVelocity(DtUtils.DtLinM_to_MotorRotationRad(rightVel));
        rightFollower.sim_setActualVelocity(DtUtils.DtLinM_to_MotorRotationRad(rightVel));

        leftLeader.sim_setActualPosition(DtUtils.DtLinM_to_MotorRotationRad(leftDist));
        leftFollower.sim_setActualPosition(DtUtils.DtLinM_to_MotorRotationRad(leftDist));
        rightLeader.sim_setActualPosition(DtUtils.DtLinM_to_MotorRotationRad(rightDist));
        rightFollower.sim_setActualPosition(DtUtils.DtLinM_to_MotorRotationRad(rightDist));

        gyroSim.simSetAngle(
                -drivetrainSimulator
                        .getHeading()
                        .getDegrees()); // Gyros have an inverted reference frame for
                                        // angles, so multiply by -1.0;

        simVision.setCurrentPose(drivetrainSimulator.getPose());
        simVision.update();
        
        PoseTelemetry.getInstance().setActualPose(drivetrainSimulator.getPose());
    }

    /**
    * Resets the simulation back to a pre-defined pose Useful to simulate the action of placing the
    * robot onto a specific spot in the field (IE, at the start of each match).
    *
    * @param pose
    */
    public void resetPose(Pose2d pose) {
        drivetrainSimulator.setPose(pose);
    }

    /** @return The simulated robot's position, in meters. */
    public Pose2d getCurPose() {
        return drivetrainSimulator.getPose();
    }

    /**
    * For testing purposes only! Applies an unmodeled, undetected offset to the pose Similar to if
    * you magically kicked your robot to the side in a way the encoders and gyro didn't measure.
    *
    * <p>This disturbance should be corrected for once a vision target is in view.
    */
    public void applyKick() {
        Pose2d newPose =
                drivetrainSimulator
                        .getPose()
                        .transformBy(new Transform2d(new Translation2d(0, 0.5), new Rotation2d()));
        drivetrainSimulator.setPose(newPose);
    }
    
}
