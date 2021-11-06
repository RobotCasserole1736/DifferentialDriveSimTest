package frc.robot.Drivetrain;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.Constants;
import frc.lib.Signal.Annotations.Signal;
import frc.robot.PoseTelemetry;

public class Drivetrain {

    /* Singleton infratructure*/
    private static Drivetrain inst = null;
    public static synchronized Drivetrain getInstance() {
        if (inst == null)
            inst = new Drivetrain();
        return inst;
    }

    public enum DtControlStrategy {
        DifferentialDrive(0), // FwdRev, rotate
        CurvatureDrive(1), //Same as diff drive but rotation is curvature
        Ramsete(2); // Ramsete controller closed loop velocity
        public final int value;
        private DtControlStrategy(int value) { this.value = value;}
        public int toInt() {return this.value;}
    }

    @Signal
    DtControlStrategy curCtrlStrat = DtControlStrategy.DifferentialDrive;

    double leftMotorOLCmd;
    double rightMotorOLCmd;

    RamseteController ramsete_ctrl;
    Trajectory.State curDesTrajState;

    private HwInterface hwInf;
    private PoseEstimator poseEst;

    DifferentialDrive dt;

    private Drivetrain(){
        hwInf = new HwInterface();
        poseEst = new PoseEstimator();

        ramsete_ctrl = new RamseteController();

    }

    public void update(){
        hwInf.updateInputs();
        poseEst.update(hwInf.getWheelSpeeds(), hwInf.getLeftDeltaDist(), hwInf.getRightDeltaDist());

        switch(curCtrlStrat){
            case CurvatureDrive:
            case DifferentialDrive:
                var vLeft  = Constants.kMaxDtAppliedVoltage * leftMotorOLCmd;
                var vRight = Constants.kMaxDtAppliedVoltage * rightMotorOLCmd;
                hwInf.setCommand(vLeft, vRight);
                break;
            case Ramsete:
                PoseTelemetry.getInstance().setDesiredPose(curDesTrajState.poseMeters);
                ChassisSpeeds adjustedSpeeds = ramsete_ctrl.calculate(poseEst.getPoseEst(), curDesTrajState); 
                var curSpdCmds = Constants.kDtKinematics.toWheelSpeeds(adjustedSpeeds);
                hwInf.setCommand(curSpdCmds);
                break;
            default:
                hwInf.setCommand(0.0, 0.0);
                break;
            
        }

        hwInf.updateOutputs();

    }

    public void setCmd(double xSpeed, double zRotation){
        curCtrlStrat = DtControlStrategy.DifferentialDrive;
    
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    
        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
    
        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOLCmd = maxInput;
                rightMotorOLCmd = xSpeed - zRotation;
            } else {
                leftMotorOLCmd = xSpeed + zRotation;
                rightMotorOLCmd = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOLCmd = xSpeed + zRotation;
                rightMotorOLCmd = maxInput;
            } else {
                leftMotorOLCmd = maxInput;
                rightMotorOLCmd = xSpeed - zRotation;
            }
        }

    }

    public static final double quickStopThreshold = 0.2;
    public static final double quickStopAlpha = 0.1;
    private double m_quickStopAccumulator = 0.0;

    public void setCmdCurvature(double xSpeed, double zRotation, boolean isQuickTurn){
        curCtrlStrat = DtControlStrategy.CurvatureDrive;

        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    
        double angularPower;
        boolean overPower;
    
        if (isQuickTurn) {
          if (Math.abs(xSpeed) < quickStopThreshold) {
            m_quickStopAccumulator =
                (1 - quickStopAlpha) * m_quickStopAccumulator
                    + quickStopAlpha * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
          }
          overPower = true;
          angularPower = zRotation;
        } else {
          overPower = false;
          angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;
    
          if (m_quickStopAccumulator > 1) {
            m_quickStopAccumulator -= 1;
          } else if (m_quickStopAccumulator < -1) {
            m_quickStopAccumulator += 1;
          } else {
            m_quickStopAccumulator = 0.0;
          }
        }
    
        //Positive Z rotation implies right side going forward, left side going reverse
        leftMotorOLCmd = xSpeed - angularPower;
        rightMotorOLCmd = xSpeed + angularPower;
    
        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
          if (leftMotorOLCmd > 1.0) {
            rightMotorOLCmd -= leftMotorOLCmd - 1.0;
            leftMotorOLCmd = 1.0;
          } else if (rightMotorOLCmd > 1.0) {
            leftMotorOLCmd -= rightMotorOLCmd - 1.0;
            rightMotorOLCmd = 1.0;
          } else if (leftMotorOLCmd < -1.0) {
            rightMotorOLCmd -= leftMotorOLCmd + 1.0;
            leftMotorOLCmd = -1.0;
          } else if (rightMotorOLCmd < -1.0) {
            leftMotorOLCmd -= rightMotorOLCmd + 1.0;
            rightMotorOLCmd = -1.0;
          }
        }
    
        // Normalize the wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftMotorOLCmd), Math.abs(rightMotorOLCmd));
        if (maxMagnitude > 1.0) {
          leftMotorOLCmd /= maxMagnitude;
          rightMotorOLCmd /= maxMagnitude;
        }

    }

    //Ramsete pose-based command
    public void setCmd(Trajectory.State dState){
        curCtrlStrat = DtControlStrategy.Ramsete;
        curDesTrajState = dState;
    }

    /**
     * Resets pose estimator to a known position
     * @param pose_in
     */
    public void setCurPose(Pose2d pose_in){
        poseEst.resetToPose(pose_in);
    }

    /**
     * Gets current drivetrain pose estimation
     */
    public Pose2d getCurPoseEst(){
      return poseEst.getPoseEst();
  }
    

    
}
