package frc.robot.Drivetrain;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.Constants;
import frc.lib.Signal.Annotations.Signal;
import frc.wrappers.TalonFX.CasseroleTalonFX;
import frc.wrappers.TalonFX.TalonFXFactory;

public class HwInterface {

    // Hardware under control
    CasseroleTalonFX leftLeader;
    CasseroleTalonFX leftFollower;
    CasseroleTalonFX rightLeader;
    CasseroleTalonFX rightFollower;

    public enum DtHwCtrlMode {
        Voltage(0), // Open Loop Voltage Command Only
        ClosedLoopVelocity(1); // Closed loop velocity
        public final int value;
        private DtHwCtrlMode(int value) { this.value = value;}
        public int toInt() {return this.value;}
    }

    @Signal
    DtHwCtrlMode curCtrlMode = DtHwCtrlMode.Voltage;

    double prevLeftPos_rad = 0;
    double prevRightPos_rad = 0;

    double leftDeltaDist_m = 0;
    double rightDeltaDist_m = 0;

    @Signal
    double leftDesSpd_rps = 0;
    @Signal
    double rightDesSpd_rps = 0;
    @Signal
    double leftActSpd_rps = 0;
    @Signal
    double rightActSpd_rps = 0;

    @Signal
    double rightOLVoltageCmd = 0;
    @Signal
    double leftOLVoltageCmd = 0;

    DifferentialDriveWheelSpeeds curSpdCmds;

    public HwInterface(){

        leftLeader    = TalonFXFactory.makeNewController(Constants.kDtLeftLeaderCAN_ID);
        leftFollower  = TalonFXFactory.makeNewController(Constants.kDtLeftFollowerCAN_ID);
        rightLeader   = TalonFXFactory.makeNewController(Constants.kDtRightLeaderCAN_ID);
        rightFollower = TalonFXFactory.makeNewController(Constants.kDtRightFollowerCAN_ID);
        
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

    }

    //Open loop percent battery voltage commands
    public void setCommand(double leftVoltage, double rightVoltage){
        curCtrlMode = DtHwCtrlMode.Voltage;
        rightOLVoltageCmd = rightVoltage;
        leftOLVoltageCmd = leftVoltage;
    }

    //Closed-loop wheel speed commands
    public void setCommand(DifferentialDriveWheelSpeeds spdCmds){
        curCtrlMode = DtHwCtrlMode.ClosedLoopVelocity;
        curSpdCmds = spdCmds;
    }

    public double getLeftDeltaDist(){
        //returns distance in meters in the last update loop
        return leftDeltaDist_m;
    }

    public double getRightDeltaDist(){
        //returns distance in meters in the last update loop
        return rightDeltaDist_m;
    }

    
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        var left_mps = DtUtils.MotorRotationRad_to_DtLinM(leftActSpd_rps);
        var right_mps = DtUtils.MotorRotationRad_to_DtLinM(rightActSpd_rps);
        return new DifferentialDriveWheelSpeeds(left_mps, right_mps);
    }

    /**
     * Samples input from the drivetrain motors
     */
    public void updateInputs(){

        var curLeftPos_rad = leftLeader.getPosition_rad();
        var curRightPos_rad = rightLeader.getPosition_rad();

        leftDeltaDist_m = DtUtils.MotorRotationRad_to_DtLinM(curLeftPos_rad - prevLeftPos_rad);
        rightDeltaDist_m = DtUtils.MotorRotationRad_to_DtLinM(curRightPos_rad - prevRightPos_rad);

        prevLeftPos_rad = curLeftPos_rad;
        prevRightPos_rad = curRightPos_rad;

        leftActSpd_rps  = leftLeader.getVelocity_radpersec();
        rightActSpd_rps = rightLeader.getVelocity_radpersec();

    }

    /**
     * Updates the commands to the drivetrain motors
     * @param estPose Most recent estimate of the robot's drivetrain pose on the field
     */
    public void updateOutputs(){
        switch(curCtrlMode){
            case Voltage:
                leftLeader.setVoltageCmd(leftOLVoltageCmd);
                rightLeader.setVoltageCmd(rightOLVoltageCmd);
            break;
            case ClosedLoopVelocity:
                leftDesSpd_rps = DtUtils.DtLinM_to_MotorRotationRad(curSpdCmds.leftMetersPerSecond);
                rightDesSpd_rps = DtUtils.DtLinM_to_MotorRotationRad(curSpdCmds.rightMetersPerSecond);
                leftLeader.setVelocityCmd(leftDesSpd_rps);
                rightLeader.setVelocityCmd(rightDesSpd_rps);
            break;
            default:
                leftLeader.setVoltageCmd(0.0);
                rightLeader.setVoltageCmd(0.0);
            break;
        }

    }

}
