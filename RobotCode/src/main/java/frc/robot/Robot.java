// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.Calibration.CalWrangler;
import frc.lib.LoadMon.CasseroleRIOLoadMonitor;
import frc.lib.Signal.SignalWrangler;
import frc.lib.Webserver2.Webserver2;
import frc.lib.miniNT4.NT4Server;
import frc.robot.Autonomous.Autonomous;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.HumanInterfaces.DriverController;
import frc.robot.HumanInterfaces.OperatorController;
import frc.sim.Plant;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Website utilities
  Webserver2 webserver;
  Dashboard db;

  // Things
  PowerDistributionPanel pdp;
  CasseroleRIOLoadMonitor loadMon;
  DriverController dc;
  OperatorController oc;

  // Robot Subsystems
  Drivetrain dt;

  // Autonomous Control Utilities
  Autonomous auto;
  private PoseTelemetry pt;

  @Override
  public void robotInit() {

    NT4Server.getInstance(); // Ensure it starts

    /* Init website utilties */
    webserver = new Webserver2();
    CalWrangler.getInstance();
    PoseTelemetry.getInstance();

    dt = Drivetrain.getInstance();

    dc = DriverController.getInstance();
    oc = OperatorController.getInstance();

    auto = Autonomous.getInstance();

    db = new Dashboard(webserver);

    pt = PoseTelemetry.getInstance();

    if(Robot.isSimulation()){
      simulationSetup();
    }


    SignalWrangler.getInstance().registerSignals(this);
    webserver.startServer();

  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // Disabled-Specific Functions
  //////////////////////////////////////////////////////////////////////////////////////////

  @Override
  public void disabledInit() {
    SignalWrangler.getInstance().logger.stopLogging();
    auto.reset();
  }

  @Override
  public void disabledPeriodic() {
    auto.sampleDashboardSelector();
    dt.setCmd(0.0, 0.0);
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // Autonomous-Specific Functions
  //////////////////////////////////////////////////////////////////////////////////////////

  @Override
  public void autonomousInit() {
    syncSimPoseToEstimate();
    auto.startSequencer();
  }

  @Override
  public void autonomousPeriodic() { 
    auto.update();
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // Teleop-Specific Functions
  //////////////////////////////////////////////////////////////////////////////////////////

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    dc.update();
    oc.update();

    dt.setCmdCurvature(dc.getXSpeedCmd(), dc.getZRotateCmd(), (dc.getXSpeedCmd() == 0));

  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // Common Periodic Update Functions
  //////////////////////////////////////////////////////////////////////////////////////////

  @Override
  public void robotPeriodic() {
    dt.update();
    db.updateDriverView();
    telemetryUpdate();
  }

  private void telemetryUpdate(){
    double time = Timer.getFPGATimestamp();
    pt.update(time);
    SignalWrangler.getInstance().sampleAllSignals(time);
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // Simulation Support
  //////////////////////////////////////////////////////////////////////////////////////////

  Plant m_plant;

  public void simulationSetup(){
    m_plant = new Plant();
  }

  public void syncSimPoseToEstimate(){
    if(Robot.isSimulation()){
      m_plant.m_dt.resetPose(PoseTelemetry.getInstance().estimatedPose);
    }
  }

  @Override
  public void simulationPeriodic(){
    m_plant.update();
  }


}
