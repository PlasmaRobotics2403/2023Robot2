// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controllers.PlasmaJoystick;

import frc.robot.auto.modes.*;
import frc.robot.auto.util.AutoMode;
import frc.robot.auto.util.AutoModeRunner;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Swerve swerve;
  private PlasmaJoystick driver;

  private Grabber grabber;
  private double armTarget;

  private double elevatorTarget;
  private Elevator elevator;

  private Boolean attemptingConeGrab;
  private Boolean attemptingCubeGrab;
  private Boolean attemptingConeOutake;
  private Boolean attemptingCubeOutake;

  AutoModeRunner autoModeRunner;
  AutoMode[] autoModes;
  int autoModeSelection;
  SendableChooser<Integer> autoChooser;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    driver = new PlasmaJoystick(0);

    elevator = new Elevator();
    swerve = new Swerve();
    grabber = new Grabber();

    armTarget = 0;
    autoModeSelection = 0;
    elevatorTarget = 0;
    attemptingConeGrab = false;
    attemptingCubeGrab = false;
    attemptingConeOutake = false;
    attemptingCubeOutake = false;


    autoModeRunner = new AutoModeRunner();
    autoModes = new AutoMode[20];
    autoModes[0] = new Nothing();
    autoModes[1] = new LeaveCommunity(swerve);
    autoModes[2] = new Score(swerve, elevator, grabber);

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("nothing", 0);
    SmartDashboard.putNumber("Nothing", 0);
    SmartDashboard.putNumber("Leave Community", 1);
    SmartDashboard.putNumber("Score", 2);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    autoModeSelection = (int) SmartDashboard.getNumber("Auton Mode", 0.0);
    SmartDashboard.putData("Auto Selector", autoChooser);
    SmartDashboard.putNumber("Auton Mode", autoModeSelection);

    swerve.logging();
    elevator.logger();
    grabber.logging();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    autoModeRunner.chooseAutoMode(autoModes[autoModeSelection]);
    autoModeRunner.start();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }
  

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    autoModeRunner.stop();
    swerve.defaultNeutralMode(true, true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(driver.START.isPressed()) {
      swerve.zeroHeading();
    }
    /* creep drive */
    else if(driver.A.isPressed()) {
      swerve.teleopDrive(Constants.Swerve.creepSpeed*driver.LeftY.getTrueAxis(), Constants.Swerve.creepSpeed*driver.LeftX.getTrueAxis(), Constants.Swerve.creepSpeed*driver.RightX.getTrueAxis(), false);
    }
    /* regular drive */
    else {
      swerve.teleopDrive(driver.LeftY.getTrueAxis(), driver.LeftX.getTrueAxis(), driver.RightX.getTrueAxis(), false);
    }

    elevator.magicElevator(elevatorTarget);
    grabber.armToPos(armTarget, 0.25);

    if(driver.dPad.getPOV() == 0) { /* high score state */
      elevatorTarget = Constants.ElevatorConstants.ELEVATOR_HIGH_EXTEND;
      armTarget = Constants.GrabberConstants.ARM_HIGH_EXTEND;

    }
    else if(driver.dPad.getPOV() == 90) { /* mid score state */
      elevatorTarget = Constants.ElevatorConstants.ELEVATOR_MID_EXTEND;
      armTarget = Constants.GrabberConstants.ARM_MID_EXTEND;
    }
    else if(driver.dPad.getPOV() == 270) { /* low score state */
      elevatorTarget = Constants.ElevatorConstants.ELEVATOR_LOW_EXTEND;
      armTarget = Constants.GrabberConstants.ARM_LOW_EXTEND;
     
    }
    else if(driver.dPad.getPOV() == 180) { /* stow state */
      elevatorTarget = Constants.ElevatorConstants.ELEVATOR_BOTTTOM_EXTEND;
      armTarget = Constants.GrabberConstants.ARM_STOW_EXTEND;
      
    }
    else if(driver.B.isPressed()) { /* feeder state */
      elevatorTarget = Constants.ElevatorConstants.ELEVATOR_FEEDER_EXTEND;
      armTarget = Constants.GrabberConstants.ARM_CONE_FEEDER_EXTEND;
      
    }
    if(driver.X.isPressed()) {
      elevatorTarget = Constants.ElevatorConstants.ELEVATOR_BOTTTOM_EXTEND;
      armTarget = Constants.GrabberConstants.ARM_CUBE_FEEDER_EXTEND;
    }
    // Cone outake
    if(driver.RB.isPressed()) {
      grabber.runGrabber(0.5);
      attemptingConeOutake = true;
    }

    if (attemptingConeOutake && !driver.RB.isPressed()) {
      grabber.runGrabber(0);
      attemptingConeOutake = false;
    }
    // Cone intake
    if(driver.RT.isPressed()) {
      grabber.runGrabberSesor(-0.5);
      attemptingConeGrab = true;
    }

    if (attemptingConeGrab && !driver.RT.isPressed()) {
      grabber.runGrabberSesor(0);
      attemptingConeGrab = false;

    }
    //Cube 
    if(driver.LB.isPressed()) {
      grabber.runGrabber(-0.5);
      attemptingCubeOutake = true;
    }

    if (attemptingCubeOutake && !driver.LB.isPressed()) {
      grabber.runGrabber(0);
      attemptingCubeOutake = false;
    }

    if(driver.LT.isPressed()) {
      grabber.runGrabberSesor(0.5);
      attemptingCubeGrab = true;
    }

    if (attemptingCubeGrab && !driver.LT.isPressed()) {
      grabber.runGrabber(0);
      attemptingCubeGrab = false;
    }
  }
  

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
