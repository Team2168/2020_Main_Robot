/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168;

import org.team2168.subsystems.Climber;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.Hopper;
import org.team2168.subsystems.ColorWheel;
import org.team2168.subsystems.ColorWheelPivot;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.HoodAdjust;
import org.team2168.subsystems.IntakeMotor;
import org.team2168.subsystems.IntakePivot;
import org.team2168.subsystems.Shooter;
//import org.team2168.utils.Debouncer;
import org.team2168.utils.PowerDistribution;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.team2168.utils.consoleprinter.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team2168.subsystems.Balancer;
import org.team2168.OI;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Subsystems
  private static Climber climber;
  private static IntakeMotor intakeMotor;
  private static IntakePivot intakePivot;
  private static Indexer indexer;
  private static Balancer balancer;
  private static Hopper hopper;
  private static ColorWheel colorWheel;
  private static ColorWheelPivot colorWheelPivot;
  private static Shooter shooter;
  private static HoodAdjust hoodAdjust;
  private static Drivetrain drivetrain;

  private static OI oi;

  private static PowerDistribution pdp;

  static boolean autoMode;
  // private static boolean matchStarted = false;
  // private static int gyroReinits;
  // private double lastAngle;
  // private Debouncer gyroDriftDetector = new Debouncer(1.0);
  // private static boolean gyroCalibrating = false;




  // private boolean lastGyroCalibrating = false;
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
  	ConsolePrinter.init();

    // colorWheel = ColorWheel.getInstance();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
	  SmartDashboard.putData("Auto choices", m_chooser);
	
    //Init Subsystems
    // climber = Climber.getInstance();
    // intakeMotor = IntakeMotor.getInstance();
    // intakePivot = IntakePivot.getInstance();
    // balancer = Balancer.getInstance();
    indexer = Indexer.getInstance();
    hopper = Hopper.getInstance();
    colorWheel = ColorWheel.getInstance();
    colorWheelPivot = ColorWheelPivot.getInstance();
    shooter = Shooter.getInstance();
    hoodAdjust = HoodAdjust.getInstance();
    drivetrain = Drivetrain.getInstance();
    oi = OI.getInstance();
    
    // pdp = new PowerDistribution(RobotMap.PDPThreadPeriod);
    // pdp.startThread();
    ConsolePrinter.startThread();
  }

  @Override
  public void robotPeriodic() {
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      break;

    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    //getControlStyleInt();
    //controlStyle = (int) controlStyleChooser.getSelected();
    Scheduler.getInstance().run();
  }
}
