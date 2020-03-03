/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The MotionMagic_TalonFX_AuxStraightPigeon example demonstrates the Talon auxiliary and 
 * remote features to peform complex closed loops. This example has the robot performing 
 * Motion Magic with an auxiliary closed loop on Pigeon Yaw to keep the robot straight.
 * 
 * This example uses:
 * - 2x Talon FX's (one per side).  
 *     Talon FX calculates the distance by taking the sum of both integrated sensors and dividing it by 2.
 * - Pigeon IMU wired on CAN Bus for Auxiliary Closed Loop on Yaw
 * 
 * This example has two modes of operation, which can be switched between with Button 2.
 * 1.) Arcade Drive
 * 2.) Motion Magic with Talon FX's Encoders and Drive Straight With Pigeon yaw
 * 
 * Controls:
 * Button 1: When pressed, zero sensors. Set integrated encoders' positions + Pigeon yaw to 0.
 * Button 2: When pressed, toggle between Arcade Drive and Motion Magic
 * 	When toggling into Motion Magic, the current heading is saved and used as the 
 * 	auxiliary closed loop target. Can be changed by toggling out and in again.
 * Y Button (button 4): when in motion magic mode, sets the heading to 0.0 degrees
 * Button 5(Left shoulder): When pushed, will decrement the smoothing of the motion magic down to 0
 * Button 6(Right shoulder): When pushed, will increment the smoothing of the motion magic up to 8
 * Select button (button 7): when in motion magic mode, sets the heading to -90 degrees
 * Start button (button 8): when in motion magic mode, sets the heading to +90.0 degrees
 * Left Joystick Y-Axis: 
 * 	+ Arcade Drive: Drive robot forward and reverse
 * 	+ Motion Magic: Servo robot forward and reverse [-6, 6] rotations
 * Right Joystick X-Axis: 
 *  + Arcade Drive: Turn robot in left and right direction
 *  + Motion Magic: Not used
 * 
 * Gains for Motion Magic and Auxiliary may need to be adjusted in Constants.java
 * 
 * Supported Version:
 * - Talon FX: 20.2.3.0
 * - Pigeon IMU: 20.0
 */

package org.team2168;

import org.team2168.commands.auto.DefaultTrenchAuto;
import org.team2168.commands.auto.DoNothing;
import org.team2168.commands.auto.OppositeTrenchAuto;
import org.team2168.commands.drivetrain.PIDCommands.DriveXDistance;
import org.team2168.commands.drivetrain.PIDCommands.TurnXAngle;
import org.team2168.commands.hood_adjust.MoveToFiringLocation;
import org.team2168.subsystems.Balancer;
import org.team2168.subsystems.Climber;
import org.team2168.subsystems.ColorWheel;
import org.team2168.subsystems.ColorWheelPivot;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.HoodAdjust;
import org.team2168.subsystems.Hopper;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeMotor;
import org.team2168.subsystems.IntakePivot;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Shooter;
//import org.team2168.utils.Debouncer;
import org.team2168.utils.PowerDistribution;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Robot extends TimedRobot {	
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  static Command autonomousCommand;
  public static SendableChooser<Command> autoChooser;

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
  private static Limelight limelight;

  private static OI oi;

  private static DigitalInput practiceBot;

  private static PowerDistribution pdp;

  static boolean autoMode;
  public static final boolean ENABLE_BUTTON_BOX = true;
  private static boolean lastCallHoodButtonA = false;
  private MoveToFiringLocation moveHood;
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
  
    practiceBot = new DigitalInput(RobotMap.PRACTICE_BOT_JUMPER);

    //Init Subsystems
    // climber = Climber.getInstance();
    intakeMotor = IntakeMotor.getInstance();
    intakePivot = IntakePivot.getInstance();
    balancer = Balancer.getInstance();
    indexer = Indexer.getInstance();
    hopper = Hopper.getInstance();
    colorWheel = ColorWheel.getInstance();
    colorWheelPivot = ColorWheelPivot.getInstance();
    shooter = Shooter.getInstance();
    hoodAdjust = HoodAdjust.getInstance();
    drivetrain = Drivetrain.getInstance();
    limelight = Limelight.getInstance();
    oi = OI.getInstance();  

    // pdp = new PowerDistribution(RobotMap.PDPThreadPeriod);
    // pdp.startThread();
    ConsolePrinter.init();
    ConsolePrinter.startThread();

    //Initialize Autonomous Selector Choices
    autoSelectInit();

    ConsolePrinter.putBoolean("isPracticeBot", ()->{return isPracticeBot();}, true, false);
    ConsolePrinter.putSendable("Autonomous Mode Chooser", () -> {return Robot.autoChooser;}, true, false);

    drivetrain.setDefaultBrakeMode();
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
    autoMode = true;
    drivetrain.setDefaultBrakeMode();

		autonomousCommand = (Command) autoChooser.getSelected();
    	
    // schedule the autonomous command
    if (autonomousCommand != null) 
      autonomousCommand.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    autoMode = true;
    Scheduler.getInstance().run();
  }

  /**
   * This function called prior to robot entering Teleop Mode
   */
	public void teleopInit() {
    autoMode = false;
    drivetrain.setDefaultBrakeMode();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to 
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) autonomousCommand.cancel();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    boolean buttonBox2_buttonA = oi.buttonBox2.isPressedButtonA();
    autoMode = false;

    if(!oi.driverJoystick.isPressedButtonLeftBumper()
        && (buttonBox2_buttonA && !lastCallHoodButtonA)) {
      //The driver isn't going under the trench
      //the operator pressed hood raise button
      //raise the hood to the firing position
      moveHood = new MoveToFiringLocation(shooter.getFiringLocation());
      moveHood.start();
    } else if (!buttonBox2_buttonA && lastCallHoodButtonA) {
      // or the operator isn't pressing hood raise button
      //lower the hood
      moveHood = new MoveToFiringLocation(Shooter.FiringLocation.WALL);
      moveHood.start();
    }

    lastCallHoodButtonA = buttonBox2_buttonA;

    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    autoMode = false;
  }

  @Override
  public void disabledInit() {
    autoMode = false;

    if(!DriverStation.getInstance().isFMSAttached()) {
      //If we're not on a real field, let the robot be pushed around if it's disabled.
      drivetrain.setAllMotorsCoast();
    }

    lastCallHoodButtonA = false;
    limelight.setLedMode(1);
  }

  @Override
  public void disabledPeriodic() {
    autoMode = false;
    //getControlStyleInt();
    //controlStyle = (int) controlStyleChooser.getSelected();
    Scheduler.getInstance().run();
    autonomousCommand = (Command) autoChooser.getSelected();

  }

      
    /**
     * Adds the autos to the selector
     */
    public void autoSelectInit() {
      autoChooser = new SendableChooser<Command>();
      autoChooser.setDefaultOption("Drive Straight", new DriveXDistance(-60.0));
      autoChooser.addOption("Do Nothing", new DoNothing());
      autoChooser.addOption("Opposite Trench Auto ", new OppositeTrenchAuto());
      autoChooser.addOption("Near Trench Auto", new DefaultTrenchAuto());
      autoChooser.addOption("Turn 13.25", new TurnXAngle(13.25, 0.3));

    }

  /**
   * TODO return jumper value from DIO 24
   */
  public static boolean isPracticeBot() {
    // return true;
    return !practiceBot.get();
  }

  public static boolean isAutoMode() {
    return autoMode;
  }

  public static boolean onBlueAlliance() {
    return DriverStation.getInstance().getAlliance() == Alliance.Blue;
  }
}
