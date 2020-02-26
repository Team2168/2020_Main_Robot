
package org.team2168;

import org.team2168.commands.auto.DefaultTrenchAuto;
import org.team2168.commands.auto.DoNothing;
import org.team2168.commands.auto.OppositeTrenchAuto;
import org.team2168.commands.drivetrain.FirstPath;
import org.team2168.commands.drivetrain.PIDCommands.DriveXDistance;
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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  	//Driverstation Instance
	public static DriverStation driverstation;

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
    driverstation = DriverStation.getInstance();

    ConsolePrinter.init();
    ConsolePrinter.startThread();

    //Initialize Autonomous Selector Choices
    autoSelectInit();

    ConsolePrinter.putBoolean("isPracticeBot", ()->{return isPracticeBot();}, true, false);
    ConsolePrinter.putSendable("Autonomous Mode Chooser", () -> {return Robot.autoChooser;}, true, false);

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
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    autoMode = false;
    Scheduler.getInstance().run();
    // limelight.enableLimelight(hoodAdjust.getHoodPosition());
  }

    /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopInit() {
    autoMode = false;
    Scheduler.getInstance().run();
        // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to 
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) autonomousCommand.cancel();
    limelight.pauseLimelight();

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledInit() {
    autoMode = false;
    limelight.pauseLimelight();

  }

  @Override
  public void disabledPeriodic() {
    //getControlStyleInt();
    //controlStyle = (int) controlStyleChooser.getSelected();
    Scheduler.getInstance().run();
    autonomousCommand = (Command) autoChooser.getSelected();

  }

  public static boolean onBlueAlliance() {
		return driverstation.getAlliance() == DriverStation.Alliance.Blue;

	}
      
    /**
     * Adds the autos to the selector
     */
    public void autoSelectInit() {
      autoChooser = new SendableChooser<Command>();
      autoChooser.setDefaultOption("Drive Straight", new DriveXDistance(60.0));
      autoChooser.addOption("Do Nothing", new DoNothing());
      autoChooser.addOption("Opposite Trench Auto ", new OppositeTrenchAuto());
      autoChooser.addOption("Near Trench Auto", new DefaultTrenchAuto());
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
}
