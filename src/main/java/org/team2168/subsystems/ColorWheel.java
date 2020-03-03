/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.ControlType;

import org.team2168.RobotMap;
import org.team2168.commands.color_wheel.DriveColorWheelWithJoystick;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;


public class ColorWheel extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final boolean COLOR_WHEEL_MOTOR_REVERSE = true;
  private CANSparkMax colorWheelMotor;
  private static ColorWheel instance = null;
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  private final double gearRatio = 25.0; // 25 internal means 1 external
  private final double ALLOWED_ERROR = (2.0 / 360.0);
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr = ALLOWED_ERROR;
  private double velocitySetPoint_sensorUnits, positionSetPoint_sensorUnits;

  private ColorWheel()
  {
    colorWheelMotor = new CANSparkMax(RobotMap.COLORWHEEL_MOTOR_PDP,MotorType.kBrushless); 
    colorWheelMotor.setSmartCurrentLimit(30);
    colorWheelMotor.setControlFramePeriodMs(20);
    colorWheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
    colorWheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    colorWheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

    colorWheelMotor.setIdleMode(IdleMode.kBrake);

        /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    colorWheelMotor.restoreFactoryDefaults();

    // initialze PID controller and encoder objects
    m_pidController = colorWheelMotor.getPIDController();
    m_encoder = colorWheelMotor.getEncoder();

    // PID coefficients
    kP = 0.0003; //5e-5
    kI = 0.0; //1e-6 
    kD = 0.0001; //0
    kIz = 20.0;
    kFF = 0.000156; 
    kMaxOutput = 1.0;
    kMinOutput = -1.0;
    maxRPM = 8.0;

    // Smart Motion Coefficients
    maxVel = 8.0*60.0; // rpm
    maxAcc = 8.0*60.0;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(revs_to_motor_rotations(kIz));
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a CANPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    m_pidController.setSmartMotionMaxVelocity(revs_to_motor_rotations(maxVel), smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity( revs_to_motor_rotations(minVel), smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(revs_to_motor_rotations(maxAcc), smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(revs_to_motor_rotations(allowedErr), smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);   

    // ConsolePrinter.putNumber("CW Velocity", () -> {return getVelocity();}, true, false);
    // ConsolePrinter.putNumber("CW Position", () -> {return getPosition();}, true, false);
    // ConsolePrinter.putNumber("CW Position Error", () -> {return getPositionError();}, true, false);
    // ConsolePrinter.putNumber("CW Motor Output Percent", () -> {return getMotorOutput();}, true, false);
  }

  /**
   * 
   * @return an instance of the ColorWheel subsystem. 
   */
  public static ColorWheel getInstance()
  {
    if (instance == null)
    {
      instance = new ColorWheel();
    }
    return instance;
  }

    /**
   * Sets the speed of the motor
   * 
   * @param speed positive moves big wheel ccw, 
   * negative moves big wheel cw, 0 is hold still
   * (positive moves little wheel cw, neg ccw)
   */
  public void drive(double speed)
  {
    if (COLOR_WHEEL_MOTOR_REVERSE)
    {
      speed = -speed;
    }
    colorWheelMotor.set(speed);
  }

  public void updatePIDValues()
  {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", ALLOWED_ERROR);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { m_pidController.setSmartMotionMaxVelocity(revs_to_motor_rotations(maxV),0); maxVel = maxV; }
    if((minV != minVel)) { m_pidController.setSmartMotionMinOutputVelocity(revs_to_motor_rotations(minV),0); minVel = minV; }
    if((maxA != maxAcc)) { m_pidController.setSmartMotionMaxAccel(revs_to_motor_rotations(maxA),0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_pidController.setSmartMotionAllowedClosedLoopError(revs_to_motor_rotations(allE),0); allowedErr = allE; }
  }

    /**
   * As with other PID modes, Velocity Closed Loop is set by calling the
   * setReference method on an existing pid object and setting
   * the control type to kVelocity

  /**
   * Gets the current color sensed by the color sensor on the robot
   * 
   */
  public char getSensorColor() //update to have code that specifically pulls the data from the sensor into one of 4 characters
  {
    return 'B';
  }

  

  /**
  * gets the color we need to sping to from the FMS 
  * 
  * @return the first letter of the color we need to position to as a char
  */
  public char getTargetColor()
  {
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      {
        case 'B' :
         return 'B';
       case 'G' :
          return 'G';
        case 'R' :
          return 'R';
        case 'Y' :
          return 'Y';
        default :
          return 'N';
      }
    }
    else {
      return 'N';
    } 
  }

  /**
  * Translates what our color sensor sees into what the Color Sensor on the color wheel itself sees
  * 
  * @return the first letter of the color that the field should be reading as a char
  */

  public char getCalculatedFieldColor()
  {
    char robotSensor = getSensorColor();
    if(robotSensor == 'B')
    {
      return 'Y';
    }
    else if(robotSensor == 'G')
    {
      return 'B';
    }
    else if(robotSensor == 'R')
    {
      return 'G';
    }
    else if(robotSensor == 'Y')
    {
      return 'R';
    }
    else
    {
      return 'N';
    }
  }

  /**
  * @return the value of the inputted color as a consistent integer or the number 5 to indicate error data
  */

  public int colorWheelMapping(char inputColor)
  {
    if (inputColor == 'B') //create a method for this
    {
      return 0;
    }
    else if (inputColor == 'G')
    {
      return 1;
    }
    else if (inputColor == 'R')
    {
      return 2;
    }
    else if (inputColor == 'Y')
    {
      return 3;
    }
    else
    {
      return 5; //error return value
    }
  }

  /**
  * @return the number of degrees required to spin the color wheel in order to land on 
  * the color needed by the FMS
  */
  
  public int degToSpin()
  {
    char desiredColor = getTargetColor(); 
    char currentColor = getCalculatedFieldColor(); 
    int desiredColorNum = 0;
    int currentColorNum = 0;
    int degrees = 0;
    
    currentColorNum = colorWheelMapping(currentColor);
    if (currentColorNum == 5)
    {
      return 0;
    }
    desiredColorNum = colorWheelMapping(desiredColor);
    if (desiredColorNum == 5)
    {
      return 0;
    }

    degrees = currentColorNum - desiredColorNum;
    if (degrees == 3)
    {
      degrees = 1;
    }
    else if (degrees == -3)
    {
      degrees = -1;
    }

    return degrees * 45;
    }

  /**
   * 
   * @return an instance of the ColorWheel subsystem. 
   */
  public void setVelocitySetPoint(double setPoint)
  {
    velocitySetPoint_sensorUnits = revs_to_motor_rotations(setPoint);
    m_pidController.setReference(velocitySetPoint_sensorUnits, ControlType.kVelocity);
    SmartDashboard.putNumber("velocity target sensor units", velocitySetPoint_sensorUnits);
    SmartDashboard.putNumber("velocity sensor unites", m_encoder.getVelocity());

  }

  public double getPosition()
  {
    return motor_rotations_to_revs(m_encoder.getPosition());
  }

  public double getVelocity()
  {
    return motor_rotations_to_revs(m_encoder.getVelocity());
  }

  public double getMotorOutput()
  {
    return colorWheelMotor.getAppliedOutput();
  }

  public double getAllowedClosedLoopError()
  {
    return motor_rotations_to_revs(m_pidController.getSmartMotionAllowedClosedLoopError(0));
  }

    /**
   * As with other PID modes, Smart Motion is set by calling the
   * setReference method on an existing pid object and setting
   * the control type to kSmartMotion
   */
  public void setPositionSetPoint(double setPoint)
  {
    positionSetPoint_sensorUnits = revs_to_motor_rotations(setPoint);

    m_pidController.setReference(positionSetPoint_sensorUnits, ControlType.kSmartMotion);
    System.out.println("setting inside method");
  }

  public void setSetpoint(double setPoint, boolean velocityMode)
  {
    if(velocityMode) {
      setVelocitySetPoint(setPoint);
    } else {
      setPositionSetPoint(setPoint);
    }
  }

  public double getPositionError()
  {
    return motor_rotations_to_revs(positionSetPoint_sensorUnits - m_encoder.getPosition());
  }

  public double getVelocityError()
  {
    return motor_rotations_to_revs(velocitySetPoint_sensorUnits - m_encoder.getVelocity());
  }

  public double getMaxVelocity()
  {
    return maxVel;
  }

  public double revs_to_motor_rotations(double setpoint)
  {
    return setpoint * gearRatio;
  }

  public double motor_rotations_to_revs(double setpoint)
  {
    return setpoint / gearRatio;
  }

  public void zeroEncoders()
  {
    m_encoder.setPosition(0.0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    // setDefaultCommand(new DriveColorWheelWithJoystick());
  }
}
