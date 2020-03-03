/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class LEDs extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static I2C _i2c;
  private static LEDs _instance;

  private java.util.Timer executor;
  private static final long THREAD_PERIOD = 20;

  private boolean writePattern;
  private boolean writePatternOneColor;

  public static final int PATTERN_OFF= 0;
	public static final int PATTERN_SOLID = 1;

  int pattern;
  byte lightByteOneColor[] = new byte[4];

/**
 * initializes _i2c variable
 */
  private LEDs()
  {
    _i2c = new I2C(RobotMap.I2C_PORT, RobotMap.I2C_ADDRESS);
    startThread();
  }

  public static LEDs getInstance()
  {
    if(_instance == null)
    {
      _instance = new LEDs();
    }
    return _instance;
  }

  public void startThread()
  {
    executor = new java.util.Timer();
    executor.schedule(new LedsUpdateTask(this), 0L, THREAD_PERIOD);
  }
  
  public void writePattern(int pattern)
  {
    this.pattern = pattern;
    writePattern = true;
  }

  public void writePatternOneColor(int pattern, int R, int B, int G)
  {
    lightByteOneColor[0] = (byte) R;
    lightByteOneColor[1] = (byte) G;
    lightByteOneColor[2] = (byte) B;
    lightByteOneColor[3] = (byte) pattern;
    writePatternOneColor = true;
  }


/**
 * Checks what the data you want to send is, and uses the correct method to format and send the data
 */
  private void run()
  {
    if(writePattern)
    {
      _i2c.write(RobotMap.I2C_ADDRESS, pattern);
      writePattern = false;
    }
    else if(writePatternOneColor)
    {
      _i2c.writeBulk(lightByteOneColor);
      writePatternOneColor = false;
    }
  }

  @Override // FIND A WAY TO FIX IT
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /**
 * If there are LEDs connected, the code runs, if not, it throws an exception
 */
    
  private class LedsUpdateTask extends TimerTask {
    private LEDs leds;

    private LedsUpdateTask(LEDs leds) {
      if (leds == null) {
        throw new NullPointerException("LEDs pointer null");
      }
      this.leds = leds;
    }

    public void run() {
      leds.run();
    }
  }
}
