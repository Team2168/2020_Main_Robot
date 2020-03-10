/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.I2C;



public class ColorSensor extends Subsystem {

  private static I2C _arduino;
  private byte[] _receivedData;
  private double g_norm = 0;
  private double r_norm = 0;
  private double b_norm = 0;
  private int r = 0;
  private int g = 0;
  private int b = 0;
  private static ColorSensor instance = null;



  private ColorSensor()
  {
    _arduino = new I2C(I2C.Port.kOnboard,8);
    _receivedData  = new byte[3];
  }

/**
 * Takes in the data from the color wheel sensor, normalizes to a degree of 255,
 * then runs and returns calculateColorData() with the data it just received
 */
  public String getColorData()
  {
    _arduino.readOnly(_receivedData, _receivedData.length);
    r = _receivedData[0] & 0xff;
    g = _receivedData[1] & 0xff;
    b = _receivedData[2] & 0xff;

    float MAX;  
    int NORM_SCALE= 255;
    


    MAX = r;
  
    if (g > MAX){
      MAX = g;
    }
    
    if (b> MAX){
      MAX = b;
    }
    
    if (MAX > 0){
      r_norm = NORM_SCALE*r/MAX;
      g_norm = NORM_SCALE*g / MAX;
      b_norm = NORM_SCALE*b / MAX;    
    }
    
    else{
      r_norm = 0;
      g_norm = 0;
      b_norm = 0;  
    }
    return (calculateColorData(r_norm, g_norm, b_norm));
  }
  
/**
 * @return a string either saying Red, Green, Blue, or Yellow based on how much of each color was provided.
 * If the values are outside that of these four colors, @return a ? instead as an error
 */

  public String calculateColorData(double r, double g, double b){
    if (r_norm ==255 && g_norm <200 && b_norm <200) {
      return("Red");
    }
  
    else if (g_norm ==255 && r_norm <200 &&(b_norm <150)) {
      return("Green");
    }
  
    else if ((r_norm ==255 && (g_norm>200) && b_norm <150) || (g_norm ==255 && (r_norm>200) && b_norm <150)){
      //Serial.print ("Yellow ");  
      return("Yellow");
      // RGB_color(200,150,0); // LED
    }
  
    else if (b_norm ==255 && r_norm <200  && g_norm <200) {
      //Serial.print ("Blue "); 
      return("Blue");
      // RGB_color(0,0,255); // LED
    }
    else 
      return("?");  
  }

  /**
   * @return an instance of the ColorSensor Subsystem
   */

  public static ColorSensor getInstance()
  {
    if (instance == null)
    {
      instance = new ColorSensor();
    }
    return instance;
  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new MySpecialCommand());
  }
}
