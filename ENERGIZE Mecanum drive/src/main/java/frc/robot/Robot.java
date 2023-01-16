// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  

  //initilizes all the motors to their respective pwm ports
  VictorSP front_Right = new VictorSP(1);
  VictorSP front_Left = new VictorSP(2);
  VictorSP back_Right = new VictorSP(3);
  VictorSP back_Left = new VictorSP(4);

  VictorSP lift_motor = new VictorSP(5);
  VictorSP Elevator = new VictorSP(6);
  VictorSP claw = new VictorSP(7);
  
  //initilizes the joystick
  Joystick stick = new Joystick(1);
  



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //takes the axis from the joystick and puts them in a variable. also z and y are swapped for controller issues
    double stickx = stick.getX();
    double stickz = -stick.getY();
    double sticky = stick.getZ();



    
    //does the math to determine which motor should move and what power
    double denominator = Math.max(Math.abs(sticky) + Math.abs(stickx) + Math.abs(stickz), 1);
    double front_Left_Power = (sticky + stickx + stickz) / denominator;
    double back_Left_Power = (sticky - stickx + stickz) / denominator;
    double front_Right_Power = (sticky - stickx - stickz) / denominator;
    double back_Right_Power = (sticky + stickx - stickz) / denominator;


    //takes the powers and sets the motors to match it on the robot
    front_Right.set(front_Right_Power);
    front_Left.set(front_Left_Power);
    back_Left.set(back_Left_Power);
    back_Right.set(back_Right_Power); 


    //Controls the lift motor using if statements
    if (stick.getRawButton(1) ){
      lift_motor.set(-.3);

    }else if (stick.getRawButton(2) ){
     lift_motor.set(.2);
    }else {
     lift_motor.set(0);
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
