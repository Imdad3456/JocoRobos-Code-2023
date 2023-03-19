// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.PIDController;


import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  AHRS ahrs;
  boolean autoBalanceXMode;
  boolean autoBalanceYMode;

  Accelerometer accelerometer = new BuiltInAccelerometer();
  DutyCycleEncoder myEncoder = new DutyCycleEncoder(7);
  //Encoder encoder = new Encoder(7, 8);

  AHRS gyro = new AHRS(SPI.Port.kMXP);
  
  
  static final double kOffBalanceAngleThresholdDegrees = 10;
  static final double kOonBalanceAngleThresholdDegrees  = 5;
  double pitchAngleDegrees    = gyro.getPitch();
  double rollAngleDegrees     = gyro.getRoll();

  VictorSP lActuator = new VictorSP(9);
  VictorSP motor2 = new VictorSP(8);
  VictorSP Claw = new VictorSP(7);
  Joystick Controller = new Joystick(0);
  Joystick Controller2 = new Joystick(1);

 
  private final Timer m_timer = new Timer();

  double prevXAccel = 0;
  double prevYAccel = 0;

  WPI_VictorSPX motorFrontLeft = new WPI_VictorSPX(1);
  WPI_VictorSPX motorFrontRight = new WPI_VictorSPX(2);
  WPI_VictorSPX motorBackLeft = new WPI_VictorSPX(3);
  WPI_VictorSPX motorBackRight = new WPI_VictorSPX(4);
  
  MotorControllerGroup right_Motor_Group = new MotorControllerGroup(motorBackRight, motorFrontRight);
  MotorControllerGroup left_Motor_Group = new MotorControllerGroup(motorBackLeft, motorFrontLeft);

  PIDController myPID;
  AddressableLED m_led = new AddressableLED(0);

  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(120);
  double allowableError = 10;

  double heading;

  DifferentialDrive drive = new DifferentialDrive(right_Motor_Group, left_Motor_Group);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
     
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    double angle = gyro.getRoll();

    System.out.println(angle);

      
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 255, 0, 0);
   }
   
   m_led.setData(m_ledBuffer);

  }
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    gyro.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

      double yAccel = accelerometer.getY();
      double angle;
      boolean onChargeStation = false;
      boolean finished = false;
      double zAccel = accelerometer.getZ();

    
    while (m_timer.get() < 7) {
     
      double xAccel = accelerometer.getX();
      drive.arcadeDrive(0, -.5);
      System.out.println(xAccel);
  
    }}/* 
    while (m_timer.get() < 7) {
      double xAccel = accelerometer.getX();
      
      drive.arcadeDrive(0, -.5); 
      
  
      
    }}*//* 
    while (m_timer.get() < 4.45) {
      double xAccel = accelerometer.getX();

      drive.arcadeDrive(0, -.6);
      System.out.println(xAccel);
  
    }
    angle = 0;
    while (m_timer.get() < 6 && angle > -10) {
       angle = gyro.getRoll();
      //angle = accelerometer.getX();
      drive.arcadeDrive(0, -.45);
      System.out.println(angle);
      
    } 
    double leveltime = m_timer.get();
    while (m_timer.get() < (leveltime + .05)){
      drive.arcadeDrive(0, .4);
    }
    
    drive.arcadeDrive(0, 0);
  }*/


  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    myEncoder.reset();
    //encoder.reset();
   // encoder.setDistancePerPulse(4.0);
    myEncoder.setDistancePerRotation(4.0/256.0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    System.out.println(myEncoder.getDistance());
   
   
    double axis_z = Controller.getRawAxis(1);
    double axis_y= -Controller.getRawAxis(2);
    double speeds = convertToZeroPointOneToOne(-Controller.getRawAxis(3));

   

    // arcade drive code
    double speed = axis_y * speeds;
    double turn = axis_z * speeds;
    // CHANGE THE X VALUE TO THE AXIS FOR RIGHT JOYSTICK X AXIS

  

    drive.arcadeDrive(-speed, -turn);
    

    if (Controller2.getRawButton(3) == true && myEncoder.getDistance() < 2.) {
      motor2.set(-.4);
    } else if (Controller2.getRawButton(4) == true) // && myEncoder.getDistance() / 360 > 2.)
    {
      motor2.set(.4);
    } else {
      motor2.set(0);
    }

    // Linear
    lActuator.set(-Controller2.getRawAxis(1));

    // Claw controls with bumpers
    
    if (Controller2.getRawButton(1) == true /*&& myEncoder.getDistance() < 2 */) {
      Claw.set(.7);
      System.out.println(1);
    } else  if (Controller2.getRawButton(2) == true){
      Claw.set(-1);
    }else{
      Claw.set(0);}
    /*
     * double distancetraveled = myEncoder.getDistance() / 360;
     * System.out.println(distancetraveled);
     */

  }

  private double convertToZeroPointOneToOne(double value) {
    return (value + 1) / 2 * 0.9 + 0.1;
  }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }
  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

 
}