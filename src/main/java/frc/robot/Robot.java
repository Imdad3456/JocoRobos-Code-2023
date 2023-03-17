// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.Servo;


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

  private final static byte MPU6050_ADDRESS = 0x68;
    private final static int REGISTER_PWR_MGMT_1 = 0x6B;
    private final static int REGISTER_GYRO = 0x43;

    
    private byte[] buffer = new byte[6];

    Accelerometer accelerometer = new BuiltInAccelerometer();
  

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
  MPU6050Sensor accel = new MPU6050Sensor();
 
  AnalogGyro gyro = new AnalogGyro(0);
  

  Encoder myEncoder = new Encoder(7, 8);
  PIDController myPID;
  AddressableLED m_led = new AddressableLED(0);

  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(120);
  double allowableError = 10;

  double heading;

  DifferentialDrive drive = new DifferentialDrive(right_Motor_Group, left_Motor_Group);

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

 

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
     // Take the accelerometer out of sleep mode
    
   

    // motor controller on port 0
    myPID = new PIDController(1, 1, 1); // setup a PID controller with an encoder source and motor output


      // PWM port 9
    // Must be a PWM header, not MXP or DIO
     
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    SmartDashboard.putNumber("P", .01);
    SmartDashboard.putNumber("I", 0);
    SmartDashboard.putNumber("D", 0);
    SmartDashboard.putNumber("Distance Setpoint", 0);
    SmartDashboard.putNumber("Allowable Error: ", allowableError);

    myPID.setSetpoint(0);

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

      // Gets the current accelerations in the X and Y directions
      
   
  

      
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 255, 0, 0);
   }
   
   m_led.setData(m_ledBuffer);

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
     
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    

    


    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

      double yAccel = accelerometer.getY();
      double zAccel = accelerometer.getZ();
  
      // Calculates the jerk in the X and Y directions
      // Divides by .02 because default loop timing is 20ms
      
  
    
     


    
    while (m_timer.get() < 1) {
     
      double xAccel = accelerometer.getX();
      motorFrontLeft.set(.25);
      motorBackLeft.set(.25);
      motorFrontRight.set(-.25);
      motorBackRight.set(-.25);
      System.out.println(xAccel);
  
      // Drive forwards half speed, make sure to turn input squaring off
    }
    while (m_timer.get() < 2) {
      double xAccel = accelerometer.getX();
      
      motorFrontLeft.set(0);
      motorBackLeft.set(0);
      motorFrontRight.set(0);
      motorBackRight.set(0);
      System.out.println(xAccel);
  
      // Drive forwards half speed, make sure to turn input squaring off
    }
    while (m_timer.get() < 4) {
      double xAccel = accelerometer.getX();

      motorFrontLeft.set(-.5);
      motorBackLeft.set(-.5);
      motorFrontRight.set(.5);
      motorBackRight.set(.5);
      System.out.println(xAccel);
  
      // Drive forwards half speed, make sure to turn input squaring off
    }
    while (m_timer.get() < 10) {
     
      while (accelerometer.getX() > 0 ){
        double xAccel = accelerometer.getX();
        motorFrontLeft.set(-.2);
        motorBackLeft.set(-.2);
        motorFrontRight.set(.2);
        motorBackRight.set(.2);
        System.out.println(xAccel);
    
      }
      
      // Drive forwards half speed, make sure to turn input squaring off
    }
  }


  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    myEncoder.reset();
    myEncoder.setDistancePerPulse(1. / 64.);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Read the gyro measurements (6 bytes)
    




   
    

   
    double axis_z = Controller.getRawAxis(2);
    double  axis_y= -Controller.getRawAxis(1);
    double speeds = convertToZeroPointOneToOne(-Controller.getRawAxis(3));

   

    // arcade drive code
    double speed = axis_y * speeds;
    double turn = axis_z * speeds;
    // CHANGE THE X VALUE TO THE AXIS FOR RIGHT JOYSTICK X AXIS

    double left = speed + turn;
    double right = speed - turn;

    motorFrontLeft.set(left);
    motorBackLeft.set(left);
    motorFrontRight.set(-right);
    motorBackRight.set(-right);

    

    if (Controller.getRawButton(3) == true && myEncoder.getDistance() / 360 < 2.) {
      motor2.set(-.4);
    } else if (Controller.getRawButton(4) == true) // && myEncoder.getDistance() / 360 > 2.)
    {
      motor2.set(.4);
    } else {
      motor2.set(0);
    }

    // Linear
    lActuator.set(Controller2.getRawAxis(1));

    // Claw controls with bumpers
    
    if (Controller.getRawButton(1) == true) {
      Claw.set(1);
      System.out.println(1);
    } else  if (Controller.getRawButton(2) == true){
      Claw.set(-.7);
    }else{
      Claw.set(0);}


    
    /*
     * double distancetraveled = myEncoder.getDistance() / 360;
     * System.out.println(distancetraveled);
     */


    // We know we are on target when the difference in our setpoint and
    // encoder value are less than our determined allowable error

  }
  /** Adapted from {@link ADXL345_I2C#accelFromBytes} */


  private double convertToZeroPointOneToOne(double value) {
    return (value + 1) / 2 * 0.9 + 0.1;
  }
  private double convertToZeroPointOneT(double value) {
    return (value + 1) / 2;
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

  

public class MPU6050Sensor implements Accelerometer {
    private static final int MPU6050_ADDRESS = 0x68;
    private static final int MPU6050_ACCEL_XOUT_H = 0x3B;
    private static final int MPU6050_PWR_MGMT_1 = 0x6B;

    private I2C i2c;
    private byte[] buffer;

    public MPU6050Sensor() {
        i2c = new I2C(I2C.Port.kOnboard, MPU6050_ADDRESS);
        buffer = new byte[6];
        i2c.write(MPU6050_PWR_MGMT_1, 0);
    }

    public double getX() {
        i2c.read(MPU6050_ACCEL_XOUT_H, 6, buffer);
        int x = (buffer[0] << 8) | (buffer[1] & 0xFF);
        return x / 16384.0;
    }

    public double getY() {
        i2c.read(MPU6050_ACCEL_XOUT_H + 2, 6, buffer);
        int y = (buffer[2] << 8) | (buffer[3] & 0xFF);
        return y / 16384.0;
    }

    public double getZ() {
        i2c.read(MPU6050_ACCEL_XOUT_H + 4, 6, buffer);
        int z = (buffer[4] << 8) | (buffer[5] & 0xFF);
        return z / 16384.0;
    }

    public void setRange(Range range) {
        // Not supported
    }

    public void setZero(double zero) {
        // Not supported
    }
}
  

}