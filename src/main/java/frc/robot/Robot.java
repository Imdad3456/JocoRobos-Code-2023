// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;

/*
 import edu.wpi.first.wpilibj.motorcontrol.Victor;
 import com.ctre.phoenix.motorcontrol.ControlMode;
*/

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
  VictorSP lActuator = new VictorSP(3);
  VictorSP motor2 = new VictorSP(1);
  Servo Claw = new Servo(5);
  XboxController Controller = new XboxController(0);
  private final Timer m_timer = new Timer();

  WPI_VictorSPX motorFrontLeft = new WPI_VictorSPX(1);
  WPI_VictorSPX motorFrontRight = new WPI_VictorSPX(2);
  WPI_VictorSPX motorBackLeft = new WPI_VictorSPX(3);
  WPI_VictorSPX motorBackRight = new WPI_VictorSPX(4);
  MotorControllerGroup right_Motor_Group = new MotorControllerGroup(motorBackRight, motorFrontRight);
  MotorControllerGroup left_Motor_Group = new MotorControllerGroup(motorBackLeft, motorFrontLeft);

  DifferentialDrive drive = new DifferentialDrive(left_Motor_Group, right_Motor_Group);

  Encoder myEncoder = new Encoder(7, 8);
  PIDController myPID;

  double allowableError = 10;

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // motor controller on port 0
    myPID = new PIDController(1, 1, 1); // setup a PID controller with an encoder source and motor output

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
    System.out.println(m_timer.get());
    while (m_timer.get() < .5) {

      drive.arcadeDrive(-.8, 0);
      // Drive forwards half speed, make sure to turn input squaring off
    }
    while (m_timer.get() < 1) {

      drive.arcadeDrive(0, 0);
      // Drive forwards half speed, make sure to turn input squaring off
    }
    while (m_timer.get() < 3) {

      drive.arcadeDrive(.6, 0);
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
    System.out.println(MPU6050.readGyro("x"));

    //double triggerValue = Controller.getRawAxis(3);
    double Left_y = -Controller.getLeftY();
    double Right_y = -Controller.getLeftX();

    double scale = 1.0;
    if (Controller.getRawAxis(3) > 0.1) {
      // Right trigger is pressed, set motor speeds to 50%
      scale = 0.5;
    } else {
      scale = 1.0;
    }

    // arcade drive code
    double speed = Left_y * scale;
    double turn = Right_y * scale;
    // CHANGE THE X VALUE TO THE AXIS FOR RIGHT JOYSTICK X AXIS

    double left = speed + turn;
    double right = speed - turn;

    motorFrontLeft.set(left);
    motorBackLeft.set(left);
    motorFrontRight.set(-right);
    motorBackRight.set(-right);

    Claw.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    if (Controller.getXButton() == true && myEncoder.getDistance() / 360 < 2.) {
      motor2.set(-.4);
    } else if (Controller.getBButton() == true) // && myEncoder.getDistance() / 360 > 2.)
    {
      motor2.set(.4);
    } else {
      motor2.set(0);
    }

    // Linear
    if (Controller.getYButton() == true) {
      lActuator.set(-1);
    } else if (Controller.getAButton() == true) {
      lActuator.set(1);
    } else {
      lActuator.set(0);
    }

    // Claw controls with bumpers
    if (Controller.getLeftBumper() == true) {
      Claw.set(1);
    } else {
      Claw.set(0);
    }

    double distancetraveled = myEncoder.getDistance() / 360;
    System.out.println(distancetraveled);

    // read the PID values from the SmartDashboard and update the PID controller
    myPID.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));

    // update setpoint to whatever has been entered on smartdashboard
    myPID.setSetpoint(SmartDashboard.getNumber("Distance Setpoint", 0));

    // update the allowable error from the SmartDashboard
    allowableError = SmartDashboard.getNumber("Allowable Error: ", 0);

    SmartDashboard.putNumber("Encoder Get: ", myEncoder.get()); // show encoder reading on the dashboard

    // We know we are on target when the difference in our setpoint and
    // encoder value are less than our determined allowable error
    SmartDashboard.putBoolean("On Target: ", Math.abs(myPID.getSetpoint() - myEncoder.get()) < allowableError);
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