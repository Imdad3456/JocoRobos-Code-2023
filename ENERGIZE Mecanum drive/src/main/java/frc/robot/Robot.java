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
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C;

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
  // VictorSP motor = new VictorSP(0);
  XboxController Controller = new XboxController(0);
  private final Timer m_timer = new Timer();

  // DigitalInput limitSwitch = new DigitalInput(5);
  // DigitalInput limitSwitch2 = new DigitalInput(6);
  LinearServo lActuator = new LinearServo(3, 150, 60);

  VictorSP motor = new VictorSP(0);
  VictorSP motor1 = new VictorSP(1);
  VictorSP motor2 = new VictorSP(2);
  WPI_VictorSPX motorFrontLeft = new WPI_VictorSPX(1);
  WPI_VictorSPX motorFrontRight = new WPI_VictorSPX(2);
  WPI_VictorSPX motorBackLeft = new WPI_VictorSPX(3);
  WPI_VictorSPX motorBackRight = new WPI_VictorSPX(4);
  // LinearServo LiftMotor = new LinearServo(0, 152, 100);

  Encoder myEncoder = new Encoder(0, 1);
  PIDController myPID;

  double allowableError = 10;

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private static final int MPU6050_ADDR = 0x68;
  private static final int MPU6050_PWR_MGMT_1 = 0x6B;
  private static final int MPU6050_RESET = 0x80;
  private static final int MPU6050_CLOCK_PLL_XGYRO = 0x01;
  private static final int MPU6050_SMPLRT_DIV = 0x19;
  private static final int MPU6050_SMPLRT_DIV_VAL = 0x07;
  private static final int MPU6050_CONFIG = 0x1A;
  private static final int MPU6050_DLPF_CFG_0 = 0x00;
  private static final int MPU6050_GYRO_CONFIG = 0x1B;
  private static final int MPU6050_GYRO_FS_SEL_2000 = 0x18;
  private static final int MPU6050_ACCEL_CONFIG = 0x1C;
  private static final int MPU6050_ACCEL_FS_SEL_16 = 0x18;

  private I2C i2c;

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

    i2c = new I2C(I2C.Port.kOnboard, MPU6050_ADDR);

    i2c.write(MPU6050_PWR_MGMT_1, MPU6050_RESET);
    Timer.delay(0.1);
    i2c.write(MPU6050_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO);
    i2c.write(MPU6050_SMPLRT_DIV, MPU6050_SMPLRT_DIV_VAL);
    i2c.write(MPU6050_CONFIG, MPU6050_DLPF_CFG_0);
    i2c.write(MPU6050_GYRO_CONFIG, MPU6050_GYRO_FS_SEL_2000);
    i2c.write(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_FS_SEL_16);
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
    if (m_timer.get() < 2000.0) {

      motorFrontLeft.set(.3);
      motorBackLeft.set(.3);
      motorFrontRight.set(-.3);
      motorBackRight.set(-.3);
      // Drive forwards half speed, make sure to turn input squaring off

    } else {
      motorFrontLeft.stopMotor();
      motorBackLeft.stopMotor();
      motorFrontRight.stopMotor();
      motorBackRight.stopMotor();
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    myEncoder.setDistancePerPulse(2. / 64.);
    myEncoder.setReverseDirection(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double rx = Controller.getRightX(); // Remember, this is reversed!
    double x = Controller.getLeftX();
    double y = -Controller.getLeftY();

    double denominator = 1;// Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    double frontLeftPower = (y + x + rx) / denominator;
    double backLeftPower = (y - x + rx) / denominator;
    double frontRightPower = (y - x - rx) / denominator;
    double backRightPower = (y + x - rx) / denominator;

    motorFrontLeft.set(frontLeftPower * (18.0 / 16.72));
    motorBackLeft.set(backLeftPower * (16.72 / 16.72));
    motorFrontRight.set(frontRightPower * (19.76 / 16.72));
    motorBackRight.set(backRightPower * (17.71 / 16.72));

    if (Controller.getXButton() == true && myEncoder.getDistance() / 365 < 5.) {
      motor.set(.3);

    } else if (Controller.getBButton() == true && myEncoder.getDistance() > 0.) {
      motor.set(-.3);

    } else {

      motor.set(0);

    }

    /*
     * if (Controller.getYButton() == true) {
     * LiftMotor.setSpeed(1);
     * } else if (Controller.getAButton() == true) {
     * LiftMotor.setSpeed(-1);
     * } else {
     * LiftMotor.setSpeed(0);
     * }
     */

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

  /** This function is called periodically during test mode. */

}
