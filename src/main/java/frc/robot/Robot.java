/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  TalonSRX leftTalon, rightTalon;
  AHRS gyro;

  Waypoint[] points = new Waypoint[]{
    new Waypoint(-4, -1, Pathfinder.d2r(45)),
    new Waypoint(-2, -2, 0),
    new Waypoint(0, 0, 0)
  };

  Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
  Trajectory trajectory = Pathfinder.generate(points, config);
  TankModifier modifier = new TankModifier(trajectory).modify(Constants.wheelBaseWidth);
  
  EncoderFollower left = new EncoderFollower(modifier.getLeftTrajectory());
  EncoderFollower right = new EncoderFollower(modifier.getRightTrajectory()); 

  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    gyro = new AHRS(SPI.Port.kMXP);
    leftTalon = new TalonSRX(2);
    rightTalon = new TalonSRX(3);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    left.configureEncoder(leftTalon.getSensorCollection().getQuadraturePosition(), Constants.ticksPerRevolution, Constants.wheelRadius * 2);
    right.configureEncoder(rightTalon.getSensorCollection().getQuadraturePosition(), Constants.ticksPerRevolution, Constants.wheelRadius * 2);

    right.configurePIDVA(1.0, 0.0, 0.0, 1 / Constants.maxVelocity, 0);
    left.configurePIDVA(1.0, 0.0, 0.0, 1 / Constants.maxVelocity, 0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    double l = left.calculate(leftTalon.getSensorCollection().getQuadraturePosition());
    double r = right.calculate(rightTalon.getSensorCollection().getQuadraturePosition());

    double gyro_heading = gyro.getYaw();     // Assuming the gyro is giving a value in degrees
    double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees

    // This allows the angle difference to respect 'wrapping', where 360 and 0 are the same value
    double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    angleDifference = angleDifference % 360.0;
    if (Math.abs(angleDifference) > 180.0) {
      angleDifference = (angleDifference > 0) ? angleDifference - 360 : angleDifference + 360;
    } 

    double turn = 0.8 * (-1.0/80.0) * angleDifference;

    leftTalon.set(ControlMode.PercentOutput, l + turn);
    rightTalon.set(ControlMode.PercentOutput, r - turn);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
