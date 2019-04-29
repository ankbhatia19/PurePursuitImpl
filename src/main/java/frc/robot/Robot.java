/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
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

    private final Joystick leftJoy = new Joystick(0);
    private final Joystick rightJoy = new Joystick(1);

    private final XboxController controller = new XboxController(2);

    private final CheesyDriveHelper cheesyDrive = new CheesyDriveHelper();
    public static final Drivetrain drive = new Drivetrain();


    Waypoint[] points = new Waypoint[]{
            new Waypoint(-4, -1, Pathfinder.d2r(45)),
            new Waypoint(-2, -2, 0),
            new Waypoint(0, 0, 0)
    };

    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
    Trajectory trajectory = Pathfinder.generate(points, config);
    public PurePursuit pursuit = new PurePursuit(trajectory);
    TankModifier modifier = new TankModifier(trajectory).modify(Constants.wheelBaseWidth);

    EncoderFollower left = new EncoderFollower(modifier.getLeftTrajectory());
    EncoderFollower right = new EncoderFollower(modifier.getRightTrajectory());


    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override

    public void robotInit() {

        drive.getInstance();
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

        left.configureEncoder(drive.getTalon(Drivetrain.DrivetrainSide.left).getSensorCollection().getQuadraturePosition(), Constants.ticksPerRevolution, Constants.wheelRadius * 2);
        right.configureEncoder(drive.getTalon(Drivetrain.DrivetrainSide.right).getSensorCollection().getQuadraturePosition(), Constants.ticksPerRevolution, Constants.wheelRadius * 2);

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
        SmartDashboard.putNumber("Left hand", controller.getY(Hand.kLeft));
        SmartDashboard.putNumber("Right hand", controller.getX(Hand.kRight));
        //System.out.println(controller.getX(Hand.kRight));
        //System.out.println(controller.getY(Hand.kLeft));
        double[] powers = cheesyDrive.cheesyDrive(controller.getY(Hand.kLeft), controller.getX(Hand.kRight), false, false);
        SmartDashboard.putNumber("Left Power", powers[0]);
        SmartDashboard.putNumber("Right Power", powers[1]);
        SmartDashboard.putString("Odometry", drive.getOdo().toString());

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
        double l = left.calculate(drive.getTalon(Drivetrain.DrivetrainSide.left).getSensorCollection().getQuadraturePosition());
        double r = right.calculate(drive.getTalon(Drivetrain.DrivetrainSide.right).getSensorCollection().getQuadraturePosition());

        double gyro_heading = Math.toDegrees(drive.getOdo().getTheta());     // Assuming the gyro is giving a value in degrees
        double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees

        // This allows the angle difference to respect 'wrapping', where 360 and 0 are the same value
        double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
        angleDifference = angleDifference % 360.0;
        if (Math.abs(angleDifference) > 180.0) {
            angleDifference = (angleDifference > 0) ? angleDifference - 360 : angleDifference + 360;
        }

        double turn = 0.8 * (-1.0 / 80.0) * angleDifference;

        drive.setSpeeds(l + turn, r - turn);

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

        //controller.getX(Hand.kLeft);
        System.out.println(controller.getX(Hand.kRight));
        System.out.println(controller.getY(Hand.kLeft));
        double[] powers = cheesyDrive.cheesyDrive(controller.getY(Hand.kLeft), controller.getX(Hand.kRight), false, false);
        drive.setSpeeds(powers[0], powers[1]);
    }

    /**
     * This function is called periodically during test mode.
     */

    @Override
    public void testPeriodic() {

        double radius = 5;
        double[] powers = pursuit.getTargetVelocities(1 / radius);
        //leftTalon.set(ControlMode.Velocity, Units.feetToTicks(powers[0]));
        //rightTalon.set(ControlMode.Velocity, Units.feetToTicks(powers[1]));
        System.out.println("Left power: " + powers[0] + ", Right Power: " + powers[1]);

        drive.getTalon(Drivetrain.DrivetrainSide.left).set(ControlMode.Velocity, Units.FPSToTalonNative(powers[0]));
        drive.getTalon(Drivetrain.DrivetrainSide.right).set(ControlMode.Velocity, Units.FPSToTalonNative(powers[1]));
    }
}
