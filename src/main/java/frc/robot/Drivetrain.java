package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Drivetrain extends Subsystem {

    private TalonSRX leftTalon;
    private TalonSRX rightTalon;

    private AHRS gyro;

    private double x, y, theta;
    private Drivetrain instance;
    private double lastPos, currentPos;

    public Drivetrain() {
        gyro = new AHRS(SPI.Port.kMXP);
        leftTalon = new TalonSRX(2);
        rightTalon = new TalonSRX(3);
        rightTalon.setInverted(true);
        leftTalon.setInverted(false);

        lastPos = (leftTalon.getSelectedSensorPosition(0) + rightTalon.getSelectedSensorPosition(0)) / 2;

        Notifier odoThread = new Notifier(() -> {
            currentPos = (leftTalon.getSelectedSensorPosition(0) + rightTalon.getSelectedSensorPosition(0)) / 2;
            double dPos = Units.feetToTicks(currentPos - lastPos);
            theta = Math.toRadians(gyro.getAngle());
            x += Math.cos(theta) * dPos;
            y += Math.sin(theta) * dPos;
            lastPos = currentPos;
        });

        odoThread.startPeriodic(0.01);

    }

    public Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;

    }

    public enum DrivetrainSide {
        left, right;
    }

    public TalonSRX getTalon(DrivetrainSide side){
        if (side.equals(DrivetrainSide.left)){
            return leftTalon;
        }
        else if (side.equals(DrivetrainSide.right)){
            return rightTalon;
        }
        else{
            return null;
        }
    }

    public void setSpeeds(double left, double right) {
        leftTalon.set(ControlMode.PercentOutput, left);
        rightTalon.set(ControlMode.PercentOutput, right);
    }

    public void setFPS(double left, double right) {
        /*
        double actualLeft = Units.TalonNativeToFPS(talonLeftA.getSelectedSensorVelocity(0));
        double actualRight = Units.TalonNativeToFPS(talonRightA.getSelectedSensorVelocity(0));
        if ( actualLeft > LOW_GEAR_TOP_SPEED-2 || actualRight > LOW_GEAR_TOP_SPEED-2) {
            setTransmissionHigh();
        }

        if (actualLeft < LOW_GEAR_TOP_SPEED - 4 && actualRight < LOW_GEAR_TOP_SPEED - 4) {
            setTransmissionLow();
        }*/
        //System.out.println("wanted " + left + " " + right);
        //System.out.println("real " + Units.TalonNativeToFPS(talonLeftA.getSelectedSensorVelocity(0)) + " " + Units.TalonNativeToFPS(talonRightA.getSelectedSensorVelocity(0)));
        leftTalon.set(ControlMode.Velocity, Units.FPSToTalonNative(left));
        rightTalon.set(ControlMode.Velocity, Units.FPSToTalonNative(right));
    }

    public Odometry getOdo() {
        return new Odometry(x, y, theta);
    }

    @Override
    protected void initDefaultCommand() {

    }

}
