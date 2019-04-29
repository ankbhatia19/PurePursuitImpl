package frc.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import jaci.pathfinder.Trajectory;

public class AutoPurePursuit extends Command{

    private PurePursuit pursuit;
    private Notifier tracking;
    private volatile boolean isFinished;
    private volatile Timer time;
    private double now;

    public AutoPurePursuit(Trajectory traj){
        requires(Robot.drive.getInstance());
        pursuit = new PurePursuit(traj);


        tracking = new Notifier(() -> {
            if(!isFinished){
                isFinished = pursuit.isFinished();
                double myLeftVel = Robot.drive.getTalon(Drivetrain.DrivetrainSide.left).getSensorCollection().getQuadratureVelocity();
                double myRightVel = Robot.drive.getTalon(Drivetrain.DrivetrainSide.right).getSensorCollection().getQuadratureVelocity();
                double powers[] = pursuit.getNextVelocities(myLeftVel, myRightVel, Robot.drive.getOdo());
                Robot.drive.setFPS(powers[0], powers[1]);
                //Robot.drive.setFPS(pursuit.getNextDriveSignal(now).getLeft(), pursuit.getNextDriveSignal(now).getRight());
            }
        });
        tracking.startPeriodic(0.02);

    }

    protected boolean isFinished(){
        return pursuit.isFinished();
    }
}