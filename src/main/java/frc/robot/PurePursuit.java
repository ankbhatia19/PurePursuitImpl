/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import jaci.pathfinder.*;
import jaci.pathfinder.Trajectory.Segment;

/**
 * Add your docs here.
 */
public class PurePursuit {

    private Trajectory path;
    private int indexCurrentPoint;
    private int indexLookaheadPoint;
    private Odometry odo;

    public PurePursuit(Trajectory path) {
        this.path = path;
        indexCurrentPoint = 0;
        indexLookaheadPoint = indexCurrentPoint + 1;
    }

    public double[] getNextVelocities(double leftVel, double rightVel, Odometry odo) {
        /* The algorithm to follow the path is as follows:
            ● Find the closest point
            ● Find the lookahead point
            ● Calculate the curvature of the arc to the lookahead point
            ● Calculate the target left and right wheel velocities
            ● Use a control loop to achieve the target left and right wheel velocities
        */
        Segment currentPoint = this.path.get(getCurrentPointIndex());
        Segment lookaheadPoint = this.path.get(getLookaheadPointIndex());
        double curvature = this.calculateCurvature(odo.getTheta(), currentPoint, lookaheadPoint);
        double[] targetVels = this.getTargetVelocities(curvature);
        double[] finishedVels = {0, 0};
        finishedVels[0] = calcFeedForward(targetVels[0]) + calcFeedBack(targetVels[0], leftVel);
        finishedVels[1] = calcFeedForward(targetVels[1]) + calcFeedBack(targetVels[1], rightVel);
        return finishedVels;

    }

    private int getCurrentPointIndex() {

        double closestDistance = Double.MAX_VALUE;
        int index = -1;
        for (int i = indexCurrentPoint; i < indexCurrentPoint + 6; i++) {
            double distanceFromPoint = Math.sqrt(Math.pow((path.get(indexCurrentPoint).x - odo.getX()), 2) + Math.pow((path.get(indexCurrentPoint).y - odo.getY()), 2));
            if (distanceFromPoint < closestDistance) {
                closestDistance = distanceFromPoint;
                index = i;
            }
        }
        indexCurrentPoint = index;
        return indexCurrentPoint;
    }

    private int getLookaheadPointIndex() {
        return indexLookaheadPoint + Constants.lookaheadDistance;
    }

    public double calculateCurvature(double robotAngle, Segment robot, Segment lookahead) {
        /* curvature = 2x/L^2 */
        /*
         * a = − tan(robot angle)
         * b = 1
         * c = tan(robot angle) * robot x − robot y The
         * point-line distance formula is: d = |ax + by + c| /sqrt(a^2 + b^2) Plugging
         * in our coefficients and the coordinates of the lookahead point gives:
         * x = |a * lookahead x + b * lookahead y + c| /sqrt(a^2 + b^2)
         */
        /*
         * side = signum(sin(robot angle) * (Lx - Rx) - cos(robot angle) * (Ly - Ry))
         * Signed curvature = curvature * sign
         */
        double a = -1 * Math.tan(robotAngle);
        double b = 1;
        double c = Math.tan(robotAngle) * robot.x - robot.y;
        double x = Math.abs(a * lookahead.x + b * lookahead.y + c) / Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
        double curvature = (2 * x) / ((Math.pow(lookahead.x - robot.x, 2) + Math.pow(lookahead.y - robot.y, 2)));
        double side = Math.signum((Math.sin(robotAngle) * (lookahead.x - robot.x)) - (Math.cos(robotAngle) * (lookahead.y - robot.y)));
        return (curvature * side);
    }

    public double[] getTargetVelocities(double curvature) {

        double[] velocities = {0, 0};

        double velocity = Constants.maxVelocity;
        //velocity = Units.feetToTicks(velocity); // V
        //curvature = Units.feetToTicks(curvature); // C
        //double wheelBaseWidth = Units.feetToTicks(Constants.wheelBaseWidth); // T
        double wheelBaseWidth = Constants.wheelBaseWidth;
        /*
         * Target wheel velocities are given by L = V * (2 + CT)/2; R = V * (2 - CT)/2
         */

        /* Left Wheel */
        double targetLeftVelocity = velocity * (2 + (curvature * wheelBaseWidth) / 2);
        velocities[0] = targetLeftVelocity;

        /* Right Wheel */
        double targetRightVelocity = velocity * (2 - (curvature * wheelBaseWidth) / 2);
        velocities[1] = targetRightVelocity;

        return velocities;
    }

    private double calcFeedForward(double targetVel) {
        return Constants.kV * targetVel;
    }

    private double calcFeedBack(double targetVel, double measuredVel) {
        return Constants.kP * (targetVel - measuredVel);
    }

    public boolean isFinished() {
        if (indexCurrentPoint == (path.length() - 1)) {
            return true;
        }
        return false;
    }

}
