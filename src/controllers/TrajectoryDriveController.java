package controllers;

import subsystems.Drive;

import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryFollower;

import lib.Controller;
import utils.DaisyMath;
/**
 * TrajectoryDriveController.java
 * This controller drives the robot along a specified trajectory.
 * @author Tom Bottiglieri
 */
public class TrajectoryDriveController extends Controller {
	
	public static Drive drivebase = Drive.getInstance();

  public TrajectoryDriveController() {
    init();
  }
  Trajectory trajectory;
  TrajectoryFollower followerLeft = new TrajectoryFollower("left");
  TrajectoryFollower followerRight = new TrajectoryFollower("right");
  double direction;
  double heading;
  double kTurn = -3.0/80.0;

  public boolean onTarget() {
    return followerLeft.isFinishedTrajectory(); 
  }

  private void init() {
    followerLeft.configure(1.5, 0, 0, 1.0/12.0, 0.0*1.0/20.0);
    followerRight.configure(1.5, 0, 0, 1.0/12.0, 0.0*1.0/20.0);
  }

  public void loadProfile(Trajectory leftProfile, Trajectory rightProfile, double direction, double heading) {
    reset();
    followerLeft.setTrajectory(leftProfile);
    followerRight.setTrajectory(rightProfile);
    this.direction = direction;
    this.heading = heading;
  }
  
  public void loadProfileNoReset(Trajectory leftProfile, Trajectory rightProfile) {
    followerLeft.setTrajectory(leftProfile);
    followerRight.setTrajectory(rightProfile);
  }

  public void reset() {
    followerLeft.reset();
    followerRight.reset();
    drivebase.resetEncoders();
  }
  
  public int getFollowerCurrentSegment() {
    return followerLeft.getCurrentSegment();
  }
  
  public int getNumSegments() {
    return followerLeft.getNumSegments();
  }
  
  public void update() {
    if (!enabled) {
      return;
    }

    if (onTarget()) {
      drivebase.setSpeed(0.0, 0.0);
    } else  {
      double distanceL = direction * drivebase.getLeftEncoderDistance();
      double distanceR = direction * drivebase.getRightEncoderDistance();

      double speedLeft = direction * followerLeft.calculate(distanceL);
      double speedRight = direction * followerRight.calculate(distanceR);
      
      double goalHeading = followerLeft.getHeading();
      double observedHeading = drivebase.getGyroAngleInRadians();

      double angleDiffRads = DaisyMath.getDifferenceInAngleRadians(observedHeading, goalHeading);
      double angleDiff = Math.toDegrees(angleDiffRads);

      double turn = kTurn * angleDiff;
      drivebase.setSpeed(speedLeft + turn, speedRight - turn);
    }
  }

  public void setTrajectory(Trajectory t) {
    this.trajectory = t;
  }

  public double getGoal() {
    return 0;
  }
}
