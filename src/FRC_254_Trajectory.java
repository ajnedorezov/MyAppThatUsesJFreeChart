import subsystems.Drive;

import com.team254.lib.trajectory.*;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator.Config;
import com.team254.lib.trajectory.TrajectoryGenerator.Strategy;

import controllers.TrajectoryDriveController;

/**
 * Test script to walk through and visualize the trajectory following
 * logic implemented by 254
 *
 * @author AJN
 */

public class FRC_254_Trajectory {

	private static Drive mDrive;
	private static TrajectoryFollower mDriveFollower;
	private static TrajectoryDriveController mTrajController;
	
	private static double timeStep = 0.005;
	
    public static void main(String[] args) {
        
    	// Grab the drive base simulator
    	mDrive = Drive.getInstance();
    	
    	// Set the robot dynamics
    	Config config = new Config();
    	config.dt = timeStep;
    	config.max_vel = 12; 	//fps
    	config.max_acc = 20; 	//fps^2
    	config.max_jerk = 40; 	//fps^3
    	
    	// Choose the acceleration profile
    	Strategy strategy = TrajectoryGenerator.TrapezoidalStrategy;
    	//Strategy strategy = TrajectoryGenerator.SCurvesStrategy;
    	
    	// Define the starting config of the robot
    	double start_vel = 0.0;			//fps
    	double start_heading = 0.0; 	//deg
    	
    	// Define the goal config of the robot
    	double goal_pos = 24; 			// inches
    	double goal_vel = 0.0; 			//fps
    	double goal_heading = 20.0 * Math.PI / 180.0; 	//deg
    	
    	// Generate the profile to be traveled
    	Trajectory traj = TrajectoryGenerator.generate(config, strategy, start_vel, start_heading, goal_pos, goal_vel, goal_heading);
    	
    	System.out.println("# of segments " + traj.getNumSegments());
    	System.out.println(traj.toStringProfile());
    	
    	// Initialize the Trajectory Follower
    	mDriveFollower = new TrajectoryFollower("Drive");
    	
    	// Set the controller gains
    	double kp = 1.5;
    	double ki = 0.0;
    	double kd = 0.0;
    	double kv = 1/config.max_vel;
    	double ka = 0.0*1/config.max_acc;
    	
    	mDriveFollower.configure(kp, ki, kd, kv, ka);
    	
    	// Provide the trajectory to the follower
    	mDriveFollower.setTrajectory(traj);
    	mDriveFollower.reset();
    	
    	// Simulate the following of the path using just the trajectory follower
    	mDrive.resetEncoders();
    	double distTraveled = mDrive.getAverageDistance(); 
    	for (int t = 0; t <= traj.getNumSegments(); t++){
    		
    		double command = mDriveFollower.calculate(distTraveled);
    		double turn = 0.0;
    		mDrive.setSpeedTurn(command, turn);
    		
    		// Have the simulated drive propagate based on the issued commands
    		mDrive.step(timeStep);
    		distTraveled = mDrive.getAverageDistance();
    	}
    	System.out.println("");
    	mDrive.reinit();
    	//mDrive.invertLeftMotor();
    	    	
    	// Now simulate using a trajectory drive controller instead, start by creating the controller
    	mTrajController = new TrajectoryDriveController();
    	
    	// Load the profiles into the controller
    	mTrajController.loadProfile(traj, traj, 1.0, mDrive.getGyroAngleInRadians());
    	mTrajController.enable();
    	
    	double t = 0.0;
    	while (!mTrajController.onTarget()){
    		mTrajController.update();
    		mDrive.step(timeStep);
    		System.out.println("Robot Heading: " + mDrive.getGyroAngleInDegrees());
    		
    		t += timeStep;
    		System.out.println("Time: " + t);
    		
    		if (t > 100){
    			break;
    		}
    	}
    	
    }
}