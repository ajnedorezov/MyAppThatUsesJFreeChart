package utils;

import subsystems.Drive;
import utils.DaisyMath;
import utils.Trajectory;
import utils.TrajectoryFollower;

/**
 *
 * @author Jared
 */
public class DriveDistanceController
{
    private Drive mDrive = Drive.getInstance();
    private TrajectoryFollower mFollower;
    private double kTurn;
    private double distanceThreshold;
    private double heading;
    private double direction;
    
    private double mLastSpeed = 0.0;
    
    private static DriveDistanceController instance = null;
    
    public static DriveDistanceController getInstance()
    {
        if( instance == null )
        {
            instance = new DriveDistanceController(0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0);
        }
        return instance;
    }
    
    public DriveDistanceController(double kp, double ki, double kd, double kv, double ka, double distThresh, double turnP)
    {
        mFollower = new TrajectoryFollower();
        loadProperties(kp, ki, kd, kv, ka, distThresh, turnP);
    }
    
    public void loadProfile(Trajectory profile, double direction, double heading)
    {
        reset();
        mFollower.setTrajectory(profile);        
        this.direction = direction;
        this.heading = heading;
    }

    public void reset() 
    {
        //loadProperties();
        mFollower.reset();
        mDrive.resetEncoders();
    }

    public boolean onTarget() 
    {
        return mFollower.isFinishedTrajectory();// && mFollower.onTarget(distanceThreshold);
    }

    public final void loadProperties(double kp, double ki, double kd, double kv, double ka, double distThresh, double turnP) 
    {
        mFollower.configure(kp, ki, kd, kv, ka);
        
        distanceThreshold = distThresh;
        kTurn = turnP;
    }

    public void run() 
    {
        //System.out.println(this.onTarget() + " " + mFollower.isFinishedTrajectory() + " " + mFollower.onTarget(1.0));
    	
        if( onTarget() )
        {
            mDrive.setSpeed(0.0, 0.0);
            
            mLastSpeed = 0.0;
        }
        else
        {
            double distance = direction*mDrive.getAverageDistance();
            double angleDiff = DaisyMath.boundAngleNeg180to180Degrees(heading-mDrive.getGyroAngleInDegrees());
            
            double speed = direction*mFollower.calculate(distance);
            double turn = kTurn*angleDiff;
            mDrive.setSpeedTurn(speed, turn);
            
            mLastSpeed = speed;
        }
    }

    public double getFollowError(){
    	return mFollower.getLastError();
    }
    
    public double getLastSpeedCommand(){
    	return mLastSpeed;
    }
    
}
