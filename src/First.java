import java.awt.BorderLayout;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import subsystems.Drive;
import utils.DriveDistanceController;
import utils.Trajectory;
import utils.TrajectoryFollower;

/**
 * This program demonstrates how to draw XY line chart with XYDataset
 * using JFreechart library.
 * @author www.codejava.net
 *
 */
public class First extends JFrame {
	
 
    public First() {
        super("XY Line Chart Example with JFreechart");
 
        JPanel chartPanel = createChartPanel();
        add(chartPanel, BorderLayout.CENTER);
 
        setSize(640, 480);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLocationRelativeTo(null);
    }
 
    private JPanel createChartPanel() {
    	String chartTitle = "Trajectory Planning";
        String xAxisLabel = "Time";
        String yAxisLabel = "Y";
     
        // Create a trajectory
        Trajectory newTraj = new Trajectory();
        double loopPeriod = (double) 10L/1000.0;
        newTraj.generate(15, 10, 20, 40, loopPeriod);
        
        // Just plot the trajectory info
    	XYSeriesCollection dataset = createDataset(loopPeriod, newTraj);
        
        //int followType = 0; // Show Trajectory
        //int followType = 1; // Trajectory Follower 
        int followType = 2; // Drive Controller
        if (followType == 1){
        
	        // Follow the trajectory with the trajectory follower
	        TrajectoryFollower newFollower = new TrajectoryFollower();
	        newFollower.configure(1.0, 0.0, 0.0, 0.1, 0.0);
	        newFollower.setTrajectory(newTraj);
	        newFollower.reset();
	        
	        double distTraveled = 0.0;
	        double cmd = 0.0;
	        XYSeries commands = new XYSeries("Command");
	        XYSeries dist = new XYSeries("Traveled Dist");
	        XYSeries err = new XYSeries("Error");
	        for (int j=0; j<newTraj.getNumSegments(); j++){
	        	distTraveled += cmd*12.0*loopPeriod;
		        cmd = newFollower.calculate(distTraveled);
		        boolean isFinished = newFollower.isFinishedTrajectory();
		        boolean onTarget = newFollower.onTarget(0.5); 
		        
		        commands.add(j*loopPeriod, cmd);
		        dist.add(j*loopPeriod, distTraveled);
		        err.add(j*loopPeriod, newFollower.getLastError());
		        
		        System.out.println("PID:" + cmd + ", isFinished: " + isFinished + ", onTarget: " + onTarget);
		        
		        if (isFinished && onTarget){
		        	break;
		        }
	        }
	        dataset = new XYSeriesCollection();
	        dataset.addSeries(commands);
	        dataset.addSeries(dist);
	        dataset.addSeries(err);
        }else if(followType == 2){
        	// Initialize the drive
        	Drive mDrive = Drive.getInstance();
        	DriveDistanceController mDriveController = new DriveDistanceController(0.5, 0.9, 0.0, 0.1, 0.0, 0.0, 0.0);
        	mDriveController.loadProfile(newTraj, 1.0, 0.0);
        	
	        double cmd = 0.0;
        	XYSeries commands = new XYSeries("Command");
	        XYSeries dist = new XYSeries("Traveled Dist");
	        XYSeries err = new XYSeries("Error");
	        for (int j=0; j<newTraj.getNumSegments(); j++){
		        mDriveController.run();
		        mDrive.step(loopPeriod);
		        boolean onTarget = mDriveController.onTarget(); 
		        
		        commands.add(j*loopPeriod, mDriveController.getLastSpeedCommand());
		        dist.add(j*loopPeriod, mDrive.getAverageDistance());
		        err.add(j*loopPeriod, mDriveController.getFollowError());
		        
		        System.out.println("PID:" + cmd + ", onTarget: " + onTarget);
		        
		        if (onTarget){
		        	break;
		        }
	        }
        	
        	
        	dataset = new XYSeriesCollection();
	        dataset.addSeries(commands);
	        dataset.addSeries(dist);
	        dataset.addSeries(err);
        }
        
     
        JFreeChart chart = ChartFactory.createXYLineChart(chartTitle,
                xAxisLabel, yAxisLabel, dataset);
     
        return new ChartPanel(chart);
    }
 
    private XYSeriesCollection createDataset(double loopPeriod, Trajectory newTraj) {
    	XYSeriesCollection dataset = new XYSeriesCollection();
        XYSeries pos = new XYSeries("Position");
        XYSeries vel = new XYSeries("Velocity");
        XYSeries acc = new XYSeries("Acceleration");
        
        for(int j=0; j<newTraj.getNumSegments(); j++){
        	pos.add(j*loopPeriod, newTraj.getSegment(j).position);
        	vel.add(j*loopPeriod, newTraj.getSegment(j).velocity);
        	acc.add(j*loopPeriod, newTraj.getSegment(j).acceleration);
        }
     
        dataset.addSeries(pos);
        dataset.addSeries(vel);
        dataset.addSeries(acc);
     
        return dataset;
    }
 
    public static void main(String[] args) {
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                new First().setVisible(true);
            }
        });
    }
}