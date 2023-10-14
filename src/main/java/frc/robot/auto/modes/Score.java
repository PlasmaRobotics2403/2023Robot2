package frc.robot.auto.modes;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Elevator;
import frc.robot.Grabber;
import frc.robot.Swerve;
import frc.robot.auto.actions.AutoGrabber;
import frc.robot.auto.actions.FollowTrajectory;
import frc.robot.auto.util.Action;
import frc.robot.auto.util.AutoMode;
import frc.robot.auto.util.AutoModeEndedException;

public class Score extends AutoMode {

    Swerve swerve;
    Elevator elevator;
    Grabber grabber;
    PathPlannerTrajectory moveOneZachShoeBackward;
    PathPlannerTrajectory moveOneZachShoeForward;


    public Score(Swerve swerve, Elevator elevator, Grabber grabber) {
        this.swerve = swerve;
        this.elevator = elevator;
        this.grabber = grabber;

        try {
                moveOneZachShoeForward = PathPlanner.loadPath("moveOneZachShoeForward", new PathConstraints(2, 2));
                moveOneZachShoeBackward = PathPlanner.loadPath("moveOneZachShoeBackward", new PathConstraints(2, 2));

        }
        catch (Exception e) {
            DriverStation.reportError(e.getMessage(), false);
        }
        

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new AutoGrabber(grabber, -0.5, 0.75));

    }
    
}
