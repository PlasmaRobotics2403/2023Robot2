package frc.robot.auto.modes;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Elevator;
import frc.robot.Grabber;
import frc.robot.Swerve;
import frc.robot.auto.actions.AutoArm;
import frc.robot.auto.actions.AutoElevator;
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
    PathPlannerTrajectory leaveCommunity;


    public Score(Swerve swerve, Elevator elevator, Grabber grabber) {
        this.swerve = swerve;
        this.elevator = elevator;
        this.grabber = grabber;

        try {
                moveOneZachShoeForward = PathPlanner.loadPath("moveOneZachShoeForward", new PathConstraints(0.5, 0.5));
                moveOneZachShoeBackward = PathPlanner.loadPath("moveOneZachShoeBackward", new PathConstraints(0.5, 0.5));
                leaveCommunity = PathPlanner.loadPath("leave community", new PathConstraints(0.5, 0.5));

        }
        catch (Exception e) {
            DriverStation.reportError(e.getMessage(), false);
        }
        

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runActionsParallel(new AutoElevator(elevator, Constants.ElevatorConstants.ELEVATOR_MID_EXTEND, 5), new AutoArm(grabber, Constants.GrabberConstants.ARM_MID_EXTEND, 5));
        runAction(new AutoGrabber(grabber, -0.5, 0.75));
        Action[] leaveComunity = {new FollowTrajectory(leaveCommunity, swerve, true), new AutoElevator(elevator, Constants.ElevatorConstants.ELEVATOR_BOTTTOM_EXTEND, 5), new AutoArm(grabber, Constants.GrabberConstants.ARM_STOW_EXTEND, 3)};
        parallel(leaveComunity);
    }
    
}
