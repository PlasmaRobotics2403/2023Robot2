package frc.robot.auto.modes;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Swerve;

import frc.robot.auto.actions.FollowTrajectory;
import frc.robot.auto.util.AutoMode;
import frc.robot.auto.util.AutoModeEndedException;

public class LeaveCommunity extends AutoMode {

    Swerve swerve;
    PathPlannerTrajectory leaveCommunity;

    public LeaveCommunity(Swerve swerve) {
        this.swerve = swerve;

        try {
            leaveCommunity = PathPlanner.loadPath("leave community", new PathConstraints(0.5, 0.5));
        }
        catch (Exception e) {
            DriverStation.reportError(e.getMessage(), false);
        }
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new FollowTrajectory(leaveCommunity, swerve, true));

    }
    
}
