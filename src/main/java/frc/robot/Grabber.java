package frc.robot;

import java.util.ResourceBundle.Control;

import org.ejml.sparse.csc.decomposition.qr.QrLeftLookingDecomposition_DSCC;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.CTREConfigs;

public class Grabber {
    private CANSparkMax rotMotor;
    private CANSparkMax grabberMotor;
    private WPI_CANCoder grabberCC;

    private ProfiledPIDController pidController;
    private TrapezoidProfile.Constraints constraints;
    private ArmFeedforward feedForward;

    public Grabber() {
        constraints = new TrapezoidProfile.Constraints(0.1, 0.1);
        pidController = new ProfiledPIDController(Constants.GrabberConstants.armkP, Constants.GrabberConstants.armkI, Constants.GrabberConstants.armkD, constraints);
        feedForward = new ArmFeedforward(0.1, 0.1, 0.1);

        grabberCC = new WPI_CANCoder(Constants.GrabberConstants.GRABBER_CC_ID);
        grabberCC.configFactoryDefault();
        grabberCC.configAllSettings(CTREConfigs.armCanCoderConfig());

        rotMotor = new CANSparkMax(0, MotorType.kBrushless);
        grabberMotor = new CANSparkMax(0, MotorType.kBrushless);

        rotMotor.setIdleMode(IdleMode.kBrake);
        rotMotor.setInverted(false);

    }

    public void runGrabber(double speed) {
        grabberMotor.set(speed);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(grabberCC.getAbsolutePosition());
    }

    public double getAbsoluteArmPosition() {
        return grabberCC.getAbsolutePosition();
    }

    public void magicArm(double rotPosition) {
        pidController.setGoal(Math.toRadians(rotPosition));
        double speed =  pidController.calculate(Math.toRadians(grabberCC.getAbsolutePosition()));
        speed -= feedForward.calculate(Math.toRadians(grabberCC.getAbsolutePosition()) - Math.PI/2, 0);
        rotMotor.set(speed);
    }

    public void logging() {
        SmartDashboard.putNumber("GrabberCC", getAbsoluteArmPosition());

    }
}
