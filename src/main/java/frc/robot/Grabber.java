package frc.robot;

import java.util.ResourceBundle.Control;

import org.ejml.sparse.csc.decomposition.qr.QrLeftLookingDecomposition_DSCC;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.CTREConfigs;

public class Grabber {
    private CANSparkMax rotMotor;
    private CANSparkMax grabberMotor;
    private WPI_CANCoder grabberCC;

    private PIDController pidController;
    private TrapezoidProfile.Constraints constraints;
    private ArmFeedforward feedForward;

    private DigitalInput limitSwitch;


    public Grabber() {
        limitSwitch = new DigitalInput(Constants.GrabberConstants.GRABBER_LIMITSWITCH);
        constraints = new TrapezoidProfile.Constraints(0.1, 0.1);
        pidController = new PIDController(Constants.GrabberConstants.armkP, Constants.GrabberConstants.armkI, Constants.GrabberConstants.armkP);
        feedForward = new ArmFeedforward(0.1, 0.1, 0.1);

        grabberCC = new WPI_CANCoder(Constants.GrabberConstants.GRABBER_CC_ID);
        grabberCC.configFactoryDefault();
        grabberCC.configAllSettings(CTREConfigs.armCanCoderConfig());

        rotMotor = new CANSparkMax(Constants.GrabberConstants.ARM_ID, MotorType.kBrushless);
        grabberMotor = new CANSparkMax(Constants.GrabberConstants.GRABBER_ID, MotorType.kBrushless);

        rotMotor.setIdleMode(IdleMode.kBrake);
        rotMotor.setInverted(false);

    }

    public void runGrabberSesor(double speed) {
        if(!limitSwitch.get()){
            grabberMotor.set(0);
        }     

        else{
            grabberMotor.set(speed);
        }
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
        double pid = pidController.calculate(grabberCC.getAbsolutePosition(), rotPosition);

        if(pid > 0) {
            pid = Math.min(pid, 0.2);
        }
        else {
            pid = Math.max(pid, -0.2);
        }

        rotMotor.set(pid);

        DriverStation.reportError(Double.toString(pid), false);
    }

    public void armToPos(double position, double speed) {
        DriverStation.reportError(Double.toString(getAbsoluteArmPosition()) + "-" + Double.toString(position), false);
        if(position + 4.5 >= grabberCC.getAbsolutePosition() && position - 4.5 <= grabberCC.getAbsolutePosition()) {
            DriverStation.reportError("STOP", false);
            rotMotor.set(0);
        }
        else{
            if(position >= getAbsoluteArmPosition()) {
                rotMotor.set(speed);
            }
            else if(position <= getAbsoluteArmPosition()) {
                rotMotor.set(-speed);
            }
        }
    }

    public void logging() {
        SmartDashboard.putNumber("GrabberCC", getAbsoluteArmPosition());
        SmartDashboard.putBoolean("Grabber LimitSwitch", !limitSwitch.get());

    }
}
