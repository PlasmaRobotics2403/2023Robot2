package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Grabber {
    CANSparkMax rotMotor;
    CANSparkMax grabberMotor;

    public Grabber() {
        rotMotor = new CANSparkMax(0, MotorType.kBrushless);
        grabberMotor = new CANSparkMax(0, MotorType.kBrushless);

        rotMotor.setIdleMode(IdleMode.kBrake);
        rotMotor.setInverted(false);
    }

    public void runGrabber(double speed) {
        grabberMotor.set(speed);
    }

    public void rotateGrabber() {
         
    }
}
