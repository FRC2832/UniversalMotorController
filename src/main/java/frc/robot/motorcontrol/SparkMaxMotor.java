package frc.robot.motorcontrol;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SparkMaxMotor extends SparkBaseMotor {

    public SparkMaxMotor(String motorName, int id) {
        this(motorName, new SparkMax(id, MotorType.kBrushless), false);
    }

    public SparkMaxMotor(String motorName, int id, MotorType type) {
        this(motorName, new SparkMax(id, type), false);
    }

    public SparkMaxMotor(String motorName, SparkMax motor, boolean logOnly) {
        super(motorName, motor, logOnly);
    }
}
