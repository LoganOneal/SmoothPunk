package frc.robot.drivers.motors.sparkMAX;

import com.revrobotics.CANError;

import edu.wpi.first.wpilibj.DriverStation;

public class SparkMaxUtil {
    // Checks the specified error code for issues.
    public static void checkError(CANError errorCode, String message) {
        if (errorCode != CANError.kOk) {
            DriverStation.reportError(message + errorCode, false);
        }
    }
}