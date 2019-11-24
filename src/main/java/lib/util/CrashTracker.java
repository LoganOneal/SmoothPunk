package lib.util;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;

/**
 * Tracks start-up and caught crash events, logging them to a file which dosn't roll over
 */
public class CrashTracker {

    private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();

    private static  String outputPath;

    public static void logRobotConstruction() {
        logMarker("robot startup");
    }

    public static void logRobotInit() {
        logMarker("robot init");
    }

    public static void logTeleopInit() {
        logMarker("teleop init");
    }

    public static void logAutoInit() {
        logMarker("auto init");
    }

    public static void logDisabledInit() {
        logMarker("disabled init");
    }

    public static void logTestInit() {
        logMarker("test init");
    }

    public static void logThrowableCrash(Throwable throwable) {
        logMarker("Exception", throwable);
    }

    private static void logMarker(String mark) {
        logMarker(mark, null);
    }

    private static void logMarker(String mark, Throwable nullableException) {
        
        if (RobotBase.isReal()) {
            outputPath = "/home/lvuser/crash_tracking.txt";
        } else {
            outputPath = System.getProperty("user.dir") + "/crash_tracking.txt";
            System.out.println(outputPath);
        }

        try (PrintWriter writer = new PrintWriter(new FileWriter(outputPath, true))) {
            writer.print(RUN_INSTANCE_UUID.toString());
            writer.print(", ");
            writer.print(mark);
            writer.print(", ");
            writer.print(new Date().toString());

            if (nullableException != null) {
                writer.print(", ");
                nullableException.printStackTrace(writer);
            }

            writer.println();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}