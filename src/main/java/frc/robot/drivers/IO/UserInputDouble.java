package frc.robot.drivers.IO;

/**
 * An interface primarily for passing values from a joystick to wherever it
 * needs to go. Can support grabbing of axis, or applying some transformation to
 * axis(s)
 */
public interface UserInputDouble {
    /**
     * @return a value -1.0 to 1.0
     */
    public double getValue();
}