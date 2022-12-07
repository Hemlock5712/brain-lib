package org.team5712.lib.swerve;

public class SwerveConstants {
    public int DRIVE_MOTOR;
    public int STEER_MOTOR;
    public int STEER_ENCODER;
    public double STEER_OFFSET;

    public SwerveConstants(int DRIVE_MOTOR, int STEER_MOTOR, int STEER_ENCODER, double STEER_OFFSET) {
        this.DRIVE_MOTOR = DRIVE_MOTOR;
        this.STEER_MOTOR = STEER_MOTOR;
        this.STEER_ENCODER = STEER_ENCODER;
        this.STEER_OFFSET = STEER_OFFSET;
    }
}
