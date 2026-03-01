package frc.robot;




    import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Constants {

  public static final class IntakeConstants {

        // Roller Motor ID (CAN ID from REV Hardware Client)
        public static final int ROLLER_MOTOR_ID = 19;

        // Roller settings
        public static final boolean kRollerInverted = false;
        public static final IdleMode kRollerIdleMode = IdleMode.kCoast;

        // Current limit (amps)
        public static final int kRollerCurrentLimit = 20;

        public static final double kIntakeSpeed = 0.6;
        public static final double kOuttakeSpeed = -0.6;
  }
    public final class ShooterConstants {

    // CAN IDs
    public static final int shooterLEFT_ID = 16;
    public static final int shooterRIGHT_ID = 17;
    public static final int shooterINDEXER_ID = 22;

    // Inversion (true if the motor spins opposite of desired)
    public static final boolean shooterLEFT_INVERTED = true;
    public static final boolean shooterRIGHT_INVERTED = false;
    public static final boolean shooterINDEXER_INVERTED = true;


    // Default speeds (percent output or RPM)
    public static final double kShooterRPM = 4000;   // Example RPM for flywheel
    public static final double kIndexerSpeed = 0.5;  // Example percent output for indexer

    private ShooterConstants() {
        // Prevent instantiation
    }
}
}

    




