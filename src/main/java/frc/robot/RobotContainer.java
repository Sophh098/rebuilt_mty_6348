// File: src/main/java/frc/robot/RobotContainer.java
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Util.SparkMaxMotorTestCmd;
import frc.robot.Util.TalonFxMotorTestCmd;

public class RobotContainer {

    // private final double maximumSpeedMetersPerSecond =
    //     1.0 * TunerConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond);

    // private final double maximumAngularRateRadiansPerSecond =
    //     RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // private final SwerveRequest.FieldCentric fieldCentricDriveRequest =
    //     new SwerveRequest.FieldCentric()
    //         .withDeadband(maximumSpeedMetersPerSecond * 0.05)
    //         .withRotationalDeadband(maximumAngularRateRadiansPerSecond * 0.05)
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // private final SwerveRequest.SwerveDriveBrake brakeRequest =
    //     new SwerveRequest.SwerveDriveBrake();

    // private final SwerveRequest.PointWheelsAt pointWheelsRequest =
    //     new SwerveRequest.PointWheelsAt();

    // private final Telemetry telemetry = new Telemetry(maximumSpeedMetersPerSecond);

    private final CommandXboxController driverController =
        new CommandXboxController(0);

    private final CommandXboxController addOnsController =
        new CommandXboxController(1);

    // public final CommandSwerveDrivetrain drivetrain =
    //     TunerConstants.createDrivetrain();

    // ---------------- Vision ----------------

    // private final VisionStandardDeviationModel visionStandardDeviationModel =
    //     new VisionStandardDeviationModel(
    //         VisionConstants.MAXIMUM_AMBIGUITY_FOR_SINGLE_TAG,
    //         VisionConstants.MAXIMUM_Z_ERROR_METERS,
    //         VisionConstants.MAXIMUM_OBSERVATION_AGE_SECONDS,
    //         VisionConstants.MAXIMUM_DISTANCE_FOR_SINGLE_TAG_METERS,
    //         VisionConstants.MAXIMUM_DISTANCE_FOR_MULTI_TAG_METERS,
    //         VisionConstants.MAXIMUM_YAW_RATE_RADIANS_PER_SECOND,
    //         VisionConstants.MAXIMUM_LINEAR_STANDARD_DEVIATION_METERS,
    //         VisionConstants.MAXIMUM_ANGULAR_STANDARD_DEVIATION_RADIANS
    //     );

    // private final VisionHardwareFactoryImpl visionHardwareFactory =
    //     new VisionHardwareFactoryImpl(false);

    // private final VisionSubsystem visionSubsystem;

    // // ---------------- Shooting ----------------

    // private final ShootingHelper shootingHelper =
    //     new ShootingHelper(FieldCosntants.IS_ANDYMARK_FIELD);

    // private final HoodSubsystem hoodSubsystem;

    // // ---------------- Intake ----------------

    //  private final SparkMax intakeRollerMotorController =
    //     new SparkMax(IntakeConstants.ROLLER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    //  private final SparkMax intakePivotMotorLeftController =
    //     new SparkMax(IntakeConstants.PIVOT_INTAKE_LEFT_MOTOR, SparkLowLevel.MotorType.kBrushless);

    //  private final SparkMax intakePivotMotorRightController =
    //     new SparkMax(IntakeConstants.PIVOT_INTAKE_RIGHT_MOTOR, SparkLowLevel.MotorType.kBrushless);

    //  private final IntakeSubsystem intakeSubsystem =
    //     new IntakeSubsystem(intakeRollerMotorController, intakePivotMotorRightController, intakePivotMotorLeftController);

    // // ---------------- Climber ----------------

    // private final SparkMax leftClimberMotorController =
    //     new SparkMax(frc.robot.Constants.ClimberConstants.LEFT_CLIMBER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    // private final SparkMax rightClimberMotorController =
    //     new SparkMax(frc.robot.Constants.ClimberConstants.RIGHT_CLIMBER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    // private final ClimberSubsystem climberSubsystem =
    //     new ClimberSubsystem(leftClimberMotorController, rightClimberMotorController);

    private final Command leftPivotIntakeTestCommand =
    new SparkMaxMotorTestCmd("LeftPivotIntake", IntakeConstants.PIVOT_INTAKE_LEFT_MOTOR_ID, () -> addOnsController.getLeftY(), false, 0.4);

    private final Command rightPivotIntakeTestCommand =
        new SparkMaxMotorTestCmd("RightPivotIntake", IntakeConstants.PIVOT_INTAKE__RIGHT_MOTOR_ID, () -> addOnsController.getLeftY(), true, 0.4);

    private final Command hoodAngleMotorTestCommand =
        new TalonFxMotorTestCmd("HoodAngleMotor", HoodConstants.HOOD_ANGLE_TALON_ID, () -> addOnsController.getRightY(), false, 0.4);

    public RobotContainer() {
        // hoodSubsystem = new HoodSubsystem();

        // visionSubsystem =
        //     createVisionSubsystem();

        configureBindings();
    }

    // private VisionSubsystem createVisionSubsystem() {
    //     // You need a real AprilTagFieldLayout reference here.
    //     // I assume you already have it in VisionConstants.
    //     AprilTagFieldLayout aprilTagFieldLayout = FieldCosntants.IS_ANDYMARK_FIELD ? AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark) : AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    //     VisionSubsystem.VisionPoseMeasurementConsumer visionPoseMeasurementConsumer =
    //         (visionRobotPose, timestampSeconds, visionMeasurementStandardDeviations) ->
    //             drivetrain.addVisionMeasurement(visionRobotPose, timestampSeconds, visionMeasurementStandardDeviations);

    //     return new VisionSubsystem(
    //         aprilTagFieldLayout,
    //         FieldCosntants.FIELD_LENGTH_METERS,
    //         FieldCosntants.FIELD_WIDTH_METERS,
    //         ()-> drivetrain.getPose(),
    //         ()-> drivetrain.getYawRateRadiansPerSecond(),
    //         visionPoseMeasurementConsumer,
    //         visionStandardDeviationModel,
    //         VisionConstants.cameraSpecificationsList,
    //         visionHardwareFactory
    //     );
    // }

    private void configureBindings() {
        
/* 
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                fieldCentricDriveRequest
                    .withVelocityX(-driverController.getLeftY() * maximumSpeedMetersPerSecond)
                    .withVelocityY(-driverController.getLeftX() * maximumSpeedMetersPerSecond)
                    .withRotationalRate(-driverController.getRightX() * maximumAngularRateRadiansPerSecond)
            )
        );

        final var idleRequest = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idleRequest).ignoringDisable(true)
        );

        addOnsController.a().whileTrue(drivetrain.applyRequest(() -> brakeRequest));
        addOnsController.b().whileTrue(
            drivetrain.applyRequest(() ->
                pointWheelsRequest.withModuleDirection(
                    new Rotation2d(-addOnsController.getLeftY(), -addOnsController.getLeftX())
                )
            )
        );

        addOnsController.back().and(addOnsController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        addOnsController.back().and(addOnsController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        addOnsController.start().and(addOnsController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        addOnsController.start().and(addOnsController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(telemetry::telemeterize);

        // ---------------- Intake bindings (example, simple) ----------------
        // Adjust buttons to your preference.
        driverController.x().whileTrue(new frc.robot.Intake.ActivateIntakeCmd(intakeSubsystem));
        driverController.rightBumper().onTrue(new frc.robot.Intake.DeployIntakeCmd(intakeSubsystem));
        driverController.leftBumper().onTrue(new frc.robot.Intake.RetractIntakeCmd(intakeSubsystem));
        
        // ---------------- Climber bindings (example) ----------------
        addOnsController.rightBumper().onTrue(new frc.robot.Climber.ExpandClimberCmd(climberSubsystem));
        addOnsController.leftBumper().onTrue(new frc.robot.Climber.RetractClimberCmd(climberSubsystem));

        // ---------------- Hood / shooting bindings (example) ----------------
        // If you have a HoodCmd that uses vision + helper, bind it here.
        driverController.rightTrigger().whileTrue(new frc.robot.Shooting.HoodCmd(hoodSubsystem, visionSubsystem, shootingHelper));*/

        
        

        addOnsController.a().whileTrue(
            Commands.parallel(
                new SparkMaxMotorTestCmd(
                    "LeftPivotIntake",
                    IntakeConstants.PIVOT_INTAKE_LEFT_MOTOR_ID,
                    () -> addOnsController.getLeftY(),
                    false,
                    0.4
                ),
                new SparkMaxMotorTestCmd(
                    "RightPivotIntake",
                    IntakeConstants.PIVOT_INTAKE__RIGHT_MOTOR_ID,
                    () -> addOnsController.getLeftY(),
                    true,
                    0.4
                )
            )
        );

        addOnsController.y().whileTrue(
            new TalonFxMotorTestCmd(
                "HoodAngleMotor",
                HoodConstants.HOOD_ANGLE_TALON_ID,
                () -> addOnsController.getRightY(),
                false,
                0.4
            )
        );
        
    }

    public Command getAutonomousCommand() {
        // final var idleRequest = new SwerveRequest.Idle();

        // return Commands.sequence(
        //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //     drivetrain.applyRequest(() ->
        //         fieldCentricDriveRequest
        //             .withVelocityX(0.5)
        //             .withVelocityY(0.0)
        //             .withRotationalRate(0.0)
        //     ).withTimeout(5.0),
        //     drivetrain.applyRequest(() -> idleRequest)
        // );
        return null;
    }
}