// File: src/main/java/frc/robot/RobotContainer.java
package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.FieldCosntants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Drive.CommandSwerveDrivetrain;
import frc.robot.Drive.DriveCommands;
import frc.robot.Drive.generated.TunerConstants;
import frc.robot.Shooting.HoodSubsystem;
import frc.robot.Shooting.ShootingHelper;
import frc.robot.Util.ShooterMotorTestCmd;
import frc.robot.Util.SparkMaxMotorTestCmd;
import frc.robot.Util.TalonFxMotorTestCmd;
import frc.robot.Vision.VisionHardwareFactoryImpl;
import frc.robot.Vision.VisionStandardDeviationModel;
import frc.robot.Vision.VisionSubsystem;

public class RobotContainer {

    private final double maximumSpeedMetersPerSecond =
        1.0 * TunerConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond);

    private final double maximumAngularRateRadiansPerSecond =
        RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric fieldCentricDriveRequest =
        new SwerveRequest.FieldCentric()
            .withDeadband(maximumSpeedMetersPerSecond * 0.05)
            .withRotationalDeadband(maximumAngularRateRadiansPerSecond * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brakeRequest =
        new SwerveRequest.SwerveDriveBrake();

    private final SwerveRequest.PointWheelsAt pointWheelsRequest =
        new SwerveRequest.PointWheelsAt();

    private final Telemetry telemetry = new Telemetry(maximumSpeedMetersPerSecond);

    
 // ------------ CONTROLES ---------------
    private final CommandXboxController driverController =
        new CommandXboxController(0);

    private final CommandXboxController addOnsController =
        new CommandXboxController(1);

    

    public final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain();

    //---------------- Vision ----------------

    private final VisionStandardDeviationModel visionStandardDeviationModel =
        new VisionStandardDeviationModel(
            VisionConstants.MAXIMUM_AMBIGUITY_FOR_SINGLE_TAG,
            VisionConstants.MAXIMUM_Z_ERROR_METERS,
            VisionConstants.MAXIMUM_OBSERVATION_AGE_SECONDS,
            VisionConstants.MAXIMUM_DISTANCE_FOR_SINGLE_TAG_METERS,
            VisionConstants.MAXIMUM_DISTANCE_FOR_MULTI_TAG_METERS,
            VisionConstants.MAXIMUM_YAW_RATE_RADIANS_PER_SECOND,
            VisionConstants.MAXIMUM_LINEAR_STANDARD_DEVIATION_METERS,
            VisionConstants.MAXIMUM_ANGULAR_STANDARD_DEVIATION_RADIANS
        );

    private final VisionHardwareFactoryImpl visionHardwareFactory =
        new VisionHardwareFactoryImpl(false);

    private final VisionSubsystem visionSubsystem;

    // ---------------- Shooting ----------------

    private final ShootingHelper shootingHelper =
        new ShootingHelper(FieldCosntants.IS_ANDYMARK_FIELD);

    private final HoodSubsystem hoodSubsystem;

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

    public RobotContainer() {

        hoodSubsystem = new HoodSubsystem();

        visionSubsystem = createVisionSubsystem();
        configureBindings();
    }

    private VisionSubsystem createVisionSubsystem() {
        AprilTagFieldLayout aprilTagFieldLayout = FieldCosntants.IS_ANDYMARK_FIELD 
            ? AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark)
            : AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        VisionSubsystem.VisionPoseMeasurementConsumer visionPoseMeasurementConsumer =
            (visionRobotPose, timestampSeconds, visionMeasurementStandardDeviations) ->
                drivetrain.addVisionMeasurement(visionRobotPose, timestampSeconds, visionMeasurementStandardDeviations);

        return new VisionSubsystem(
            aprilTagFieldLayout,
            FieldCosntants.FIELD_LENGTH_METERS,
            FieldCosntants.FIELD_WIDTH_METERS,
            () -> drivetrain.getPose(),
            () -> drivetrain.getYawRateRadiansPerSecond(),
            visionPoseMeasurementConsumer,
            visionStandardDeviationModel,
            VisionConstants.cameraSpecificationsList,
            visionHardwareFactory
        );
    }

    private void configureBindings() {

         // drivetrain.setDefaultCommand(
        //     drivetrain.applyRequest(() ->
        //         fieldCentricDriveRequest
        //             .withVelocityX(-driverController.getLeftY() * maximumSpeedMetersPerSecond)
        //             .withVelocityY(-driverController.getLeftX() * maximumSpeedMetersPerSecond)
        //             .withRotationalRate(-driverController.getRightX() * maximumAngularRateRadiansPerSecond)
        //     )
        // );

        
        drivetrain.setDefaultCommand(
            DriveCommands.joystickDriveWithVisionAim(
                drivetrain,
                fieldCentricDriveRequest,
                visionSubsystem,
                shootingHelper,
                () -> driverController.rightBumper().getAsBoolean(),
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                maximumSpeedMetersPerSecond,
                maximumAngularRateRadiansPerSecond
            )
        );

        final var idleRequest = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idleRequest).ignoringDisable(true)
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brakeRequest));
        driverController.b().whileTrue(drivetrain.applyRequest(() -> pointWheelsRequest.withModuleDirection( new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX())))
        );

        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        drivetrain.registerTelemetry(telemetry::telemeterize);


/*
        // ---------------- Intake bindings (example, simple) ----------------
        // Adjust buttons to your preference.
        driverController.x().whileTrue(new frc.robot.Intake.ActivateIntakeCmd(intakeSubsystem));
        driverController.rightBumper().onTrue(new frc.robot.Intake.DeployIntakeCmd(intakeSubsystem));
        driverController.leftBumper().onTrue(new frc.robot.Intake.RetractIntakeCmd(intakeSubsystem));
        
        // ---------------- Climber bindings (example) ----------------
        addOnsController.rightBumper().onTrue(new frc.robot.Climber.ExpandClimberCmd(climberSubsystem));
        addOnsController.leftBumper().onTrue(new frc.robot.Climber.RetractClimberCmd(climberSubsystem));*/

        // ---------------- Hood / shooting bindings (example) ----------------
        // If you have a HoodCmd that uses vision + helper, bind it here.
        //driverController.rightTrigger().whileTrue(new frc.robot.Shooting.HoodCmd(hoodSubsystem, visionSubsystem, shootingHelper));
        driverController.start().onTrue(Commands.runOnce(() -> hoodSubsystem.zeroHoodPosition()));
        driverController.leftTrigger().whileTrue(new ShooterMotorTestCmd());

    
        

        driverController.x().whileTrue(
            drivetrain.applyRequest(() -> fieldCentricDriveRequest .withVelocityX(1.0) .withVelocityY(0.0) .withRotationalRate(0.0)));

        addOnsController.a().whileTrue(
            Commands.parallel(
                new SparkMaxMotorTestCmd(
                    "LeftPivotIntake",
                    IntakeConstants.PIVOT_INTAKE_LEFT_MOTOR_ID,
                    () -> addOnsController.getLeftY(), false, 0.3),
                new SparkMaxMotorTestCmd(
                    "RightPivotIntake",
                    IntakeConstants.PIVOT_INTAKE__RIGHT_MOTOR_ID,
                    () -> addOnsController.getLeftY(), true, 0.3)));

        addOnsController.y().whileTrue(
            new TalonFxMotorTestCmd(
                "HoodAngleMotor",
                HoodConstants.HOOD_ANGLE_TALON_ID,
                () -> addOnsController.getLeftY(), false, 0.2));
    }

    public Command getAutonomousCommand() {
        final var idleRequest = new SwerveRequest.Idle();

        return Commands.sequence(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            drivetrain.applyRequest(() -> fieldCentricDriveRequest .withVelocityX(0.5) .withVelocityY(0.0) .withRotationalRate(0.0) ).withTimeout(5.0),
            drivetrain.applyRequest(() -> idleRequest)
        );
    }
}
