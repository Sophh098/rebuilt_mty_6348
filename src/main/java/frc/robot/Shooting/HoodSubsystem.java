package frc.robot.Shooting;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Drive.generated.TunerConstants;
import frc.robot.Vision.VisionSubsystem;

/**
 * HoodSubsystem — Sistema de disparo con hood activo (Phoenix Pro).
 *
 * ═══════════════════════════════════════════════════════════════
 * FLUJO DE ESTADOS:
 *   STOWED ──► AIMING_AND_SPINNING ──► POST_SHOT_COASTING ──► STOWING ──► STOWED
 *
 * LÓGICA DE ÁNGULO:
 *   ShootingHelper devuelve el ángulo del PROYECTIL (trayectoria).
 *   El ángulo FÍSICO del hood es el complementario:
 *     hoodFísico   = 90° − proyectil_angle
 *     hoodRelativo = hoodFísico − 24°         (encoder = 0 cuando hood = 24°)
 *     rotaciones   = hoodRelativo_rad × CONVERSION_RAD_TO_ROT
 *
 * REGLA DE INVERSIÓN DE RUEDAS:
 *   Las ruedas están en lados OPUESTOS de la barra de disparo.
 *   Para empujar la nota en el mismo sentido deben girar en sentidos opuestos.
 *   → Cada rueda tiene su propio objeto MotorOutputConfigs (nunca compartido).
 *   → RIGHT_WHEEL_INVERTED y LEFT_WHEEL_INVERTED en HoodConstants para ajustar.
 *
 * CONDICIÓN DEL INDEXER:
 *   indexerEnabled = shootingRequestActive && readyToShoot
 *   readyToShoot   = ruedas en velocidad && hood en ángulo   (sin isSafeToShoot)
 *   isSafeToShoot() es solo informativo en telemetría — no bloquea el disparo.
 * ═══════════════════════════════════════════════════════════════
 */
public class HoodSubsystem extends SubsystemBase {

    // ──────────────────────────────────────────────────────────────
    // ENUMS PÚBLICOS
    // ──────────────────────────────────────────────────────────────
    public enum ShootingMode {
        DEFAULT,    // Valores fijos de Constants (sin visión válida)
        CALCULATED  // Modelo matemático de ShootingHelper (con AprilTag válido)
    }

    private enum HoodOperationalState {
        STOWED,
        AIMING_AND_SPINNING,
        POST_SHOT_COASTING,
        STOWING
    }

    // ──────────────────────────────────────────────────────────────
    // HARDWARE
    // ──────────────────────────────────────────────────────────────
    private final TalonFX hoodAngleMotorController;
    private final TalonFX rightWheelMotorController;
    private final TalonFX leftWheelMotorController;
    private final TalonFX indexerMotorController;

    // ──────────────────────────────────────────────────────────────
    // CONTROL REQUESTS — Phoenix Pro
    // ──────────────────────────────────────────────────────────────
    private final VelocityTorqueCurrentFOC rightWheelVelocityRequest =
        new VelocityTorqueCurrentFOC(0.0).withSlot(0);

    private final VelocityTorqueCurrentFOC leftWheelVelocityRequest =
        new VelocityTorqueCurrentFOC(0.0).withSlot(0);

    private final MotionMagicExpoTorqueCurrentFOC hoodAnglePositionRequest =
        new MotionMagicExpoTorqueCurrentFOC(0.0).withSlot(0);

    private final DutyCycleOut indexerDutyCycleRequest =
        new DutyCycleOut(0.0);

    // ──────────────────────────────────────────────────────────────
    // STATE
    // ──────────────────────────────────────────────────────────────
    private final Debouncer readyDebouncer =
        new Debouncer(0.12, Debouncer.DebounceType.kRising);

    private HoodOperationalState operationalState = HoodOperationalState.STOWED;
    private ShootingMode         shootingMode     = ShootingMode.DEFAULT;

    private boolean shootingRequestActive = false;
    private boolean readyToShoot          = false;
    private boolean indexerEnabled        = false;

    // ──────────────────────────────────────────────────────────────
    // PARÁMETROS DE DISPARO
    // ──────────────────────────────────────────────────────────────
    /** Velocidad de salida del proyectil [m/s]. */
    private double desiredExitSpeedMetersPerSecond =
        ShootingConstants.DEFAULT_EXIT_SPEED_METERS_PER_SECOND;

    /**
     * Ángulo del PROYECTIL calculado por ShootingHelper [radianes].
     * NO es el ángulo físico del hood — se convierte en executeAimingState().
     */
    private double desiredProjectileAngleRadians =
        ShootingConstants.DEFAULT_HOOD_ANGLE_RADIANS;

    // ──────────────────────────────────────────────────────────────
    // TRACKING INTERNO
    // ──────────────────────────────────────────────────────────────
    private double coastStartTimestamp        = 0.0;
    private double hoodHoldPositionRotations  = 0.0;

    private double goalWheelRPS               = 0.0;
    private double goalHoodPositionRotations  = 0.0;

    private double hoodPositionRotations      = 0.0;
    private double leftWheelRPS               = 0.0;
    private double rightWheelRPS              = 0.0;

    // ══════════════════════════════════════════════════════════════
    // CONSTRUCTORES
    // ══════════════════════════════════════════════════════════════

    public HoodSubsystem() {
        this(TunerConstants.CONTROLLER_AREA_NETWORK_BUS);
    }

    public HoodSubsystem(CANBus canBusName) {
        hoodAngleMotorController  = new TalonFX(HoodConstants.HOOD_ANGLE_TALON_ID,            canBusName);
        rightWheelMotorController = new TalonFX(HoodConstants.RIGHT_HOOD_PROPULSION_TALON_ID, canBusName);
        leftWheelMotorController  = new TalonFX(HoodConstants.LEFT_HOOD_PROPULSION_TALON_ID,  canBusName);
        indexerMotorController    = new TalonFX(HoodConstants.INDEXER_TALON_ID,               canBusName);

        configureHoodAngleMotor();
        configureRightWheelMotor();
        configureLeftWheelMotor();
        configureIndexerMotor();

        System.out.println("✅ [HoodSubsystem] Inicialización completa (Phoenix Pro)");
        System.out.printf ("   CONVERSION_RAD_TO_ROT    = %.4f rot/rad%n",
            HoodConstants.CONVERSION_RATIO_RAD_TO_ROT);
        System.out.printf ("   Hood físico mín/máx       = %.1f° / %.1f°%n",
            Math.toDegrees(ShootingConstants.MINIMUM_HOOD_ANGLE_RADIANS),
            Math.toDegrees(ShootingConstants.MAXIMUM_HOOD_ANGLE_RADIANS));
        System.out.printf ("   DEFAULT exit speed        = %.2f m/s%n",
            ShootingConstants.DEFAULT_EXIT_SPEED_METERS_PER_SECOND);
    }

    // ══════════════════════════════════════════════════════════════
    // CONFIGURACIÓN DE MOTORES
    // ══════════════════════════════════════════════════════════════

    private void configureHoodAngleMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0               = HoodConstants.HOOD_ANGLE_SLOT_CONFIGS;
        config.MotionMagic         = HoodConstants.HOOD_MOTION_MAGIC_CONFIGS;
        config.MotorOutput         = HoodConstants.HOOD_ANGLE_MOTOR_OUTPUT_CONFIGS;
        config.CurrentLimits       = HoodConstants.HOOD_ANGLE_CURRENT_LIMITS;
        config.SoftwareLimitSwitch = HoodConstants.HOOD_SOFTWARE_LIMITS;
        applyConfig(hoodAngleMotorController, config, "hood angle motor");
    }

    /**
     * Rueda DERECHA — objeto MotorOutputConfigs propio (no compartido).
     * Dirección controlada por HoodConstants.RIGHT_WHEEL_INVERTED.
     */
    private void configureRightWheelMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0         = HoodConstants.RIGHT_HOOD_PROPULSION_SLOT_CONFIGS;
        config.CurrentLimits = HoodConstants.WHEEL_CURRENT_LIMITS;

        // ⚠️ SIEMPRE crear un objeto nuevo — nunca reutilizar el de Constants directamente
        MotorOutputConfigs rightOutput = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(HoodConstants.RIGHT_WHEEL_INVERTED);
        config.MotorOutput = rightOutput;

        applyConfig(rightWheelMotorController, config, "right wheel motor");
    }

    /**
     * Rueda IZQUIERDA — siempre la dirección opuesta a la derecha.
     * Objeto MotorOutputConfigs propio (no compartido con la derecha).
     * Dirección controlada por HoodConstants.LEFT_WHEEL_INVERTED.
     */
    private void configureLeftWheelMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0         = HoodConstants.LEFT_HOOD_PROPULSION_SLOT_CONFIGS;
        config.CurrentLimits = HoodConstants.WHEEL_CURRENT_LIMITS;

        // ⚠️ SIEMPRE crear un objeto nuevo — nunca reutilizar el de la rueda derecha
        MotorOutputConfigs leftOutput = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(HoodConstants.LEFT_WHEEL_INVERTED);
        config.MotorOutput = leftOutput;

        applyConfig(leftWheelMotorController, config, "left wheel motor");
    }

    private void configureIndexerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0         = HoodConstants.INDEXER_SLOT_CONFIGS;
        config.MotorOutput   = HoodConstants.INDEXER_MOTOR_OUTPUT_CONFIGS;
        config.CurrentLimits = HoodConstants.INDEXER_CURRENT_LIMITS;
        applyConfig(indexerMotorController, config, "indexer motor");
    }

    private void applyConfig(TalonFX motor, TalonFXConfiguration config, String name) {
        var status = motor.getConfigurator().apply(config);
        if (!status.isOK()) {
            System.err.printf("❌ [HoodSubsystem] ERROR configurando %s: %s%n", name, status);
        } else {
            System.out.printf("✅ [HoodSubsystem] %s configurado%n", name);
        }
    }

    // ══════════════════════════════════════════════════════════════
    // PERIODIC
    // ══════════════════════════════════════════════════════════════

    @Override
    public void periodic() {
        readHardwareSensors();

        if (DriverStation.isDisabled()) {
            applyDisabledOutputs();
            readyToShoot = false;
            shootingMode = ShootingMode.DEFAULT;
            return;
        }

        updateStateTransitions();
        executeStateLogic();
        applyEnabledOutputs();
        logTelemetry();
    }

    // ══════════════════════════════════════════════════════════════
    // LÓGICA DE ESTADOS
    // ══════════════════════════════════════════════════════════════

    private void executeStateLogic() {
        switch (operationalState) {
            case AIMING_AND_SPINNING -> executeAimingState();
            case POST_SHOT_COASTING  -> executeCoastingState();
            case STOWING             -> executeStowingState();
            case STOWED              -> executeStowedState();
        }
    }

    /**
     * AIMING_AND_SPINNING — apuntando y girando las ruedas.
     *
     * CONVERSIÓN DE ÁNGULO:
     *   1. hoodFísico   = 90° − proyectil_angle        (complementario)
     *   2. hoodRelativo = hoodFísico − 24°             (offset al encoder = 0)
     *   3. rotaciones   = hoodRelativo_rad × CONVERSION_RAD_TO_ROT
     *
     * INDEXER:
     *   Se activa SOLO cuando ruedas Y hood están en objetivo.
     *   isSafeToShoot() NO bloquea — es solo telemetría.
     */
    private void executeAimingState() {
        // ── Velocidad de ruedas ──────────────────────────────────────────
        goalWheelRPS =
            desiredExitSpeedMetersPerSecond * HoodConstants.CONVERSION_RATIO_FROM_METERS_TO_RPS;

        // ── Ángulo del hood ──────────────────────────────────────────────
        // 1. Complementario del proyectil → ángulo físico del hood
        double hoodPhysicalAngleRad =
            HoodConstants.COMPLEMENTARY_ANGLE - desiredProjectileAngleRadians;

        // 2. Restar offset mínimo (24°) → relativo al encoder
        double hoodRelativeAngleRad =
            hoodPhysicalAngleRad - HoodConstants.HOOD_ANGLE_OFFSET_RADIANS;

        // 3. Convertir radianes → rotaciones del encoder
        goalHoodPositionRotations =
            hoodRelativeAngleRad * HoodConstants.CONVERSION_RATIO_RAD_TO_ROT;

        // ── Ready check (sin isSafeToShoot como condición de bloqueo) ────
        boolean rawReady =
            isWheelVelocityReady(goalWheelRPS) &&
            isHoodAngleReady(goalHoodPositionRotations);

        readyToShoot   = readyDebouncer.calculate(rawReady);
        indexerEnabled = shootingRequestActive && readyToShoot;
    }

    /**
     * POST_SHOT_COASTING — desaceleración libre tras el disparo.
     */
    private void executeCoastingState() {
        indexerEnabled            = false;
        goalWheelRPS              = 0.0;
        goalHoodPositionRotations = hoodHoldPositionRotations;
        readyToShoot              = false;

        double elapsed    = Timer.getFPGATimestamp() - coastStartTimestamp;
        boolean minDone   = elapsed >= HoodConstants.POST_SHOT_COAST_MINIMUM_TIME_SECONDS;
        boolean maxDone   = elapsed >= HoodConstants.POST_SHOT_COAST_MAXIMUM_TIME_SECONDS;
        boolean slowEnough = areWheelsBelowStopThreshold();

        if ((minDone && slowEnough) || maxDone) {
            operationalState = HoodOperationalState.STOWING;
        }
    }

    /**
     * STOWING — regresando a posición home.
     */
    private void executeStowingState() {
        indexerEnabled            = false;
        goalWheelRPS              = 0.0;
        goalHoodPositionRotations = HoodConstants.HOOD_HOME_POSITION_ROTATIONS;
        readyToShoot              = false;

        if (isHoodAngleReady(HoodConstants.HOOD_HOME_POSITION_ROTATIONS)) {
            operationalState = HoodOperationalState.STOWED;
            shootingMode     = ShootingMode.DEFAULT;
        }
    }

    /**
     * STOWED — estado de reposo, todo apagado.
     */
    private void executeStowedState() {
        indexerEnabled            = false;
        goalWheelRPS              = 0.0;
        goalHoodPositionRotations = HoodConstants.HOOD_HOME_POSITION_ROTATIONS;
        readyToShoot              = false;
        shootingMode              = ShootingMode.DEFAULT;
    }

    // ══════════════════════════════════════════════════════════════
    // API PÚBLICA
    // ══════════════════════════════════════════════════════════════

    public void setShootingRequestActive(boolean isActive) {
        shootingRequestActive = isActive;
    }

    /**
     * Actualizar la solución de tiro.
     *
     * CALCULADO si: hasTarget && targetValid && shotPossible
     * DEFAULT en cualquier otro caso.
     *
     * Llamar DESPUÉS de shootingHelper.update() desde HoodCmd.execute().
     */
    public void updateShootingSolution(VisionSubsystem visionSubsystem,
                                       ShootingHelper shootingHelper) {
        if (!shootingRequestActive) return;

        boolean hasTarget    = visionSubsystem.hasTarget();
        boolean targetValid  = visionSubsystem.isShootingTargetValid();
        boolean shotPossible = shootingHelper.isPossibleToShoot();

        if (hasTarget && targetValid && shotPossible) {
            shootingMode                   = ShootingMode.CALCULATED;
            desiredExitSpeedMetersPerSecond = shootingHelper.getExitSpeedMetersPerSecond();
            desiredProjectileAngleRadians   = shootingHelper.getHoodAngleRadians();
            SmartDashboard.putString("Hood/ShootingModeReason",
                "CALCULATED - Vision OK, Tag válido, Tiro posible");
        } else {
            shootingMode                   = ShootingMode.DEFAULT;
            desiredExitSpeedMetersPerSecond = ShootingConstants.DEFAULT_EXIT_SPEED_METERS_PER_SECOND;
            desiredProjectileAngleRadians   = ShootingConstants.DEFAULT_HOOD_ANGLE_RADIANS;

            String reason;
            if      (!hasTarget)    reason = "DEFAULT - No AprilTag detectado";
            else if (!targetValid)  reason = "DEFAULT - Tag no válido para disparo";
            else if (!shotPossible) reason = "DEFAULT - Tiro fuera de restricciones";
            else                    reason = "DEFAULT - Razón desconocida";
            SmartDashboard.putString("Hood/ShootingModeReason", reason);
        }
    }

    public boolean isReadyToShoot()            { return readyToShoot; }
    public ShootingMode getCurrentShootingMode() { return shootingMode; }

    public void stop() {
        shootingRequestActive     = false;
        operationalState          = HoodOperationalState.STOWED;
        goalWheelRPS              = 0.0;
        goalHoodPositionRotations = HoodConstants.HOOD_HOME_POSITION_ROTATIONS;
        indexerEnabled            = false;
        readyToShoot              = false;
        shootingMode              = ShootingMode.DEFAULT;
        applyDisabledOutputs();
    }

    /**
     * Zerear el encoder del hood.
     * LLAMAR CUANDO EL HOOD ESTÉ EN LA POSICIÓN MÍNIMA (24° físico).
     */
    public void zeroHoodPosition() {
        hoodAngleMotorController.setPosition(0.0);
        System.out.println("✅ [HoodSubsystem] Hood encoder zeroado (0 rot = 24° físico)");
    }

    // ══════════════════════════════════════════════════════════════
    // HELPERS PRIVADOS
    // ══════════════════════════════════════════════════════════════

    private void updateStateTransitions() {
        if (shootingRequestActive) {
            if (operationalState != HoodOperationalState.AIMING_AND_SPINNING) {
                operationalState = HoodOperationalState.AIMING_AND_SPINNING;
                readyDebouncer.calculate(false); // reset debouncer
            }
            return;
        }

        if (operationalState == HoodOperationalState.AIMING_AND_SPINNING) {
            operationalState          = HoodOperationalState.POST_SHOT_COASTING;
            coastStartTimestamp       = Timer.getFPGATimestamp();
            hoodHoldPositionRotations = hoodPositionRotations;
            readyToShoot              = false;
        }
    }

    private void readHardwareSensors() {
        hoodPositionRotations = hoodAngleMotorController.getPosition().getValueAsDouble();
        leftWheelRPS          = leftWheelMotorController.getVelocity().getValueAsDouble();
        rightWheelRPS         = rightWheelMotorController.getVelocity().getValueAsDouble();
    }

    private void applyDisabledOutputs() {
        rightWheelMotorController.setControl(rightWheelVelocityRequest.withVelocity(0.0));
        leftWheelMotorController.setControl(leftWheelVelocityRequest.withVelocity(0.0));
        indexerMotorController.setControl(indexerDutyCycleRequest.withOutput(0.0));
        hoodAngleMotorController.setControl(
            hoodAnglePositionRequest.withPosition(hoodPositionRotations)
        );
    }

    private void applyEnabledOutputs() {
        // Ambas ruedas reciben el mismo RPS; la inversión de sentido está en cada config
        rightWheelMotorController.setControl(rightWheelVelocityRequest.withVelocity(goalWheelRPS));
        leftWheelMotorController.setControl(leftWheelVelocityRequest.withVelocity(goalWheelRPS));

        hoodAngleMotorController.setControl(
            hoodAnglePositionRequest.withPosition(goalHoodPositionRotations)
        );

        indexerMotorController.setControl(
            indexerDutyCycleRequest.withOutput(indexerEnabled ? HoodConstants.INDEXER_SPEED : 0.0)
        );
    }

    private boolean isWheelVelocityReady(double goalRPS) {
        return Math.abs(leftWheelRPS  - goalRPS) <= HoodConstants.HOOD_SHOOTING_VELOCITY_TOLERANCE_RPS
            && Math.abs(rightWheelRPS - goalRPS) <= HoodConstants.HOOD_SHOOTING_VELOCITY_TOLERANCE_RPS;
    }

    private boolean isHoodAngleReady(double goalRotations) {
        return Math.abs(hoodPositionRotations - goalRotations) <= HoodConstants.HOOD_ANGLE_TOLERANCE_ROT;
    }

    private boolean areWheelsBelowStopThreshold() {
        return Math.abs(leftWheelRPS)  <= HoodConstants.POST_SHOT_WHEEL_STOP_THRESHOLD_ROTATIONS_PER_SECOND
            && Math.abs(rightWheelRPS) <= HoodConstants.POST_SHOT_WHEEL_STOP_THRESHOLD_ROTATIONS_PER_SECOND;
    }

    /** Solo informativo — no bloquea el disparo. */
    private boolean isSafeToShoot() {
        return hoodPositionRotations >= HoodConstants.HOOD_SAFE_MIN_ROTATIONS
            && hoodPositionRotations <= HoodConstants.HOOD_SAFE_MAX_ROTATIONS;
    }

    private void logTelemetry() {
        SmartDashboard.putString ("Hood/State",                 operationalState.toString());
        SmartDashboard.putString ("Hood/ShootingMode",          shootingMode.toString());
        SmartDashboard.putBoolean("Hood/ShootingRequestActive", shootingRequestActive);
        SmartDashboard.putBoolean("Hood/ReadyToShoot",          readyToShoot);
        SmartDashboard.putBoolean("Hood/IndexerEnabled",        indexerEnabled);
        SmartDashboard.putBoolean("Hood/IsSafeToShoot",         isSafeToShoot());

        SmartDashboard.putNumber("Hood/ProjectileAngleDeg",
            Math.toDegrees(desiredProjectileAngleRadians));

        double desiredPhysicalDeg =
            Math.toDegrees(HoodConstants.COMPLEMENTARY_ANGLE - desiredProjectileAngleRadians);
        SmartDashboard.putNumber("Hood/DesiredPhysicalAngleDeg", desiredPhysicalDeg);

        SmartDashboard.putNumber("Hood/CurrentPositionRotations", hoodPositionRotations);
        SmartDashboard.putNumber("Hood/GoalPositionRotations",    goalHoodPositionRotations);
        SmartDashboard.putNumber("Hood/PositionErrorRotations",
            Math.abs(hoodPositionRotations - goalHoodPositionRotations));

        double currentPhysicalDeg =
            (hoodPositionRotations / HoodConstants.CONVERSION_RATIO_RAD_TO_ROT)
            * (180.0 / Math.PI)
            + Math.toDegrees(ShootingConstants.MINIMUM_HOOD_ANGLE_RADIANS);
        SmartDashboard.putNumber("Hood/CurrentPhysicalAngleDeg", currentPhysicalDeg);

        SmartDashboard.putNumber("Hood/LeftWheelRPS",    leftWheelRPS);
        SmartDashboard.putNumber("Hood/RightWheelRPS",   rightWheelRPS);
        SmartDashboard.putNumber("Hood/GoalWheelRPS",    goalWheelRPS);
        SmartDashboard.putNumber("Hood/LeftWheelError",  Math.abs(leftWheelRPS  - goalWheelRPS));
        SmartDashboard.putNumber("Hood/RightWheelError", Math.abs(rightWheelRPS - goalWheelRPS));
        SmartDashboard.putNumber("Hood/DesiredExitSpeedMPS", desiredExitSpeedMetersPerSecond);

        SmartDashboard.putNumber("Hood/HoodAngleCurrent",
            hoodAngleMotorController.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood/LeftWheelCurrent",
            leftWheelMotorController.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood/RightWheelCurrent",
            rightWheelMotorController.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood/IndexerCurrent",
            indexerMotorController.getSupplyCurrent().getValueAsDouble());
    }
}