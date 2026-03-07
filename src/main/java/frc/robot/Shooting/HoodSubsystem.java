package frc.robot.Shooting;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

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
 * HoodSubsystem - VERSIÓN FINAL CORREGIDA
 * 
 * SISTEMA DE TIRO DUAL:
 * 1. DEFAULT: Valores fijos de Constants (sin visión válida)
 * 2. CALCULADO: Modelo matemático de ShootingHelper (con visión válida)
 * 
 * CONDICIONES PARA TIRO CALCULADO:
 * - visionSubsystem.hasTarget() = true
 * - visionSubsystem.isShootingTargetValid() = true
 * - shootingHelper.isPossibleToShoot() = true
 * 
 * Si cualquiera falla → FALLBACK a DEFAULT
 */
public class HoodSubsystem extends SubsystemBase {

    /**
     * Modo de tiro actualmente en uso
     */
    public enum ShootingMode {
        DEFAULT,      // Usando valores fijos de Constants
        CALCULATED    // Usando modelo matemático con visión
    }

    private enum HoodOperationalState {
        STOWED,
        AIMING_AND_SPINNING,
        POST_SHOT_COASTING,
        STOWING
    }

    // ========== HARDWARE ==========
    private final TalonFX hoodAngleMotorController;
    private final TalonFX rightWheelMotorController;
    private final TalonFX leftWheelMotorController;
    private final TalonFX indexerMotorController;

    // ========== CONTROL REQUESTS ==========
    private final VelocityTorqueCurrentFOC rightWheelVelocityControlRequest =
        new VelocityTorqueCurrentFOC(0.0);

    private final VelocityTorqueCurrentFOC leftWheelVelocityControlRequest =
        new VelocityTorqueCurrentFOC(0.0);

    private final MotionMagicExpoTorqueCurrentFOC hoodAnglePositionControlRequest =
        new MotionMagicExpoTorqueCurrentFOC(0.0);

    private final DutyCycleOut indexerDutyCycleControlRequest =
        new DutyCycleOut(0.0);

    // ========== STATE MANAGEMENT ==========
    private final Debouncer readyDebouncerSeconds =
        new Debouncer(0.12, Debouncer.DebounceType.kRising);

    private HoodOperationalState hoodOperationalState = HoodOperationalState.STOWED;
    private ShootingMode currentShootingMode = ShootingMode.DEFAULT;

    private boolean shootingRequestActive = false;
    private boolean readyToShoot = false;

    // ========== SHOOTING PARAMETERS ==========
    private double desiredExitSpeedMetersPerSecond =
        ShootingConstants.DEFAULT_EXIT_SPEED_METERS_PER_SECOND;

    private double desiredHoodAngleRadiansFromModel =
        ShootingConstants.DEFAULT_HOOD_ANGLE_RADIANS;

    // ========== COAST STATE TRACKING ==========
    private double postShotCoastStartTimestampSeconds = 0.0;
    private double hoodHoldPositionRotations = 0.0;

    // ========== GOAL SETPOINTS ==========
    private double goalWheelRotationsPerSecond = 0.0;
    private double goalHoodAnglePositionRotations = 0.0;
    private boolean indexerEnabled = false;

    // ========== SENSOR READINGS ==========
    private double hoodAnglePositionRotations = 0.0;
    private double leftWheelVelocityRotationsPerSecond = 0.0;
    private double rightWheelVelocityRotationsPerSecond = 0.0;

    // ========== CONSTRUCTOR ==========
    public HoodSubsystem() {
        this(TunerConstants.CONTROLLER_AREA_NETWORK_BUS);
    }

    public HoodSubsystem(CANBus canBusName) {
        // Crear instancias de TalonFX
        hoodAngleMotorController = new TalonFX(HoodConstants.HOOD_ANGLE_TALON_ID, canBusName);
        rightWheelMotorController = new TalonFX(HoodConstants.RIGHT_HOOD_PROPULSION_TALON_ID, canBusName);
        leftWheelMotorController = new TalonFX(HoodConstants.LEFT_HOOD_PROPULSION_TALON_ID, canBusName);
        indexerMotorController = new TalonFX(HoodConstants.INDEXER_TALON_ID, canBusName);

        // Configurar todos los motores con configuraciones COMPLETAS
        configureHoodAngleMotor();
        configureWheelMotors();
        configureIndexerMotor();
        
        System.out.println("✅ [HoodSubsystem] Inicialización completa");
        System.out.println("   CONVERSION_RATIO_RAD_TO_ROT: " + HoodConstants.CONVERSION_RATIO_RAD_TO_ROT);
        System.out.println("   Hood kP: 50.0, kG: 0.25, kD: 0.5");
        System.out.println("   Motion Magic cruise: 300 RPS");
        System.out.println("   Brake mode: ACTIVADO");
    }

    /**
     * Configurar hood angle motor con TODAS las configuraciones
     * 
     * ⚠️ CRÍTICO: Sin Motion Magic, el motor NO se mueve
     * ⚠️ CRÍTICO: Sin Brake mode, el motor NO sostiene contra gravedad
     */
    private void configureHoodAngleMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // 1. PID y Feedforward
        config.Slot0 = HoodConstants.HOOD_ANGLE_SLOT_CONFIGS;

        // 2. Motion Magic - CRÍTICO
        config.MotionMagic = HoodConstants.HOOD_MOTION_MAGIC_CONFIGS;

        // 3. Motor Output - CRÍTICO
        config.MotorOutput = HoodConstants.HOOD_ANGLE_MOTOR_OUTPUT_CONFIGS;

        // 4. Current Limits
        config.CurrentLimits = HoodConstants.HOOD_ANGLE_CURRENT_LIMITS;

        // 5. Software Limits
        config.SoftwareLimitSwitch = HoodConstants.HOOD_SOFTWARE_LIMITS;

        // Aplicar configuración completa
        var status = hoodAngleMotorController.getConfigurator().apply(config);
        
        if (!status.isOK()) {
            System.err.println("❌ [HoodSubsystem] ERROR configurando hood angle motor: " + status);
        } else {
            System.out.println("✅ [HoodSubsystem] Hood angle motor configurado");
        }
    }

    /**
     * Configurar wheel motors
     */
    private void configureWheelMotors() {
        // Rueda DERECHA
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.Slot0 = HoodConstants.RIGHT_HOOD_PROPULSION_SLOT_CONFIGS;
        rightConfig.MotorOutput = HoodConstants.WHEEL_MOTOR_OUTPUT_CONFIGS;
        rightConfig.CurrentLimits = HoodConstants.WHEEL_CURRENT_LIMITS;
        
        var rightStatus = rightWheelMotorController.getConfigurator().apply(rightConfig);
        
        // Rueda IZQUIERDA (invertida)
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.Slot0 = HoodConstants.LEFT_HOOD_PROPULSION_SLOT_CONFIGS;
        leftConfig.MotorOutput = HoodConstants.WHEEL_MOTOR_OUTPUT_CONFIGS;
        leftConfig.CurrentLimits = HoodConstants.WHEEL_CURRENT_LIMITS;
        
        // Invertir rueda izquierda
        leftConfig.MotorOutput.Inverted = 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
        
        var leftStatus = leftWheelMotorController.getConfigurator().apply(leftConfig);
        
        if (!rightStatus.isOK() || !leftStatus.isOK()) {
            System.err.println("❌ [HoodSubsystem] ERROR configurando wheel motors");
        } else {
            System.out.println("✅ [HoodSubsystem] Wheel motors configurados");
        }
    }

    /**
     * Configurar indexer motor
     */
    private void configureIndexerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0 = HoodConstants.INDEXER_SLOT_CONFIGS;
        config.MotorOutput = HoodConstants.INDEXER_MOTOR_OUTPUT_CONFIGS;
        config.CurrentLimits = HoodConstants.INDEXER_CURRENT_LIMITS;
        
        var status = indexerMotorController.getConfigurator().apply(config);
        
        if (!status.isOK()) {
            System.err.println("❌ [HoodSubsystem] ERROR configurando indexer motor");
        } else {
            System.out.println("✅ [HoodSubsystem] Indexer motor configurado");
        }
    }

    // ========================================
    // MAIN PERIODIC LOOP
    // ========================================
    
    @Override
    public void periodic() {
        // 1. Leer sensores
        readHardwareSensors();

        // 2. Manejar estado deshabilitado
        if (DriverStation.isDisabled()) {
            applyDisabledOutputs();
            readyToShoot = false;
            currentShootingMode = ShootingMode.DEFAULT;
            return;
        }

        // 3. Actualizar transiciones de estado
        updateOperationalStateTransitions();

        // 4. Ejecutar lógica del estado actual
        executeStateLogic();

        // 5. Enviar telemetría
        logTelemetry();

        // 6. Aplicar outputs a los motores
        applyEnabledOutputs();
    }

    /**
     * Ejecutar lógica específica del estado actual
     */
    private void executeStateLogic() {
        switch (hoodOperationalState) {
            case AIMING_AND_SPINNING -> executeAimingState();
            case POST_SHOT_COASTING -> executeCoastingState();
            case STOWING -> executeStowingState();
            case STOWED -> executeStowedState();
        }
    }

    /**
     * Estado AIMING_AND_SPINNING:
     * - Calcular velocidad de ruedas
     * - Calcular ángulo del hood (usando ángulo complementario)
     * - Verificar si está listo para disparar
     * - Activar indexer cuando esté listo
     */
    private void executeAimingState() {
        // Calcular velocidad de ruedas
        goalWheelRotationsPerSecond =
            desiredExitSpeedMetersPerSecond * HoodConstants.CONVERSION_RATIO_FROM_METERS_TO_RPS;

        // ========================================
        // CÁLCULO DEL ÁNGULO DEL HOOD
        // ========================================
        
        // 1. Ángulo de TRAYECTORIA del proyectil (desde ShootingHelper)
        // Ejemplo: 45° para tiro parabólico
        
        // 2. Calcular ángulo FÍSICO del hood (complementario)
        // hood_angle = 90° - trayectoria_angle
        double complementaryAngleRadians =
            HoodConstants.COMPLEMENTARY_ANGLE - desiredHoodAngleRadiansFromModel;

        // 3. Restar offset para obtener ángulo RELATIVO al mínimo (24°)
        // hood_relative = hood_physical - 24°
        double correctedAngleRadians =
            complementaryAngleRadians - HoodConstants.HOOD_ANGLE_OFFSET_RADIANS;

        // 4. Convertir a rotaciones del encoder
        // rotations = radianes * 218.76
        goalHoodAnglePositionRotations =
            correctedAngleRadians * HoodConstants.CONVERSION_RATIO_RAD_TO_ROT;

        // Verificar si está listo (con debouncer)
        boolean rawReady =
            isWheelVelocityReady(goalWheelRotationsPerSecond) &&
            isHoodAngleReady(goalHoodAnglePositionRotations) &&
            isSafeToShoot();

        readyToShoot = readyDebouncerSeconds.calculate(rawReady);

        // Activar indexer SOLO cuando está listo Y shooting request activo
        indexerEnabled = shootingRequestActive && readyToShoot;
    }

    /**
     * Estado POST_SHOT_COASTING:
     * - Desactivar indexer inmediatamente
     * - Desacelerar ruedas gradualmente
     * - Mantener posición del hood
     * - Transicionar a STOWING cuando sea apropiado
     */
    private void executeCoastingState() {
        indexerEnabled = false;
        goalWheelRotationsPerSecond = 0.0;
        goalHoodAnglePositionRotations = hoodHoldPositionRotations;

        double currentTimestampSeconds = Timer.getFPGATimestamp();
        double elapsedSeconds = currentTimestampSeconds - postShotCoastStartTimestampSeconds;

        boolean hasMinimumTimeElapsed =
            elapsedSeconds >= HoodConstants.POST_SHOT_COAST_MINIMUM_TIME_SECONDS;

        boolean hasMaximumTimeElapsed =
            elapsedSeconds >= HoodConstants.POST_SHOT_COAST_MAXIMUM_TIME_SECONDS;

        boolean wheelsAreSlowEnough = areWheelsBelowStopThreshold();

        // Transicionar a STOWING
        if ((hasMinimumTimeElapsed && wheelsAreSlowEnough) || hasMaximumTimeElapsed) {
            hoodOperationalState = HoodOperationalState.STOWING;
        }

        readyToShoot = false;
    }

    /**
     * Estado STOWING:
     * - Regresar hood a posición home
     * - Parar ruedas completamente
     * - Transicionar a STOWED cuando llegue a home
     */
    private void executeStowingState() {
        indexerEnabled = false;
        goalWheelRotationsPerSecond = 0.0;
        goalHoodAnglePositionRotations = HoodConstants.HOOD_HOME_POSITION_ROTATIONS;

        readyToShoot = false;

        // Transicionar a STOWED cuando llegue a home
        if (isHoodAngleReady(HoodConstants.HOOD_HOME_POSITION_ROTATIONS)) {
            hoodOperationalState = HoodOperationalState.STOWED;
            currentShootingMode = ShootingMode.DEFAULT;  // Reset a DEFAULT
        }
    }

    /**
     * Estado STOWED:
     * - Todo apagado y en posición home
     * - Esperando siguiente comando
     */
    private void executeStowedState() {
        indexerEnabled = false;
        goalWheelRotationsPerSecond = 0.0;
        goalHoodAnglePositionRotations = HoodConstants.HOOD_HOME_POSITION_ROTATIONS;
        readyToShoot = false;
        currentShootingMode = ShootingMode.DEFAULT;
    }

    // ========================================
    // PUBLIC API
    // ========================================
    
    /**
     * Activar/desactivar shooting request
     */
    public void setShootingRequestActive(boolean isActive) {
        shootingRequestActive = isActive;
    }

    /**
     * Actualizar solución de tiro
     * 
     * LÓGICA DE SELECCIÓN:
     * 1. Verificar si hay visión válida
     * 2. Verificar si el tiro es posible dentro de restricciones
     * 3. Si TODO OK → usar CALCULADO
     * 4. Si CUALQUIER cosa falla → usar DEFAULT
     */
    public void updateShootingSolution(VisionSubsystem visionSubsystem, ShootingHelper shootingHelper) {
        // Si no hay shooting request, no actualizar
        if (!shootingRequestActive) {
            return;
        }

        // ========================================
        // EVALUACIÓN DE CONDICIONES PARA TIRO CALCULADO
        // ========================================
        
        // Condición 1: ¿Hay un target detectado?
        boolean hasTarget = visionSubsystem.hasTarget();
        
        // Condición 2: ¿Es un target válido para shooting?
        boolean targetIsValid = visionSubsystem.isShootingTargetValid();
        
        // Condición 3: ¿El tiro es físicamente posible?
        boolean shootIsPossible = shootingHelper.isPossibleToShoot();
        
        // Todas las condiciones deben cumplirse
        boolean shouldUseLiveSolution = hasTarget && targetIsValid && shootIsPossible;

        // ========================================
        // SELECCIÓN DE MODO DE TIRO
        // ========================================
        
        if (shouldUseLiveSolution) {
            // ✅ MODO CALCULADO: Usar modelo matemático
            currentShootingMode = ShootingMode.CALCULATED;
            
            desiredExitSpeedMetersPerSecond = shootingHelper.getExitSpeedMetersPerSecond();
            desiredHoodAngleRadiansFromModel = shootingHelper.getHoodAngleRadians();
            
            // Log para debugging
            SmartDashboard.putString("Hood/ShootingModeReason", 
                "CALCULATED - Vision OK, Target Valid, Shot Possible");
            
        } else {
            // ❌ MODO DEFAULT: Fallback a valores fijos
            currentShootingMode = ShootingMode.DEFAULT;
            
            desiredExitSpeedMetersPerSecond = ShootingConstants.DEFAULT_EXIT_SPEED_METERS_PER_SECOND;
            desiredHoodAngleRadiansFromModel = ShootingConstants.DEFAULT_HOOD_ANGLE_RADIANS;
            
            // Log razón del fallback
            String reason;
            if (!hasTarget) {
                reason = "DEFAULT - No vision target detected";
            } else if (!targetIsValid) {
                reason = "DEFAULT - Target not valid for shooting";
            } else if (!shootIsPossible) {
                reason = "DEFAULT - Shot not possible (out of restrictions)";
            } else {
                reason = "DEFAULT - Unknown reason";
            }
            SmartDashboard.putString("Hood/ShootingModeReason", reason);
        }
    }

    /**
     * Verificar si está listo para disparar
     */
    public boolean isReadyToShoot() {
        return readyToShoot;
    }

    /**
     * Obtener modo de tiro actual
     */
    public ShootingMode getCurrentShootingMode() {
        return currentShootingMode;
    }

    /**
     * Detener todo el sistema
     */
    public void stop() {
        shootingRequestActive = false;
        hoodOperationalState = HoodOperationalState.STOWED;

        goalWheelRotationsPerSecond = 0.0;
        goalHoodAnglePositionRotations = HoodConstants.HOOD_HOME_POSITION_ROTATIONS;
        indexerEnabled = false;

        readyToShoot = false;
        currentShootingMode = ShootingMode.DEFAULT;
        
        applyDisabledOutputs();
    }

    /**
     * Resetear encoder del hood a cero
     * Llamar cuando el hood esté en posición mínima (24°)
     */
    public void zeroHoodPosition() {
        hoodAngleMotorController.setPosition(0.0);
        System.out.println("✅ [HoodSubsystem] Hood position zeroed");
    }

    // ========================================
    // PRIVATE HELPERS
    // ========================================
    
    /**
     * Actualizar transiciones de estado
     */
    private void updateOperationalStateTransitions() {
        // Si shooting request activo, ir a AIMING_AND_SPINNING
        if (shootingRequestActive) {
            if (hoodOperationalState != HoodOperationalState.AIMING_AND_SPINNING) {
                hoodOperationalState = HoodOperationalState.AIMING_AND_SPINNING;
                readyDebouncerSeconds.calculate(false);  // Reset debouncer
            }
            return;
        }

        // Si estábamos AIMING y shooting se desactivó, ir a COASTING
        if (hoodOperationalState == HoodOperationalState.AIMING_AND_SPINNING) {
            hoodOperationalState = HoodOperationalState.POST_SHOT_COASTING;
            postShotCoastStartTimestampSeconds = Timer.getFPGATimestamp();
            hoodHoldPositionRotations = hoodAnglePositionRotations;
            readyToShoot = false;
        }
    }

    /**
     * Leer todos los sensores
     */
    private void readHardwareSensors() {
        hoodAnglePositionRotations = hoodAngleMotorController.getPosition().getValueAsDouble();
        leftWheelVelocityRotationsPerSecond = leftWheelMotorController.getVelocity().getValueAsDouble();
        rightWheelVelocityRotationsPerSecond = rightWheelMotorController.getVelocity().getValueAsDouble();
    }

    /**
     * Aplicar outputs cuando robot está deshabilitado
     */
    private void applyDisabledOutputs() {
        rightWheelMotorController.setControl(rightWheelVelocityControlRequest.withVelocity(0.0));
        leftWheelMotorController.setControl(leftWheelVelocityControlRequest.withVelocity(0.0));
        indexerMotorController.setControl(indexerDutyCycleControlRequest.withOutput(0.0));

        // Mantener posición actual
        double currentHoodAnglePositionRotations = hoodAngleMotorController.getPosition().getValueAsDouble();
        hoodAngleMotorController.setControl(
            hoodAnglePositionControlRequest.withPosition(currentHoodAnglePositionRotations)
        );
    }

    /**
     * Aplicar outputs cuando robot está habilitado
     */
    private void applyEnabledOutputs() {
        // Ruedas (inversión configurada en motor output)
        rightWheelMotorController.setControl(
            rightWheelVelocityControlRequest.withVelocity(goalWheelRotationsPerSecond)
        );

        leftWheelMotorController.setControl(
            leftWheelVelocityControlRequest.withVelocity(goalWheelRotationsPerSecond)
        );

        // Hood angle
        hoodAngleMotorController.setControl(
            hoodAnglePositionControlRequest.withPosition(goalHoodAnglePositionRotations)
        );

        // Indexer
        double requestedDutyCycle = indexerEnabled ? HoodConstants.INDEXER_SPEED : 0.0;
        indexerMotorController.setControl(indexerDutyCycleControlRequest.withOutput(requestedDutyCycle));
    }

    /**
     * Verificar si las ruedas están en velocidad objetivo
     */
    private boolean isWheelVelocityReady(double goalWheelRotationsPerSecond) {
        double leftError = Math.abs(leftWheelVelocityRotationsPerSecond - goalWheelRotationsPerSecond);
        double rightError = Math.abs(rightWheelVelocityRotationsPerSecond - goalWheelRotationsPerSecond);

        return leftError <= HoodConstants.HOOD_SHOOTING_VELOCITY_TOLERANCE_RPS
            && rightError <= HoodConstants.HOOD_SHOOTING_VELOCITY_TOLERANCE_RPS;
    }

    /**
     * Verificar si el hood está en ángulo objetivo
     */
    private boolean isHoodAngleReady(double goalHoodAnglePositionRotations) {
        double errorRotations = Math.abs(hoodAnglePositionRotations - goalHoodAnglePositionRotations);
        return errorRotations <= HoodConstants.HOOD_ANGLE_TOLERANCE_ROT;
    }

    /**
     * Verificar si las ruedas están suficientemente lentas
     */
    private boolean areWheelsBelowStopThreshold() {
        double leftAbsoluteRotationsPerSecond = Math.abs(leftWheelVelocityRotationsPerSecond);
        double rightAbsoluteRotationsPerSecond = Math.abs(rightWheelVelocityRotationsPerSecond);

        return leftAbsoluteRotationsPerSecond <= HoodConstants.POST_SHOT_WHEEL_STOP_THRESHOLD_ROTATIONS_PER_SECOND
            && rightAbsoluteRotationsPerSecond <= HoodConstants.POST_SHOT_WHEEL_STOP_THRESHOLD_ROTATIONS_PER_SECOND;
    }

    /**
     * Verificar si es seguro disparar
     */
    private boolean isSafeToShoot() {
        // Verificar que el hood esté dentro del rango seguro
        return hoodAnglePositionRotations >= -5.0
            && hoodAnglePositionRotations <= 132.3;
    }

    /**
     * Logging de telemetría para debugging
     */
    private void logTelemetry() {
        // Estado del sistema
        SmartDashboard.putString("Hood/State", hoodOperationalState.toString());
        SmartDashboard.putString("Hood/ShootingMode", currentShootingMode.toString());
        SmartDashboard.putBoolean("Hood/ShootingRequestActive", shootingRequestActive);
        SmartDashboard.putBoolean("Hood/ReadyToShoot", readyToShoot);
        SmartDashboard.putBoolean("Hood/IndexerEnabled", indexerEnabled);
        SmartDashboard.putBoolean("Hood/SafeToShoot", isSafeToShoot());
        
        // Ángulos
        SmartDashboard.putNumber("Hood/DesiredTrajectoryAngleRadians", desiredHoodAngleRadiansFromModel);
        SmartDashboard.putNumber("Hood/DesiredTrajectoryAngleDegrees", 
            Math.toDegrees(desiredHoodAngleRadiansFromModel));
        SmartDashboard.putNumber("Hood/CurrentAngleRotations", hoodAnglePositionRotations);
        SmartDashboard.putNumber("Hood/GoalAngleRotations", goalHoodAnglePositionRotations);
        SmartDashboard.putNumber("Hood/AngleErrorRotations", 
            Math.abs(hoodAnglePositionRotations - goalHoodAnglePositionRotations));
        
        // Convertir rotaciones a ángulo físico
        double currentPhysicalAngleDegrees = 
            (hoodAnglePositionRotations / HoodConstants.CONVERSION_RATIO_RAD_TO_ROT) * (180.0 / Math.PI) + 24.0;
        SmartDashboard.putNumber("Hood/CurrentPhysicalAngleDegrees", currentPhysicalAngleDegrees);
        
        // Ruedas
        SmartDashboard.putNumber("Hood/LeftWheelRPS", leftWheelVelocityRotationsPerSecond);
        SmartDashboard.putNumber("Hood/RightWheelRPS", rightWheelVelocityRotationsPerSecond);
        SmartDashboard.putNumber("Hood/GoalWheelRPS", goalWheelRotationsPerSecond);
        SmartDashboard.putNumber("Hood/LeftWheelError", 
            Math.abs(leftWheelVelocityRotationsPerSecond - goalWheelRotationsPerSecond));
        SmartDashboard.putNumber("Hood/RightWheelError", 
            Math.abs(rightWheelVelocityRotationsPerSecond - goalWheelRotationsPerSecond));
        
        SmartDashboard.putNumber("Hood/DesiredExitSpeedMPS", desiredExitSpeedMetersPerSecond);
        
        // Corrientes
        SmartDashboard.putNumber("Hood/HoodAngleSupplyCurrent", 
            hoodAngleMotorController.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood/LeftWheelSupplyCurrent", 
            leftWheelMotorController.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood/RightWheelSupplyCurrent", 
            rightWheelMotorController.getSupplyCurrent().getValueAsDouble());
    }
}