package frc.robot.Util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.Drive.generated.TunerConstants;

/**
 * ShooterMotorTestCmd — Prueba manual de ruedas e indexer.
 *
 * USO:
 *   1. Ligar este comando a un botón en RobotContainer (whileTrue).
 *   2. En SmartDashboard / Shuffleboard, ajustar los sliders:
 *        "Test/RightWheelDutyCycle"   -1.0 a 1.0
 *        "Test/LeftWheelDutyCycle"    -1.0 a 1.0
 *        "Test/IndexerDutyCycle"      -1.0 a 1.0
 *   3. Activar el botón → los motores responden en tiempo real.
 *   4. Soltar el botón → todo se para.
 *
 * PROPÓSITO:
 *   Aislar si el problema es mecánico o de programación:
 *   - Si los motores giran con este comando → problema en la lógica del HoodSubsystem.
 *   - Si NO giran → problema mecánico, eléctrico o de CAN ID.
 *
 * NOTA: Este comando crea sus propios TalonFX independientes del HoodSubsystem.
 *   Esto evita conflictos de control entre los dos sistemas.
 *   NO usar al mismo tiempo que HoodSubsystem.
 */
public class ShooterMotorTestCmd extends Command {

    // ── Hardware propio del comando de prueba ──────────────────────────
    private final TalonFX rightWheel;
    private final TalonFX leftWheel;
    private final TalonFX indexer;

    // ── Control requests en duty cycle puro (sin PID, sin feedforward) ─
    private final DutyCycleOut rightWheelRequest = new DutyCycleOut(0.95);
    private final DutyCycleOut leftWheelRequest  = new DutyCycleOut(-0.95);
    private final DutyCycleOut indexerRequest    = new DutyCycleOut(0.95);

    // ── Valores leídos desde SmartDashboard ────────────────────────────
    private double rightWheelDutyCycle = 0.0;
    private double leftWheelDutyCycle  = 0.0;
    private double indexerDutyCycle    = 0.0;

    // ══════════════════════════════════════════════════════════════════
    // CONSTRUCTORES
    // ══════════════════════════════════════════════════════════════════

    public ShooterMotorTestCmd() {
        this(TunerConstants.CONTROLLER_AREA_NETWORK_BUS);
    }

    public ShooterMotorTestCmd(CANBus canBus) {
        rightWheel = new TalonFX(HoodConstants.RIGHT_HOOD_PROPULSION_TALON_ID, canBus);
        leftWheel  = new TalonFX(HoodConstants.LEFT_HOOD_PROPULSION_TALON_ID,  canBus);
        indexer    = new TalonFX(HoodConstants.INDEXER_TALON_ID,               canBus);

        configureMotors();

        // Publicar entradas en SmartDashboard con valores iniciales en 0
        SmartDashboard.putNumber("Test/RightWheelDutyCycle", 0.0);
        SmartDashboard.putNumber("Test/LeftWheelDutyCycle",  0.0);
        SmartDashboard.putNumber("Test/IndexerDutyCycle",    0.0);

        // Este comando NO requiere ningún subsistema intencionalmente —
        // opera directamente sobre el hardware para pruebas de diagnóstico.
        // Si quieres evitar conflictos con HoodSubsystem, NO correr ambos al mismo tiempo.
    }

    // ══════════════════════════════════════════════════════════════════
    // CONFIGURACIÓN — mínima, solo lo necesario para mover los motores
    // ══════════════════════════════════════════════════════════════════

    private void configureMotors() {
        // Rueda DERECHA
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(HoodConstants.RIGHT_WHEEL_INVERTED);
        rightConfig.CurrentLimits = safeCurrentLimits();
        rightWheel.getConfigurator().apply(rightConfig);

        // Rueda IZQUIERDA — dirección opuesta
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(HoodConstants.LEFT_WHEEL_INVERTED);
        leftConfig.CurrentLimits = safeCurrentLimits();
        leftWheel.getConfigurator().apply(leftConfig);

        // Indexer
        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
        indexerConfig.MotorOutput = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);
        indexerConfig.CurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(30.0)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(40.0)
            .withStatorCurrentLimitEnable(true);
        indexer.getConfigurator().apply(indexerConfig);

        System.out.println("✅ [ShooterMotorTestCmd] Motores configurados para prueba");
        System.out.println("   Ajusta los sliders en SmartDashboard:");
        System.out.println("   → Test/RightWheelDutyCycle");
        System.out.println("   → Test/LeftWheelDutyCycle");
        System.out.println("   → Test/IndexerDutyCycle");
    }

    /**
     * Límites de corriente conservadores para pruebas.
     * Más bajos que producción para proteger si algo está mal mecánicamente.
     */
    private CurrentLimitsConfigs safeCurrentLimits() {
        return new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40.0)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(60.0)
            .withStatorCurrentLimitEnable(true);
    }

    // ══════════════════════════════════════════════════════════════════
    // CICLO DEL COMANDO
    // ══════════════════════════════════════════════════════════════════

    @Override
    public void initialize() {
        System.out.println("▶ [ShooterMotorTestCmd] INICIADO — ajusta los sliders en SmartDashboard");
        stopAllMotors();
    }

    @Override
    public void execute() {
        // Leer valores del SmartDashboard en cada ciclo
        rightWheelDutyCycle = SmartDashboard.getNumber("Test/RightWheelDutyCycle", 0.0);
        leftWheelDutyCycle  = SmartDashboard.getNumber("Test/LeftWheelDutyCycle",  0.0);
        indexerDutyCycle    = SmartDashboard.getNumber("Test/IndexerDutyCycle",    0.0);

        // Clamp de seguridad: nunca exceder -1.0 a 1.0
        rightWheelDutyCycle = clamp(rightWheelDutyCycle, -1.0, 1.0);
        leftWheelDutyCycle  = clamp(leftWheelDutyCycle,  -1.0, 1.0);
        indexerDutyCycle    = clamp(indexerDutyCycle,    -1.0, 1.0);

        // Aplicar a los motores
        rightWheel.setControl(rightWheelRequest.withOutput(rightWheelDutyCycle));
        leftWheel.setControl(leftWheelRequest.withOutput(leftWheelDutyCycle));
        indexer.setControl(indexerRequest.withOutput(indexerDutyCycle));

        // Telemetría de diagnóstico
        publishTelemetry();
    }

    @Override
    public void end(boolean interrupted) {
        stopAllMotors();
        // Resetear sliders a 0 al terminar
        SmartDashboard.putNumber("Test/RightWheelDutyCycle", 0.0);
        SmartDashboard.putNumber("Test/LeftWheelDutyCycle",  0.0);
        SmartDashboard.putNumber("Test/IndexerDutyCycle",    0.0);
        System.out.println("⏹ [ShooterMotorTestCmd] DETENIDO — motores apagados");
    }

    @Override
    public boolean isFinished() {
        return false; // Corre hasta que se suelte el botón (whileTrue)
    }

    // ══════════════════════════════════════════════════════════════════
    // HELPERS
    // ══════════════════════════════════════════════════════════════════

    private void stopAllMotors() {
        rightWheel.setControl(rightWheelRequest.withOutput(0.0));
        leftWheel.setControl(leftWheelRequest.withOutput(0.0));
        indexer.setControl(indexerRequest.withOutput(0.0));
    }

    private void publishTelemetry() {
        // Comandos enviados
        SmartDashboard.putNumber("Test/RightWheel_CMD",  rightWheelDutyCycle);
        SmartDashboard.putNumber("Test/LeftWheel_CMD",   leftWheelDutyCycle);
        SmartDashboard.putNumber("Test/Indexer_CMD",     indexerDutyCycle);

        // Velocidades reales (para verificar que el motor responde)
        SmartDashboard.putNumber("Test/RightWheel_RPS",
            rightWheel.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Test/LeftWheel_RPS",
            leftWheel.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Test/Indexer_RPS",
            indexer.getVelocity().getValueAsDouble());

        // Corrientes (para detectar atascos mecánicos — corriente alta sin movimiento)
        SmartDashboard.putNumber("Test/RightWheel_SupplyA",
            rightWheel.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Test/LeftWheel_SupplyA",
            leftWheel.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Test/Indexer_SupplyA",
            indexer.getSupplyCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Test/RightWheel_StatorA",
            rightWheel.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Test/LeftWheel_StatorA",
            leftWheel.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Test/Indexer_StatorA",
            indexer.getStatorCurrent().getValueAsDouble());

        // Voltaje aplicado (para verificar que el controlador está respondiendo)
        SmartDashboard.putNumber("Test/RightWheel_MotorV",
            rightWheel.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Test/LeftWheel_MotorV",
            leftWheel.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Test/Indexer_MotorV",
            indexer.getMotorVoltage().getValueAsDouble());
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}