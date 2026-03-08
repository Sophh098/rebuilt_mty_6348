package frc.robot;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Vision.VisionEntries;
import frc.robot.Vision.VisionEnums;

public class Constants {

    public static final double BATTERY_VOLTAGE = 12.0;

    // ════════════════════════════════════════════════════════════
    // DRIVE CONSTANTS
    // ════════════════════════════════════════════════════════════
    public static final class DriveConstants {
        public static final Mode SIM_MODE    = Mode.SIM;
        public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

        public static enum Mode { REAL, SIM, REPLAY }

        public static final double  ROBOT_MASS_KG  = 50.0;
        public static final double  ROBOT_MOI       = 4.8790225;
        public static final double  WHEEL_COF       = 1.2;

        public static final double  STEER_GAINS_KP = 100;
        public static final double  STEER_GAINS_KI = 0.0;
        public static final double  STEER_GAINS_KD = 0.5;
        public static final double  STEER_GAINS_KS = 0.1;
        public static final double  STEER_GAINS_KV = 1.91;
        public static final double  STEER_GAINS_KA = 0.0;

        public static final double  DRIVE_GAINS_KP = 0.1;
        public static final double  DRIVE_GAINS_KI = 0.0;
        public static final double  DRIVE_GAINS_KD = 0.0;
        public static final double  DRIVE_GAINS_KS = 0.0;
        public static final double  DRIVE_GAINS_KV = 0.124;
        public static final double  DRIVE_GAINS_KA = 0.0;

        public static final Current kSlipCurrent              = Amps.of(60.0);
        public static final Current SteeringStatorCurrentLimit = Amps.of(60.0);

        public static final String CAN_BUS        = "6348 Horus CANivore";
        public static final String HOOT_FILE_PATH = "./logs/example.hoot";

        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(3.79);

        public static final double   K_COUPLE_RATIO      = 3.5714285714285716;
        public static final double   K_DRIVE_GEAR_RATIO  = 8.142857142857142;
        public static final double   K_STEER_GEAR_RATIO  = 21.428571428571427;
        public static final Distance kWheelRadius        = Inches.of(2.0);

        public static final boolean K_INVERT_LEFT_SIDE   = false;
        public static final boolean K_INVERT_RIGHT_SIDE  = true;
        public static final int     K_PIGEON_ID          = 13;

        public static final MomentOfInertia kDriveInertia        = KilogramSquareMeters.of(0.01);
        public static final MomentOfInertia kSteerInertia        = KilogramSquareMeters.of(0.01);
        public static final Voltage         kSteerFrictionVoltage = Volts.of(0.2);
        public static final Voltage         kDriveFrictionVoltage = Volts.of(0.2);

        public static final Distance kTrackWidth = Inches.of(23.5);
        public static final Distance kWheelBase  = Inches.of(23.5);

        public static final int     K_FRONT_LEFT_DRIVE_MOTOR_ID      = 10;
        public static final int     K_FRONT_LEFT_STEER_MOTOR_ID      = 6;
        public static final int     K_FRONT_LEFT_ENCODER_ID          = 2;
        public static final Angle   kFrontLeftEncoderOffset          = Rotations.of(0.27197265625);
        public static final boolean K_FRONT_LEFT_STEER_MOTOR_INVERTED = true;
        public static final boolean K_FRONT_LEFT_ENCODER_INVERTED    = false;
        public static final Distance kFrontLeftXPos = kWheelBase.div(2);
        public static final Distance kFrontLeftYPos = kTrackWidth.div(2);

        public static final int     K_FRONT_RIGHT_DRIVE_MOTOR_ID      = 9;
        public static final int     K_FRONT_RIGHT_STEER_MOTOR_ID      = 5;
        public static final int     K_FRONT_RIGHT_ENCODER_ID          = 1;
        public static final Angle   kFrontRightEncoderOffset          = Rotations.of(-0.30810546875);
        public static final boolean K_FRONT_RIGHT_STEER_MOTOR_INVERTED = true;
        public static final boolean K_FRONT_RIGHT_ENCODER_INVERTED    = false;
        public static final Distance kFrontRightXPos = kWheelBase.div(2);
        public static final Distance kFrontRightYPos = kTrackWidth.div(-2);

        public static final int     K_BACK_LEFT_DRIVE_MOTOR_ID      = 11;
        public static final int     K_BACK_LEFT_STEER_MOTOR_ID      = 7;
        public static final int     K_BACK_LEFT_ENCODER_ID          = 3;
        public static final Angle   kBackLeftEncoderOffset          = Rotations.of(-0.41455078125);
        public static final boolean K_BACK_LEFT_STEER_MOTOR_INVERTED = true;
        public static final boolean K_BACK_LEFT_ENCODER_INVERTED    = false;
        public static final Distance kBackLeftXPos = kWheelBase.div(-2);
        public static final Distance kBackLeftYPos = kTrackWidth.div(2);

        public static final int     K_BACK_RIGHT_DRIVE_MOTOR_ID      = 12;
        public static final int     K_BACK_RIGHT_STEER_MOTOR_ID      = 8;
        public static final int     K_BACK_RIGHT_ENCODER_ID          = 4;
        public static final Angle   kBackRightEncoderOffset          = Rotations.of(-0.14013671875);
        public static final boolean K_BACK_RIGHT_STEER_MOTOR_INVERTED = true;
        public static final boolean K_BACK_RIGHT_ENCODER_INVERTED    = false;
        public static final Distance kBackRightXPos = kWheelBase.div(-2);
        public static final Distance kBackRightYPos = kTrackWidth.div(-2);

        public static final double DEADBAND               = 0.1;
        public static final double ANGLE_KP               = 5.0;
        public static final double ANGLE_KD               = 0.4;
        public static final double ANGLE_MAX_VELOCITY     = 8.0;
        public static final double ANGLE_MAX_ACCELERATION = 20.0;
        public static final double FF_START_DELAY         = 2.0;
        public static final double FF_RAMP_RATE           = 0.1;
        public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25;
        public static final double WHEEL_RADIUS_RAMP_RATE    = 0.05;
    }

    // ════════════════════════════════════════════════════════════
    // AUTO CONSTANTS
    // ════════════════════════════════════════════════════════════
    public static final class AutoConstants {
        public static final double TRANSLATION_KP = 5.0;
        public static final double TRANSLATION_KI = 0.0;
        public static final double TRANSLATION_KD = 0.0;
        public static final double ROTATION_KP    = 5.0;
        public static final double ROTATION_KI    = 0.0;
        public static final double ROTATION_KD    = 0.0;
    }

    // ════════════════════════════════════════════════════════════
    // SHOOTING CONSTANTS
    // ════════════════════════════════════════════════════════════
    public static final class ShootingConstants {

        // Posiciones del funnel y pases (sin cambios)
        public static final double PASSING_SHOOT_RIGHT_RED_ALLIANCE_WELDED_X_METERS  = 14.228191;
        public static final double PASSING_SHOOT_RIGHT_RED_ALLIANCE_WELDED_Y_METERS  = 1.344887667;
        public static final double PASSING_SHOOT_RIGHT_BLUE_ALLIANCE_WELDED_X_METERS = 2.312797;
        public static final double PASSING_SHOOT_RIGHT_BLUE_ALLIANCE_WELDED_Y_METERS = 1.344887667;
        public static final double PASSING_SHOOT_RIGHT_RED_ALLIANCE_ANDYMARK_X_METERS  = 14.207236;
        public static final double PASSING_SHOOT_RIGHT_RED_ALLIANCE_ANDYMARK_Y_METERS  = 1.340442667;
        public static final double PASSING_SHOOT_RIGHT_BLUE_ALLIANCE_ANDYMARK_X_METERS = 2.305812;
        public static final double PASSING_SHOOT_RIGHT_BLUE_ALLIANCE_ANDYMARK_Y_METERS = 1.340442667;

        public static final double PASSING_SHOOT_LEFT_RED_ALLIANCE_WELDED_X_METERS   = 14.228191;
        public static final double PASSING_SHOOT_LEFT_RED_ALLIANCE_WELDED_Y_METERS   = 6.724438333;
        public static final double PASSING_SHOOT_LEFT_BLUE_ALLIANCE_WELDED_X_METERS  = 2.312797;
        public static final double PASSING_SHOOT_LEFT_BLUE_ALLIANCE_WELDED_Y_METERS  = 6.724438333;
        public static final double PASSING_SHOOT_LEFT_RED_ALLIANCE_ANDYMARK_X_METERS   = 14.207236;
        public static final double PASSING_SHOOT_LEFT_RED_ALLIANCE_ANDYMARK_Y_METERS   = 6.702213333;
        public static final double PASSING_SHOOT_LEFT_BLUE_ALLIANCE_ANDYMARK_X_METERS  = 2.305812;
        public static final double PASSING_SHOOT_LEFT_BLUE_ALLIANCE_ANDYMARK_Y_METERS  = 6.702213333;

        public static final double FUNNEL_POSITION_RED_ALLIANCE_WELDED_X_METERS   = 11.915394;
        public static final double FUNNEL_POSITION_RED_ALLIANCE_WELDED_Y_METERS   = 4.034663;
        public static final double FUNNEL_POSITION_BLUE_ALLIANCE_WELDED_X_METERS  = 4.625594;
        public static final double FUNNEL_POSITION_BLUE_ALLIANCE_WELDED_Y_METERS  = 4.034663;
        public static final double FUNNEL_POSITION_RED_ALLIANCE_ANDYMARK_X_METERS   = 11.901424;
        public static final double FUNNEL_POSITION_RED_ALLIANCE_ANDYMARK_Y_METERS   = 4.034663;
        public static final double FUNNEL_POSITION_BLUE_ALLIANCE_ANDYMARK_X_METERS  = 4.611624;
        public static final double FUNNEL_POSITION_BLUE_ALLIANCE_ANDYMARK_Y_METERS  = 4.034663;

        // ── Física del proyectil ───────────────────────────────────────
        public static final double FUNNEL_HEIGHT_METERS          = 1.8288;
        public static final double SHOOTER_EXIT_HEIGHT_METERS    = 22 * 0.0254; // 0.5588 m
        public static final double GRAVITY_METERS_PER_SECOND_SQUARED = 9.81;
        public static final double APEX_MAX_HEIGHT_METERS        = 2.4;

        public static final double APEX_RELATIVE_TO_SHOOTER_METERS =
            APEX_MAX_HEIGHT_METERS - SHOOTER_EXIT_HEIGHT_METERS;

        public static final double APEX_TO_FUNNEL_METERS =
            APEX_MAX_HEIGHT_METERS - FUNNEL_HEIGHT_METERS;

        public static final double VERTICAL_LAUNCH_VELOCITY_METERS_PER_SECOND =
            Math.sqrt(2.0 * GRAVITY_METERS_PER_SECOND_SQUARED * APEX_RELATIVE_TO_SHOOTER_METERS);

        public static final double TIME_GOING_UP_SECONDS =
            VERTICAL_LAUNCH_VELOCITY_METERS_PER_SECOND / GRAVITY_METERS_PER_SECOND_SQUARED;

        public static final double TIME_GOING_DOWN_SECONDS =
            Math.sqrt(2.0 * APEX_TO_FUNNEL_METERS / GRAVITY_METERS_PER_SECOND_SQUARED);

        public static final double TOTAL_TIME_SECONDS =
            TIME_GOING_UP_SECONDS + TIME_GOING_DOWN_SECONDS;

        // ── Tiro default ───────────────────────────────────────────────
        /**
         * Distancia horizontal asumida para el tiro default [metros].
         * CALIBRAR: medir la distancia típica de tiro sin visión.
         */
        public static final double DEFAULT_HORIZONTAL_DISTANCE_METERS = 2.0;

        public static final double DEFAULT_HORIZONTAL_VELOCITY_METERS_PER_SECOND =
            DEFAULT_HORIZONTAL_DISTANCE_METERS / TOTAL_TIME_SECONDS;

        /**
         * Ángulo del PROYECTIL para el tiro default [radianes].
         * Este es el ángulo que devuelve ShootingHelper.getHoodAngleRadians().
         * El ángulo físico del hood = 90° - este valor.
         */
        public static final double DEFAULT_HOOD_ANGLE_RADIANS =
            Math.atan2(
                VERTICAL_LAUNCH_VELOCITY_METERS_PER_SECOND,
                DEFAULT_HORIZONTAL_VELOCITY_METERS_PER_SECOND
            );

        public static final double DEFAULT_HOOD_ANGLE_ROT = DEFAULT_HOOD_ANGLE_RADIANS;

        public static final double DEFAULT_EXIT_SPEED_METERS_PER_SECOND =
            Math.hypot(
                DEFAULT_HORIZONTAL_VELOCITY_METERS_PER_SECOND,
                VERTICAL_LAUNCH_VELOCITY_METERS_PER_SECOND
            );

        // ── Restricciones físicas del shooter ─────────────────────────
        /**
         * RPM máximas de las ruedas del shooter.
         * CALIBRAR: ajustar según motor y reducción real.
         */
        public static final double MAX_RPM = 6000.0;

        /**
         * Diámetro de las ruedas del shooter [metros].
         * CALIBRAR: medir el diámetro real (4 pulgadas = 0.1016 m).
         */
        public static final double WHEEL_DIAMETER_METERS = 4.0 * 0.0254;
        public static final double WHEEL_CIRCUMFERENCE   = WHEEL_DIAMETER_METERS * Math.PI;
        public static final double EXIT_SPEED_CONVERSION = WHEEL_CIRCUMFERENCE / 60.0;

        /**
         * Velocidad de salida máxima alcanzable [m/s].
         * 60% de la velocidad libre para margen de control PID.
         * CALIBRAR: aumentar el factor si los tiros caen cortos.
         */
        public static final double MAXIMUM_EXIT_SPEED_METERS_PER_SECOND =
            MAX_RPM * EXIT_SPEED_CONVERSION * 0.60;

        /**
         * Ángulo FÍSICO mínimo del hood [radianes] = 24°.
         * Corresponde al encoder en 0.0 (posición home/stowed).
         * CALIBRAR: medir el ángulo real del hood cuando el encoder está en 0.
         */
        public static final double MINIMUM_HOOD_ANGLE_RADIANS = Math.toRadians(24.0);

        /**
         * Ángulo FÍSICO máximo del hood [radianes] = 92°.
         * Corresponde al encoder en HOOD_ENCODER_ROTATIONS_FOR_FULL_RANGE.
         * CALIBRAR: medir el ángulo real del hood en extensión máxima.
         *
         * RELACIÓN CON EL PROYECTIL:
         *   Si hood está a 24° → proyectil sale a 90° − 24° = 66°
         *   Si hood está a 92° → proyectil sale a 90° − 92° = −2° (imposible)
         *   ShootingHelper valida que el ángulo del proyectil esté dentro de este rango.
         */
        public static final double MAXIMUM_HOOD_ANGLE_RADIANS = Math.toRadians(92.0);

        public static final double MINIMUM_HORIZONTAL_DISTANCE_METERS = 0.1;
    }

    // ════════════════════════════════════════════════════════════
    // VISION CONSTANTS
    // ════════════════════════════════════════════════════════════
    public static final class VisionConstants {
        public static final List<VisionEntries.CameraSpecifications> cameraSpecificationsList =
            List.of(
                new VisionEntries.CameraSpecifications(
                    "RIGHT_FRONT_CAM",
                    new Transform3d(
                        new Translation3d(0.32, -0.32, 0.1778),
                        new Rotation3d(0, 0, 0)
                    ),
                    VisionEnums.PoseEstimateNoiseLevel.FR_CAM,
                    1.0
                ),
                new VisionEntries.CameraSpecifications(
                    "RIGHT_CAM",
                    new Transform3d(
                        new Translation3d(0.32, -0.32, 0.1778),
                        new Rotation3d(0, 0, Math.toRadians(-90))
                    ),
                    VisionEnums.PoseEstimateNoiseLevel.R_CAM,
                    0.8
                )
            );

        public static final double MAXIMUM_AMBIGUITY_FOR_SINGLE_TAG       = 0.40;
        public static final double MAXIMUM_Z_ERROR_METERS                 = 0.40;
        public static final double MAXIMUM_OBSERVATION_AGE_SECONDS        = 0.15;
        public static final double MAXIMUM_DISTANCE_FOR_SINGLE_TAG_METERS = 6.0;
        public static final double MAXIMUM_DISTANCE_FOR_MULTI_TAG_METERS  = 6.0;
        public static final double MAXIMUM_YAW_RATE_RADIANS_PER_SECOND    = 2.0;
        public static final double MAXIMUM_LINEAR_STANDARD_DEVIATION_METERS   = 1.0;
        public static final double MAXIMUM_ANGULAR_STANDARD_DEVIATION_RADIANS = 1.5;
    }

    // ════════════════════════════════════════════════════════════
    // FIELD CONSTANTS
    // ════════════════════════════════════════════════════════════
    public static final class FieldCosntants {
        public static final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        public static final double  FIELD_LENGTH_METERS  = 16.54175;
        public static final double  FIELD_WIDTH_METERS   = 8.0137;
        public static final boolean IS_ANDYMARK_FIELD    = true;

        public static long[] getShootingValidTagIdentifiers() {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
            return alliance == Alliance.Red
                ? new long[] {2, 5, 8, 9, 10, 11}
                : new long[] {18, 21, 24, 25, 26, 27};
        }
    }

    // ════════════════════════════════════════════════════════════
    // HOOD CONSTANTS
    // ════════════════════════════════════════════════════════════
    /**
     * SISTEMA DE ÁNGULOS — RESUMEN:
     *
     *   Hood físico: 24° (mín, encoder = 0) → 92° (máx, encoder = FULL_RANGE)
     *   Proyectil:   complementario del hood físico = 90° − hood_físico
     *     → Si hood = 24° físico → proyectil = 66°
     *     → Si hood = 57° físico → proyectil = 33°
     *     → Si hood = 92° físico → proyectil = −2° (inválido, ShootingHelper lo rechaza)
     *
     *   CONVERSION_RAD_TO_ROT = FULL_RANGE / (92° − 24° en radianes)
     *                         = 126.223  / 1.1868 rad
     *                         ≈ 106.36 rot/rad
     */
    public static final class HoodConstants {

        // ── CAN IDs ───────────────────────────────────────────────────
        public static final int HOOD_ANGLE_TALON_ID            = 15;
        public static final int RIGHT_HOOD_PROPULSION_TALON_ID = 16;
        public static final int LEFT_HOOD_PROPULSION_TALON_ID  = 17;
        public static final int INDEXER_TALON_ID               = 22;

        // ── Conversión ángulo → rotaciones del encoder ────────────────

        /**
         * Rotaciones del encoder de mínimo a máximo (medido empíricamente).
         * CALIBRAR: mover el hood de 24° a 92° físico y leer el encoder en Tuner X.
         * Unidad: rotaciones del encoder (sin dimensión).
         */
        public static final double HOOD_ENCODER_ROTATIONS_FOR_FULL_RANGE = 126.223;

        /**
         * Factor de conversión: radianes → rotaciones del encoder.
         *
         * Cálculo automático:
         *   rango_rad = MAXIMUM (92°) − MINIMUM (24°) = 68° = 1.1868 rad
         *   CONVERSION = 126.223 / 1.1868 ≈ 106.36 rot/rad
         *
         * Este valor se recalcula si cambia HOOD_ENCODER_ROTATIONS_FOR_FULL_RANGE.
         * Unidad: rotaciones / radián.
         */
        public static final double CONVERSION_RATIO_RAD_TO_ROT =
            HOOD_ENCODER_ROTATIONS_FOR_FULL_RANGE /
            (ShootingConstants.MAXIMUM_HOOD_ANGLE_RADIANS -
             ShootingConstants.MINIMUM_HOOD_ANGLE_RADIANS);

        /**
         * Factor de conversión: velocidad de salida [m/s] → RPS de las ruedas.
         * Las ruedas van directo al motor (gear ratio 1:1).
         * Unidad: (rot/s) / (m/s) = 1/m
         */
        public static final double CONVERSION_RATIO_FROM_METERS_TO_RPS =
            1.0 / ShootingConstants.WHEEL_CIRCUMFERENCE;

        // ── Geometría ─────────────────────────────────────────────────

        /** 90° en radianes — para el cálculo del complementario. */
        public static final double COMPLEMENTARY_ANGLE = Math.toRadians(90.0);

        /** Posición home del encoder = 0.0 (hood en 24° físico). */
        public static final double HOOD_HOME_POSITION_ROTATIONS = 0.0;

        /**
         * Offset del ángulo mínimo [rad] = 24°.
         * Se resta para obtener el ángulo relativo al encoder.
         * hood_relativo = hood_físico − OFFSET
         */
        public static final double HOOD_ANGLE_OFFSET_RADIANS =
            ShootingConstants.MINIMUM_HOOD_ANGLE_RADIANS;

        // ── Límites seguros del encoder ───────────────────────────────

        /** Rotación mínima segura [rot]. Margen negativo para errores de zeroing. */
        public static final double HOOD_SAFE_MIN_ROTATIONS = -5.0;

        /** Rotación máxima segura [rot]. 5% de margen sobre el rango completo. */
        public static final double HOOD_SAFE_MAX_ROTATIONS =
            HOOD_ENCODER_ROTATIONS_FOR_FULL_RANGE * 1.05;

        // ── Dirección de las ruedas ───────────────────────────────────
        /**
         * Las ruedas están montadas en lados OPUESTOS de la barra de disparo.
         * Para que ambas empujen la nota en la misma dirección deben girar
         * en sentidos opuestos.
         *
         * Si la nota sale en sentido contrario al esperado:
         *   → Intercambiar RIGHT_WHEEL_INVERTED y LEFT_WHEEL_INVERTED.
         *
         * ⚠️  NUNCA compartir el objeto MotorOutputConfigs entre ruedas.
         *     HoodSubsystem crea un objeto MotorOutputConfigs independiente
         *     para cada rueda usando estas constantes.
         */
        public static final InvertedValue LEFT_WHEEL_INVERTED = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue RIGHT_WHEEL_INVERTED = InvertedValue.Clockwise_Positive;

        // ── PID Hood Angle ────────────────────────────────────────────
        /**
         * Ganancias para MotionMagicExpoTorqueCurrentFOC del hood (Phoenix Pro).
         *
         * kS  [A]        Corriente de fricción estática. Aumentar si el motor no arranca.
         * kG  [A]        Compensación de gravedad (Arm_Cosine). CRÍTICO para sostener el hood.
         *                Aumentar si el hood cae al soltar; reducir si vibra.
         * kV  [A/(rot/s)] Feedforward de velocidad. Calibrar con SysId.
         * kA  [A/(rot/s²)] Feedforward de aceleración.
         * kP  [A/rot]    Proporcional. Aumentar para respuesta más rápida (con riesgo de oscilación).
         * kI  [A/(rot·s)] Integral. Mantener en 0 para empezar.
         * kD  [A/(rot/s)] Derivativo. Aumentar para amortiguamiento.
         */
        public static final Slot0Configs HOOD_ANGLE_SLOT_CONFIGS = new Slot0Configs()
            .withKS(0.1)
            .withKG(0.25)
            .withKV(0.12)
            .withKA(0.01)
            .withKP(50.0)
            .withKI(0.0)
            .withKD(0.5)
            .withGravityType(GravityTypeValue.Arm_Cosine);

        /**
         * Motion Magic para movimientos suaves del hood.
         *
         * CruiseVelocity [RPS]   Velocidad máxima del perfil. Reducir si el hood golpea topes.
         * Acceleration   [RPS/s] Aceleración del perfil. ~2× CruiseVelocity es un buen inicio.
         * Jerk           [RPS/s²] Suavizado (S-curve). ~10× Acceleration.
         */
        public static final MotionMagicConfigs HOOD_MOTION_MAGIC_CONFIGS =
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(300.0)
                .withMotionMagicAcceleration(600.0)
                .withMotionMagicJerk(6000.0);

        /**
         * Motor output del hood.
         * NeutralMode = Brake: CRÍTICO, sin esto el hood cae por gravedad.
         * Inverted: ajustar si el hood se mueve al revés.
         */
        public static final MotorOutputConfigs HOOD_ANGLE_MOTOR_OUTPUT_CONFIGS =
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);

        public static final CurrentLimitsConfigs HOOD_ANGLE_CURRENT_LIMITS =
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(40.0)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(60.0)
                .withStatorCurrentLimitEnable(true);

        public static final SoftwareLimitSwitchConfigs HOOD_SOFTWARE_LIMITS =
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(HOOD_SAFE_MAX_ROTATIONS)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(HOOD_SAFE_MIN_ROTATIONS);

        // ── PID Ruedas ────────────────────────────────────────────────
        /**
         * Ganancias para VelocityTorqueCurrentFOC de las ruedas (Phoenix Pro).
         *
         * Con TorqueCurrentFOC las unidades son AMPERIOS, no Voltios.
         * El controlador convierte internamente Amperios → torque → movimiento.
         *
         * kS  [A]          Corriente mínima para vencer la fricción estática.
         *                  Con ruedas de shooter (inercia alta): 2–5 A.
         *                  CALIBRAR: si las ruedas vibran en reposo, reducir.
         *
         * kV  [A/(rot/s)]  Feedforward de velocidad — el parámetro MÁS IMPORTANTE.
         *                  Representa cuántos Amperios se necesitan para mantener
         *                  1 rot/s en estado estacionario.
         *
         *                  CÁLCULO APROXIMADO para Kraken X60:
         *                    Torque libre ≈ 0 Nm (sin carga a velocidad libre)
         *                    Con carga (volante de inercia): ~0.3–0.5 A/(rot/s)
         *                    A 50 RPS → ~15–25 A de feedforward → rueda llega a velocidad
         *
         *                  CALIBRAR con Tuner X:
         *                    1. Poner kV=0, kP=0, kS=0
         *                    2. Aplicar corriente fija (ej. 20 A) y medir RPS resultante
         *                    3. kV = corriente_aplicada / RPS_medido
         *                    Ejemplo: 20 A → 50 RPS → kV = 20/50 = 0.4
         *
         * kP  [A/(rot/s)]  Proporcional. Corrige el error residual que deja kV.
         *                  Típico: 0.5 – 2.0 A/(rot/s).
         *                  Aumentar si hay error de estado estacionario persistente.
         *                  Reducir si las ruedas oscilan alrededor de la velocidad objetivo.
         *
         * kA  [A/(rot/s²)] Feedforward de aceleración. Ayuda al spin-up.
         *                  Típico: 0.01 – 0.05.
         */
        public static final Slot0Configs RIGHT_HOOD_PROPULSION_SLOT_CONFIGS = new Slot0Configs()
            .withKS(3.0)    // ← era 0.1  — suficiente para vencer fricción estática
            .withKG(0.0)    // Sin compensación de gravedad (ruedas horizontales)
            .withKV(0.65)    // ← era 0.12 — feedforward principal, ~20A a 50 RPS
            .withKA(0.02)   // ← era 0.01 — ayuda en el spin-up inicial
            .withKP(1.5)    // ← era 0.5  — corrige el error residual más agresivamente
            .withKI(0.0)
            .withKD(0.0);

        // La izquierda usa las mismas ganancias (la inversión está en el MotorOutput de cada una)
        public static final Slot0Configs LEFT_HOOD_PROPULSION_SLOT_CONFIGS =
            RIGHT_HOOD_PROPULSION_SLOT_CONFIGS;

        /**
         * Límites de corriente para las ruedas.
         *
         * SupplyCurrentLimit [A]: corriente máxima desde la batería.
         *   60 A — permite picos de spin-up sin fundir el cableado (16 AWG).
         *   CALIBRAR: si salta el breaker de 40 A, reducir a 35 A.
         *
         * StatorCurrentLimit [A]: corriente máxima en el bobinado del motor.
         *   100 A — Kraken X60 aguanta 120 A estator continuos; 100 A da margen.
         *   Este límite es el que realmente define el torque máximo de spin-up.
         *   CALIBRAR: reducir a 80 A si el motor se calienta demasiado.
         */
        public static final CurrentLimitsConfigs WHEEL_CURRENT_LIMITS =
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(60.0)   // ← era 40 A
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(120.0)  // ← era 80 A — más torque en spin-up
                .withStatorCurrentLimitEnable(true);

        // ── Indexer ───────────────────────────────────────────────────
        public static final Slot0Configs INDEXER_SLOT_CONFIGS = new Slot0Configs();

        /**
         * Motor output del indexer.
         * NeutralMode = Brake: frena activamente al apagar para no sobre-alimentar.
         * Inverted: ajustar según la dirección real del mecanismo de alimentación.
         */
        public static final MotorOutputConfigs INDEXER_MOTOR_OUTPUT_CONFIGS =
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);

        public static final CurrentLimitsConfigs INDEXER_CURRENT_LIMITS =
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30.0)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(40.0)
                .withStatorCurrentLimitEnable(true);

        /**
         * Duty cycle del indexer al disparar [0.0 – 1.0].
         * 0.8 = 80% de potencia (9.6 V efectivos).
         * CALIBRAR: reducir si la nota se atasca, aumentar si sale lento.
         */
        public static final double INDEXER_SPEED = 0.8;

        // ── Tolerancias ───────────────────────────────────────────────

        /**
         * Tolerancia de posición del hood [rotaciones].
         * 4.0 rot / 106.36 rot·rad⁻¹ ≈ 0.038 rad ≈ 2.2°
         * CALIBRAR: reducir para mayor precisión (readyToShoot tarda más).
         *           Aumentar si readyToShoot nunca se activa.
         */
        public static final double HOOD_ANGLE_TOLERANCE_ROT = 4.0;

        /**
         * Tolerancia de velocidad de las ruedas [RPS].
         * 5.0 RPS ≈ 10% de error a 50 RPS.
         * CALIBRAR: reducir para disparar más preciso.
         *           Aumentar si readyToShoot nunca se activa.
         */
        public static final double HOOD_SHOOTING_VELOCITY_TOLERANCE_RPS = 5.0;

        // ── Post-shot coasting ────────────────────────────────────────

        /**
         * Tiempo mínimo en POST_SHOT_COASTING antes de stow [s].
         * Asegura que la nota salga antes de mover el hood.
         * CALIBRAR: aumentar si la nota choca con el hood al stow.
         */
        public static final double POST_SHOT_COAST_MINIMUM_TIME_SECONDS = 0.3;

        /**
         * Timeout máximo en POST_SHOT_COASTING [s].
         * Si las ruedas no frenan, fuerza el stow de todas formas.
         */
        public static final double POST_SHOT_COAST_MAXIMUM_TIME_SECONDS = 2.0;

        /**
         * Velocidad de ruedas considerada "detenida" [RPS].
         * El sistema transiciona a STOWING cuando ambas ruedas están bajo este umbral.
         */
        public static final double POST_SHOT_WHEEL_STOP_THRESHOLD_ROTATIONS_PER_SECOND = 1.0;
    }

    // ════════════════════════════════════════════════════════════
    // INTAKE CONSTANTS
    // ════════════════════════════════════════════════════════════
    public static final class IntakeConstants {
        public static final int    INTAKE_MOTOR_ID               = 23;
        public static final int    PIVOT_INTAKE__RIGHT_MOTOR_ID  = 19;
        public static final int    PIVOT_INTAKE_LEFT_MOTOR_ID    = 18;

        public static final double INTAKE_ACTIVATION_VOLTAGE_VOLTS = 12.0 * 0.6;

        public static final double PIVOT_RETRACT_POSITION_ROTATIONS = 0.0;
        public static final double PIVOT_DEPLOY_POSITION_ROTATIONS  = 1.0;

        public static final double PIVOT_SOFT_LIMIT_MINIMUM_ROTATIONS = -0.10;
        public static final double PIVOT_SOFT_LIMIT_MAXIMUM_ROTATIONS =  1.10;

        public static final double PIVOT_POSITION_TOLERANCE_ROTATIONS  = 0.02;
        public static final double PIVOT_POSITION_HYSTERESIS_ROTATIONS = 0.02;

        public static final double PIVOT_POSITION_PROPORTIONAL_GAIN = 0.5;
        public static final double PIVOT_POSITION_INTEGRAL_GAIN     = 0.0;
        public static final double PIVOT_POSITION_DERIVATIVE_GAIN   = 0.0;

        public static final double PIVOT_CONTROL_MAXIMUM_ABSOLUTE_VOLTAGE_VOLTS = 12.0;

        public static final double PIVOT_RAW_ENCODER_RETRACT_ROTATIONS = 0.0;
        public static final double PIVOT_RAW_ENCODER_DEPLOY_ROTATIONS  = 25.0;
        public static final boolean PIVOT_ZERO_ENCODER_ON_BOOT_TO_RETRACT = true;

        public static final double PIVOT_SIM_RETRACT_ANGLE_RADIANS = 0.0;
        public static final double PIVOT_SIM_DEPLOY_ANGLE_RADIANS  = 1.57;

        public static final double  MAPLESIM_INTAKE_WIDTH_METERS     = 0.70;
        public static final double  MAPLESIM_INTAKE_EXTENSION_METERS = 0.20;
        public static final int     MAPLESIM_INTAKE_CAPACITY         = 1;

        public static final double  PIVOT_SIM_GEAR_REDUCTION      = 60.0;
        public static final double  PIVOT_SIM_MOMENT_OF_INERTIA   = 0.002;
        public static final double  PIVOT_SIM_ARM_LENGTH_METERS   = 0.25;
        public static final double  PIVOT_SIM_MIN_ANGLE_RADIANS   = -0.2;
        public static final double  PIVOT_SIM_MAX_ANGLE_RADIANS   = 1.8;
        public static final double  PIVOT_SIM_START_ANGLE_RADIANS = 0.0;

        public static final int     ROLLER_MOTOR_ID            = 23;
        public static final int     PIVOT_INTAKE_RIGHT_MOTOR   = 19;
        public static final int     PIVOT_INTAKE_LEFT_MOTOR    = 18;
        public static final boolean PIVOT_RIGHT_MOTOR_INVERTED = false;
    }

    // ════════════════════════════════════════════════════════════
    // CLIMBER CONSTANTS
    // ════════════════════════════════════════════════════════════
    public static final class ClimberConstants {
        public static final int    LEFT_CLIMBER_MOTOR_ID  = 20;
        public static final int    RIGHT_CLIMBER_MOTOR_ID = 21;

        public static final double CLIMBER_MAX_DUTY_CYCLE = 0.8;

        public static final double CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS = 0.0;
        public static final double CLIMBER_EXTENDED_POSITION_ROTATIONS         = 0.0;

        public static final double CLIMBER_POSITION_TOLERANCE_ROTATIONS = 0.05;

        public static final double CLIMBER_MANUAL_VOLTAGE_VOLTS =
            Constants.BATTERY_VOLTAGE * CLIMBER_MAX_DUTY_CYCLE;

        public static final double CLIMBER_POSITION_PROPORTIONAL_GAIN = 6.0;
        public static final double CLIMBER_POSITION_INTEGRAL_GAIN     = 0.0;
        public static final double CLIMBER_POSITION_DERIVATIVE_GAIN   = 0.2;

        public static final double CLIMBER_PID_INTEGRATOR_MINIMUM_VOLTS = -Constants.BATTERY_VOLTAGE;
        public static final double CLIMBER_PID_INTEGRATOR_MAXIMUM_VOLTS =  Constants.BATTERY_VOLTAGE;

        public static final double CLIMBER_OUTPUT_MINIMUM_VOLTS = -Constants.BATTERY_VOLTAGE;
        public static final double CLIMBER_OUTPUT_MAXIMUM_VOLTS =  Constants.BATTERY_VOLTAGE;

        public static final double CLIMBER_AVERAGE_MULTIPLIER = 0.5;

        private ClimberConstants() {}
    }
}