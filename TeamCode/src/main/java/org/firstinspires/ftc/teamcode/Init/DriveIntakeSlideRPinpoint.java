package org.firstinspires.ftc.teamcode.Init;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(name = "LimelightTurretR_Pinpoint", group = "TeleOp")
public class DriveIntakeSlideRPinpoint extends LinearOpMode {

    // Heading (grados) con el que el robot arranca el TeleOp, respecto al frente del campo.
    // Si siempre arranca mirando al frente del campo, déjalo en 0.
    private static final double START_HEADING_DEG = 0.0;

    // true: resetea el yaw del IMU en init (el robot debe arrancar mirando al frente del campo).
    // false: conserva el yaw que dejó el autónomo.
    private static final boolean RESET_YAW_ON_INIT = true;

    private static final double SHOOT_SPEED_SCALE = 0.45; // velocidad del chasis con right_bumper (gamepad 1)

    // Visión e IMU
    Limelight3A limelight;
    IMU imu;

    // SubSistemas
    private DcMotorEx shooter;
    private DcMotorEx turret;
    private VoltageSensor batteryVoltageSensor;

    // Tolerancia de RPM para disparo
    private static final double RPM_TOLERANCE = 25.0;

    // Configuración Torreta
    private static final double MOTOR_TICKS_PER_REV = 537.7;
    private static final double GEAR_RATIO = 1.0; // pendiente: medir dientes de los engranajes (dientes torreta / dientes piñón)
    private static final double TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * GEAR_RATIO) / 360.0;

    private static final double MAX_TURRET_ANGLE_DEG = 100.0;
    private static final double MIN_TURRET_ANGLE_DEG = -100.0;
    private static final double MAX_TURRET_TICKS = MAX_TURRET_ANGLE_DEG * TICKS_PER_DEGREE;
    private static final double MIN_TURRET_TICKS = MIN_TURRET_ANGLE_DEG * TICKS_PER_DEGREE;

    private static final double MAX_TURRET_POWER = 0.75;
    private static final double MIN_TURRET_POWER = 0.08;
    private static final double MANUAL_TURRET_POWER_SCALE = 0.50;
    private static final double MANUAL_OVERRIDE_DEADZONE = 0.10;

    // Límite suave del auto-aim: si el objetivo queda más allá, la torreta se queda en el límite
    // (sin desenredar) y la telemetría pide girar el chasis. Evita el ciclo apuntar -> desenredar -> apuntar.
    private static final double AUTO_AIM_LIMIT_DEG = 88.0;

    // Desenredado de cables (seguridad): solo se activa si la torreta pasa del límite suave
    // (inercia o tras el control manual). Vuelve hacia el centro hasta quedar cerca de 0°.
    private static final double UNWIND_THRESHOLD_DEG = 95.0;
    private static final double UNWIND_EXIT_DEG = 10.0;
    private static final double UNWIND_MAX_POWER = 0.60;
    private static final double UNWIND_KP = 0.02;
    private static final double SWEEP_POWER = 0.25;

    // Centrado con dpad_down (gamepad 2): control proporcional para no oscilar
    private static final double CENTER_KP = 0.02;
    private static final double CENTER_MAX_POWER = 0.40;
    private static final double CENTER_TOLERANCE_DEG = 1.0;

    private static final double DEADZONE_TX = 0.50;
    private static final double ALIGN_KP = 0.035;
    private static final double GYRO_FF_KP = 0.003;
    private static final double GYRO_FF_SIGN = 1.0;

    // Resultados de la Limelight más viejos que esto (ms) se ignoran
    private static final long MAX_LL_STALENESS_MS = 100;

    // Configuración Flywheel (Recuperación rápida)
    private final double ENCODER_CPR = 28.0;
    private final double ULTRA_RATIO = 2.89;

    private double kS = 0.08;
    private double kV = 0.000488;
    private double kP = 0.0050;
    private double kI = 0.0010;
    private double kD = 0.00002;

    // Recuperación agresiva del flywheel (para ráfagas): si las RPM caen más de ENTER (fracción
    // del objetivo), da potencia máxima hasta volver a quedar a menos de EXIT; luego retoma PID + feedforward.
    private static final double RECOVERY_ENTER_FRACTION = 0.03;
    private static final double RECOVERY_EXIT_FRACTION = 0.01;
    private static final double RECOVERY_POWER = 1.0;
    private boolean recovering = false;

    // Boost de objetivo en ráfagas: al detectar la caída de RPM de la primera bola, sube el objetivo
    // BURST_BOOST_FRACTION durante la ráfaga, para que las bolas siguientes salgan desde RPM más altas.
    // Se arma solo cuando el shooter ya alcanzó su objetivo (así no se activa durante el arranque).
    // Ajustar en cancha: si las bolas 2 y 3 se pasan de largo, bajar BURST_BOOST_FRACTION.
    private static final double BURST_ARM_FRACTION = 0.02;    // RPM a menos de esto del objetivo = listo
    private static final double SHOT_DETECT_FRACTION = 0.03;  // caída que se considera un disparo
    private static final double BURST_BOOST_FRACTION = 0.05;  // aumento del objetivo durante la ráfaga (antes 0.08)
    private static final double BURST_HOLD_SECONDS = 0.8;     // sin nuevas caídas por este tiempo = fin de la ráfaga
    private boolean burstArmed = false;
    private boolean burstActive = false;
    private double lastTargetRPM = 0.0;
    private ElapsedTime burstTimer = new ElapsedTime();


    private double integralSum = 0.0;
    private double maxIntegralPowerContribution = 0.25;
    private double lastRPM = 0.0; // derivada sobre la medición (sin "golpe" al cambiar el objetivo)
    private ElapsedTime pidTimer = new ElapsedTime();
    private ElapsedTime telemetryTimer = new ElapsedTime();

    private double targetRPM = 0.0;

    // Filtro anti-parpadeo y Escaneo Automático
    private ElapsedTime tagLostTimer = new ElapsedTime();
    private static final double TAG_MEMORY_TIMEOUT = 0.3;

    // Rotación de pipelines de la Limelight si lleva tiempo sin detectar el AprilTag.
    // Configura los pipelines 0, 1 y 2 en la interfaz web de la Limelight.
    private static final int PIPELINE_COUNT = 3;
    private static final double PIPELINE_SWITCH_SECONDS = 1.0; // tiempo en cada pipeline antes de pasar al siguiente
    private ElapsedTime pipelineTimer = new ElapsedTime();
    private int currentPipeline = 0;
    private boolean sweepDirectionRight = true;

    private static final double INTAKE_POWER_SCALE = 0.8;

    @Override
    public void runOpMode() {

        Chassis chassis = new Chassis(hardwareMap, telemetry);
        Robot robot = new Robot(hardwareMap, telemetry);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotor.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientation));
        if (RESET_YAW_ON_INIT) {
            imu.resetYaw();
        }

        double startHeading = START_HEADING_DEG;

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Robot Inicializado (Visión y Telemetría optimizadas).");
        telemetry.update();

        waitForStart();

        // El encoder se reseteó en init: si alguien movió la torreta a mano después, esto la regresa al frente.
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.30);

        ElapsedTime initTimer = new ElapsedTime();
        initTimer.reset();

        while (opModeIsActive() && turret.isBusy() && initTimer.seconds() < 1.5) {
            telemetry.addLine("Centrando torreta al frente (0°)...");
            telemetry.update();
        }

        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean lastDpadRight = false;
        boolean lastDpadLeft = false;
        boolean unwinding = false;

        pidTimer.reset();
        tagLostTimer.reset();
        pipelineTimer.reset();
        telemetryTimer.reset();
        lastRPM = getRPM();

        while (opModeIsActive()) {

            // =========================================================
            // 1. LECTURA DE IMU Y GIROSCOPIO (GYRO FEEDFORWARD)
            // =========================================================
            YawPitchRollAngles orientationData = imu.getRobotYawPitchRollAngles();
            double yaw = orientationData.getYaw(AngleUnit.DEGREES);
            limelight.updateRobotOrientation(yaw);

            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            double chassisYawRate = angularVelocity.zRotationRate;
            double gyroFeedForward = -chassisYawRate * GYRO_FF_KP * GYRO_FF_SIGN;

            // Auto-aim siempre activo; mover el stick derecho del gamepad 2 toma control manual mientras se mueve
            boolean manualOverride = Math.abs(gamepad2.right_stick_x) > MANUAL_OVERRIDE_DEADZONE;

            double turretPositionTicks = turret.getCurrentPosition();
            double turretAngleDeg = turretPositionTicks / TICKS_PER_DEGREE;
            double desiredTurretPower = 0;
            boolean targetOutOfRange = false;

            // Desenredado de cables (con histéresis)
            if (Math.abs(turretAngleDeg) >= UNWIND_THRESHOLD_DEG) {
                unwinding = true;
            } else if (Math.abs(turretAngleDeg) <= UNWIND_EXIT_DEG) {
                unwinding = false;
            }

            if (gamepad2.dpad_down) {
                if (Math.abs(turretAngleDeg) > CENTER_TOLERANCE_DEG) {
                    desiredTurretPower = clamp(-turretAngleDeg * CENTER_KP, CENTER_MAX_POWER);
                }
            }
            else if (unwinding && !manualOverride) {
                // Ignora el auto-aim y regresa la torreta hacia el centro (0°)
                desiredTurretPower = clamp(-turretAngleDeg * UNWIND_KP, UNWIND_MAX_POWER);
            }
            else if (!manualOverride) {
                LLResult result = limelight.getLatestResult();

                boolean tagVisible = result != null && result.isValid() && result.getTa() > 0.0
                        && result.getStaleness() <= MAX_LL_STALENESS_MS;

                if (tagVisible) {
                    tagLostTimer.reset();
                    pipelineTimer.reset(); // con tag visible se queda en el pipeline actual
                    double tx = result.getTx();

                    if (Math.abs(tx) > DEADZONE_TX) {
                        desiredTurretPower = (tx * ALIGN_KP) + gyroFeedForward;
                        desiredTurretPower = clamp(desiredTurretPower, MAX_TURRET_POWER);
                        if (Math.abs(desiredTurretPower) < MIN_TURRET_POWER) {
                            desiredTurretPower = (desiredTurretPower > 0) ? MIN_TURRET_POWER : -MIN_TURRET_POWER;
                        }
                    } else {
                        desiredTurretPower = gyroFeedForward;
                    }

                    // Límite suave: no seguir el objetivo más allá de AUTO_AIM_LIMIT_DEG
                    if ((turretAngleDeg >= AUTO_AIM_LIMIT_DEG && desiredTurretPower > 0)
                            || (turretAngleDeg <= -AUTO_AIM_LIMIT_DEG && desiredTurretPower < 0)) {
                        desiredTurretPower = 0.0;
                        targetOutOfRange = true;
                    }
                } else {
                    // Sin tag más allá de la memoria corta: rota al siguiente pipeline
                    if (tagLostTimer.seconds() >= TAG_MEMORY_TIMEOUT
                            && pipelineTimer.seconds() > PIPELINE_SWITCH_SECONDS) {
                        currentPipeline = (currentPipeline + 1) % PIPELINE_COUNT;
                        limelight.pipelineSwitch(currentPipeline);
                        pipelineTimer.reset();
                    }
                    if (tagLostTimer.seconds() < TAG_MEMORY_TIMEOUT) {
                        desiredTurretPower = gyroFeedForward;
                    } else {
                        if (turretAngleDeg >= 45.0) {
                            sweepDirectionRight = false;
                        } else if (turretAngleDeg <= -45.0) {
                            sweepDirectionRight = true;
                        }
                        desiredTurretPower = (sweepDirectionRight ? SWEEP_POWER : -SWEEP_POWER) + gyroFeedForward;
                    }
                }
            }
            else {
                desiredTurretPower = gamepad2.right_stick_x * MANUAL_TURRET_POWER_SCALE;
            }

            desiredTurretPower = applyTurretLimits(turretPositionTicks, desiredTurretPower);
            turret.setPower(desiredTurretPower);

            // =========================================================
            // 2. GAMEPAD 2: CONTROL DE RPM (DE 25 EN 25)
            // =========================================================
            boolean currentDpadRight = gamepad2.dpad_right;
            if (currentDpadRight && !lastDpadRight) {
                targetRPM += 25.0;
            }
            lastDpadRight = currentDpadRight;

            boolean currentDpadLeft = gamepad2.dpad_left;
            if (currentDpadLeft && !lastDpadLeft) {
                targetRPM -= 25.0;
                if (targetRPM < 0) targetRPM = 0.0;
            }
            lastDpadLeft = currentDpadLeft;

            if (gamepad2.x) {
                targetRPM = 0.0;
            }

            // =========================================================
            // 3. PID DEL SHOOTER (con recuperación agresiva para ráfagas)
            // =========================================================
            double currentRPM = getRPM();

            // Boost del objetivo durante la ráfaga (el cambio manual de objetivo reinicia la detección)
            if (targetRPM != lastTargetRPM) {
                lastTargetRPM = targetRPM;
                burstArmed = false;
                burstActive = false;
            }
            if (targetRPM <= 0) {
                burstArmed = false;
                burstActive = false;
            } else {
                if (!burstArmed && currentRPM >= targetRPM * (1.0 - BURST_ARM_FRACTION)) {
                    burstArmed = true;
                }
                if (burstArmed && currentRPM < targetRPM * (1.0 - SHOT_DETECT_FRACTION)) {
                    burstActive = true;
                    burstTimer.reset();
                }
                if (burstActive && burstTimer.seconds() > BURST_HOLD_SECONDS) {
                    burstActive = false;
                }
            }
            double effTarget = burstActive ? targetRPM * (1.0 + BURST_BOOST_FRACTION) : targetRPM;

            double error = effTarget - currentRPM;
            double dt = pidTimer.seconds();
            pidTimer.reset();

            // Derivada sobre la medición: no da picos cuando effTarget salta por el boost
            double derivative = dt > 0 ? -(currentRPM - lastRPM) / dt : 0.0;
            lastRPM = currentRPM;

            if (targetRPM <= 0) {
                shooter.setPower(0.0);
                integralSum = 0.0;
                recovering = false;
            } else {
                // Histéresis de la recuperación agresiva
                if (!recovering && currentRPM < effTarget * (1.0 - RECOVERY_ENTER_FRACTION)) {
                    recovering = true;
                } else if (recovering && currentRPM >= effTarget * (1.0 - RECOVERY_EXIT_FRACTION)) {
                    recovering = false;
                }

                if (!recovering && Math.abs(error) < 150.0) {
                    integralSum += error * dt;
                    double maxIntegralSum = maxIntegralPowerContribution / kI;
                    integralSum = Math.min(maxIntegralSum, Math.max(-maxIntegralSum, integralSum));
                }

                double feedForward = (kV * effTarget) + kS;
                double feedbackPID = (kP * error) + (kI * integralSum) + (kD * derivative);

                double rawVoltage = batteryVoltageSensor.getVoltage();
                double voltageMultiplier = 12.0 / Math.max(rawVoltage, 8.0);

                double calculatedPower = (feedForward + feedbackPID) * voltageMultiplier;
                calculatedPower = Math.min(1.0, Math.max(0.0, calculatedPower));

                // En recuperación: potencia máxima (el integrador queda congelado para evitar windup)
                shooter.setPower(recovering ? RECOVERY_POWER : calculatedPower);
            }

            // =========================================================
            // 4. GAMEPAD 1: CHASIS (más lento con right_bumper) E INTAKE (AL 80%)
            // =========================================================
            double driveScale = gamepad1.right_bumper ? SHOOT_SPEED_SCALE : 1.0;

            // Field-centric siempre activo. Con el bumper izquierdo (gamepad 1) y el robot mirando
            // al frente del campo, se recalibra el yaw del IMU.
            if (gamepad1.left_bumper) {
                imu.resetYaw();
                startHeading = 0.0;
            }
            double hRad = Math.toRadians(yaw + startHeading);
            double fieldFwd = -gamepad1.left_stick_y * driveScale;  // adelante del campo
            double fieldRight = gamepad1.left_stick_x * driveScale; // derecha del campo
            double robotFwd = fieldFwd * Math.cos(hRad) - fieldRight * Math.sin(hRad);
            double robotRight = fieldFwd * Math.sin(hRad) + fieldRight * Math.cos(hRad);
            chassis.move2(-robotFwd, robotRight, gamepad1.right_stick_x * driveScale);

            if (gamepad1.left_trigger > 0.2) {
                robot.moveIntake(1.0 * INTAKE_POWER_SCALE);   // agarrar
            } else if (gamepad1.right_trigger > 0.2) {
                robot.moveIntake(-1.0 * INTAKE_POWER_SCALE);  // sacar
            } else {
                robot.moveIntake(0);
            }

            // =========================================================
            // 5. TELEMETRÍA CONTROLADA (Cada 100ms)
            // =========================================================
            if (telemetryTimer.milliseconds() > 100) {
                // Listo si está en o por encima del objetivo (durante el boost va más arriba a propósito)
                boolean rpmEnRango = targetRPM > 0 && currentRPM >= targetRPM - RPM_TOLERANCE;
                String modo;
                if (gamepad2.dpad_down) modo = "CENTRANDO";
                else if (manualOverride) modo = "MANUAL";
                else if (unwinding) modo = "DESENREDANDO";
                else if (targetOutOfRange) modo = "FUERA DE RANGO - GIRA EL CHASIS";
                else modo = tagLostTimer.seconds() > TAG_MEMORY_TIMEOUT ? "BUSCANDO (Escaneo)" : "AUTO (Locked)";
                telemetry.addData("Modo Torreta", modo);
                telemetry.addData("Pipeline Limelight", currentPipeline);
                telemetry.addData("Ángulo Torreta", "%.1f° (Max ±100°)", turretAngleDeg);
                telemetry.addData("Shooter Target / Actual", "%.0f / %.0f RPM", targetRPM, currentRPM);
                telemetry.addData("Objetivo efectivo", "%.0f RPM%s", effTarget, burstActive ? " (BOOST ráfaga)" : "");
                telemetry.addData("Shooter listo", recovering ? "RECUPERANDO..." : (rpmEnRango ? "SÍ (RPM OK)" : "ESPERANDO..."));
                telemetry.update();
                telemetryTimer.reset();
            }
        }

        chassis.stopChassis();
        robot.moveIntake(0);
        shooter.setPower(0.0);
        turret.setPower(0.0);
        limelight.stop();
    }

    private static double clamp(double value, double maxAbs) {
        return Math.max(-maxAbs, Math.min(maxAbs, value));
    }

    private double applyTurretLimits(double currentTicks, double requestedPower) {
        if (currentTicks >= MAX_TURRET_TICKS && requestedPower > 0) {
            return 0.0;
        }
        if (currentTicks <= MIN_TURRET_TICKS && requestedPower < 0) {
            return 0.0;
        }
        return requestedPower;
    }

    private double getRPM() {
        double ticksPerSec = shooter.getVelocity();
        return (ticksPerSec / ENCODER_CPR) * 60.0 / ULTRA_RATIO;
    }
}
