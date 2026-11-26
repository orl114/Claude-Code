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
    // mI1/mI2 (motores del intake) ahora viven en Robot, como robot.mI1 / robot.mI2.
    private VoltageSensor batteryVoltageSensor;

    // Tolerancia de RPM para disparo
    private static final double RPM_TOLERANCE = 25.0;

    // Configuración Torreta
    private static final double MOTOR_TICKS_PER_REV = 537.7;
    // Medido contando dientes: engrane de la torreta 120T, piñón del motor 24T.
    private static final double GEAR_RATIO = 120.0 / 24.0; // = 5.0
    private static final double TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * GEAR_RATIO) / 360.0;

    // Tope mecánico real medido a mano: ~190° de cada lado.
    private static final double MAX_TURRET_ANGLE_DEG = 165.0;
    private static final double MIN_TURRET_ANGLE_DEG = -165.0;
    private static final double MAX_TURRET_TICKS = MAX_TURRET_ANGLE_DEG * TICKS_PER_DEGREE;
    private static final double MIN_TURRET_TICKS = MIN_TURRET_ANGLE_DEG * TICKS_PER_DEGREE;

    private static final double MAX_TURRET_POWER = 0.75;
    // Potencia mínima para vencer la fricción (engranes cónicos + reducción 5:1). Ajustable en vivo:
    // si la torreta pega un jalón seco al empezar a corregir, bájala.
    private double minTurretPower = 0.045;
    private static final double MANUAL_TURRET_POWER_SCALE = 0.50;
    private static final double MANUAL_OVERRIDE_DEADZONE = 0.10;

    // Límite suave del auto-aim: si el objetivo queda más allá, la torreta se queda en el límite
    // (sin desenredar) y la telemetría pide girar el chasis. Evita el ciclo apuntar -> desenredar -> apuntar.
    private static final double AUTO_AIM_LIMIT_DEG = 150.0;

    // Desenredado de cables (seguridad): solo se activa si la torreta pasa del límite suave
    // (inercia o tras el control manual). Vuelve hacia el centro hasta quedar cerca de 0°.
    private static final double UNWIND_THRESHOLD_DEG = 175.0;
    private static final double UNWIND_EXIT_DEG = 10.0;
    private static final double UNWIND_MAX_POWER = 0.60;
    private static final double UNWIND_KP = 0.02;
    private static final double SWEEP_POWER = 0.25;

    // Centrado con dpad_down (gamepad 2): control proporcional para no oscilar
    private static final double CENTER_KP = 0.02;
    private static final double CENTER_MAX_POWER = 0.40;
    private static final double CENTER_TOLERANCE_DEG = 1.0;

    // Error de apuntado que ya se considera "centrado". Si queda muy chico, la torreta no deja
    // de corregir de un lado a otro buscando una precisión que la mecánica no da.
    private static final double DEADZONE_TX = 1.50;
    private static final double GYRO_FF_KP = 0.003;

    // ===== Apuntado en cascada (visión + encoder de la torreta) =====
    // La cámara manda frames lento y con retraso; el encoder se lee en cada vuelta del loop.
    // Por eso la visión NO maneja el motor directamente: cuando llega un frame nuevo solo fija
    // un ÁNGULO OBJETIVO, y un PD sobre el encoder persigue ese ángulo en cada vuelta.
    // Así la torreta reacciona al ritmo del loop y se frena con la velocidad real del motor,
    // en vez de ir dando tumbos cada vez que llega un frame.
    // La relación aimKp/aimKd marca a qué velocidad intenta acercarse al objetivo: con estos
    // valores pide unos 2.5 °/s por cada grado de error, una velocidad desde la que sí alcanza
    // a frenar. Antes pedía 27 °/s por grado y por eso llegaba pegando y se pasaba de largo.
    private double aimKp = 0.035;  // potencia por grado de error de ángulo
    private double aimKd = 0.008;  // potencia por cada grado/segundo de diferencia de velocidad

    private double aimTargetAngleDeg = 0.0;
    private boolean aimTargetValid = false;
    private double lastStalenessForFrame = Double.MAX_VALUE;
    private double turretRateDegPerSec = 0.0;

    // Qué tan rápido se mueve el objetivo (el ángulo al que hay que apuntar cambia mientras el
    // robot avanza, aunque el chasis no gire). Sin esto, un control por posición siempre va
    // atrás del blanco: corrige, el blanco ya se movió, vuelve a corregir. Con esto la torreta
    // acompaña el movimiento y el error que le queda por cerrar es mucho más chico.
    private double aimTargetRateDegPerSec = 0.0;
    private static final double AIM_RATE_FILTER = 0.35; // suaviza el ruido entre frames
    private final ElapsedTime aimFrameTimer = new ElapsedTime();

    // Abajo de esta velocidad se considera que la torreta está detenida, y ahí sí se aplica la
    // potencia mínima para vencer la fricción. Yendo en movimiento no se aplica, porque taparía
    // el frenado del término derivativo.
    private static final double STATIC_RATE_DEG_PER_SEC = 5.0;

    // Historial de ángulos de la torreta, para compensar el retraso de la cámara: el tx que
    // llega ahora se midió hace "staleness" milisegundos, cuando la torreta estaba en otro
    // ángulo. Sumarlo al ángulo actual deja el objetivo corrido y hace que se pase.
    private static final int ANGLE_HISTORY_SIZE = 128;
    private final double[] angleHistoryDeg = new double[ANGLE_HISTORY_SIZE];
    private final double[] angleHistoryMs = new double[ANGLE_HISTORY_SIZE];
    private int angleHistoryCount = 0;
    private int angleHistoryIndex = 0;
    private final ElapsedTime aimClock = new ElapsedTime();
    private static final double GYRO_FF_SIGN = 1.0;

    // Resultados de la Limelight más viejos que esto (ms) se ignoran
    private static final long MAX_LL_STALENESS_MS = 100;

    // Diagnóstico de la Limelight (ver por qué tagVisible puede dar false), mostrado en telemetría.
    private boolean lastLLResultIsNull = true;
    private boolean lastLLValid = false;
    private double lastLLTa = 0.0;
    private double lastLLStaleness = -1;
    private int lastLLFiducialCount = 0;

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

    private double lastShooterPower = 0.0; // potencia aplicada al shooter (para telemetría)

    // Boost de objetivo en ráfagas: al detectar la caída de RPM de la primera bola, sube el objetivo
    // burstBoostFraction durante la ráfaga, para que las bolas siguientes salgan desde RPM más altas.
    // Se arma solo cuando el shooter ya alcanzó su objetivo (así no se activa durante el arranque).
    // Ajustable en vivo con gamepad 2 (ver sección de ajuste de PID/boost más abajo).
    private static final double BURST_ARM_FRACTION = 0.02;    // RPM a menos de esto del objetivo = listo
    private static final double SHOT_DETECT_FRACTION = 0.03;  // caída que se considera un disparo
    private double burstBoostFraction = 0.08;                 // aumento del objetivo durante la ráfaga
    private static final double BURST_HOLD_SECONDS = 0.8;     // sin nuevas caídas por este tiempo = fin de la ráfaga
    private boolean burstArmed = false;
    private boolean burstActive = false;
    private double lastTargetRPM = 0.0;

    // --- Ajuste en vivo de kP/kI/kV/boost de ráfaga (gamepad 2) ---
    // LB/RB eligen el parámetro; Y sube, A baja (x1.1 / ÷1.1 cada pulsación).
    private static final String[] TUNABLE_NAMES = {
            "kP", "kI", "kV", "Boost ráfaga", "Aim kP", "Aim kD", "Aim pot. mín"};
    private int tunableIndex = 0;
    private boolean lastTuneLeft = false;
    private boolean lastTuneRight = false;
    private boolean lastTuneUp = false;
    private boolean lastTuneDown = false;
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
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotor.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
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
            turretRateDegPerSec = turret.getVelocity() / TICKS_PER_DEGREE;
            double desiredTurretPower = 0;
            boolean targetOutOfRange = false;

            // Guarda el ángulo de este instante, para poder consultar después en qué ángulo
            // estaba la torreta cuando la cámara tomó la foto que llegue más adelante.
            angleHistoryMs[angleHistoryIndex] = aimClock.milliseconds();
            angleHistoryDeg[angleHistoryIndex] = turretAngleDeg;
            angleHistoryIndex = (angleHistoryIndex + 1) % ANGLE_HISTORY_SIZE;
            if (angleHistoryCount < ANGLE_HISTORY_SIZE) {
                angleHistoryCount++;
            }

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

                // Diagnóstico: por qué tagVisible puede dar false aunque la Limelight sí detecte
                // (se ve en su propia página web). Se muestra siempre en telemetría, abajo.
                lastLLResultIsNull = result == null;
                if (result != null) {
                    lastLLValid = result.isValid();
                    lastLLTa = result.getTa();
                    lastLLStaleness = result.getStaleness();
                    lastLLFiducialCount = result.getFiducialResults() != null
                            ? result.getFiducialResults().size() : 0;
                } else {
                    lastLLValid = false;
                    lastLLTa = 0.0;
                    lastLLStaleness = -1;
                    lastLLFiducialCount = 0;
                }

                if (tagVisible) {
                    tagLostTimer.reset();
                    pipelineTimer.reset(); // con tag visible se queda en el pipeline actual
                    double tx = result.getTx();

                    // Frame nuevo (la antigüedad del dato bajó de golpe): la visión fija el
                    // ángulo objetivo. Se parte del ángulo que tenía la torreta CUANDO se tomó
                    // la foto, no del de ahora, porque en ese rato la torreta ya se movió.
                    if (lastLLStaleness < lastStalenessForFrame) {
                        double angleAtCapture = turretAngleAtMsAgo(lastLLStaleness, turretAngleDeg);
                        double newTarget = angleAtCapture + tx;

                        // Qué tan rápido se está moviendo el objetivo, comparando contra el frame
                        // anterior. El límite de 0.5 s evita calcular una velocidad absurda cuando
                        // se reengancha el tag después de haberlo perdido un rato.
                        double dtFrame = aimFrameTimer.seconds();
                        if (aimTargetValid && dtFrame > 0.001 && dtFrame < 0.5) {
                            double rawRate = (newTarget - aimTargetAngleDeg) / dtFrame;
                            aimTargetRateDegPerSec +=
                                    AIM_RATE_FILTER * (rawRate - aimTargetRateDegPerSec);
                        } else {
                            aimTargetRateDegPerSec = 0.0;
                        }
                        aimFrameTimer.reset();

                        aimTargetAngleDeg = newTarget;
                        aimTargetValid = true;
                    }
                    lastStalenessForFrame = lastLLStaleness;

                    desiredTurretPower = aimPower(aimTargetAngleDeg, turretAngleDeg, gyroFeedForward);

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
                        // Parpadeo corto del tag: sigue persiguiendo el último ángulo conocido
                        // en vez de frenarse, así termina el movimiento que traía.
                        desiredTurretPower = aimTargetValid
                                ? aimPower(aimTargetAngleDeg, turretAngleDeg, gyroFeedForward)
                                : gyroFeedForward;
                        if ((turretAngleDeg >= AUTO_AIM_LIMIT_DEG && desiredTurretPower > 0)
                                || (turretAngleDeg <= -AUTO_AIM_LIMIT_DEG && desiredTurretPower < 0)) {
                            desiredTurretPower = 0.0;
                            targetOutOfRange = true;
                        }
                    } else {
                        aimTargetValid = false; // el objetivo viejo ya no sirve
                        aimTargetRateDegPerSec = 0.0;

                        // Sin tag, la torreta NO se mueve sola: solo compensa el giro del chasis
                        // para no perder hacia dónde apunta. El barrido de búsqueda es a propósito,
                        // manteniendo B (gamepad 2), para encontrar el tag de la cara de atrás del
                        // gol. Antes barría sola al no ver nada, y al arrancar el match se iba
                        // girando a la derecha sin que nadie se lo pidiera.
                        if (gamepad2.b) {
                            if (turretAngleDeg >= AUTO_AIM_LIMIT_DEG) {
                                sweepDirectionRight = false;
                            } else if (turretAngleDeg <= -AUTO_AIM_LIMIT_DEG) {
                                sweepDirectionRight = true;
                            }
                            desiredTurretPower =
                                    (sweepDirectionRight ? SWEEP_POWER : -SWEEP_POWER) + gyroFeedForward;
                        } else {
                            desiredTurretPower = gyroFeedForward;
                        }
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

            // Ajuste en vivo de kP/kI/kV/boost de ráfaga: LB/RB elige el parámetro,
            // Y sube y A baja ese valor (x1.1 / ÷1.1 por cada pulsación).
            boolean curTuneLeft = gamepad2.left_bumper;
            if (curTuneLeft && !lastTuneLeft) {
                tunableIndex = (tunableIndex + TUNABLE_NAMES.length - 1) % TUNABLE_NAMES.length;
            }
            lastTuneLeft = curTuneLeft;

            boolean curTuneRight = gamepad2.right_bumper;
            if (curTuneRight && !lastTuneRight) {
                tunableIndex = (tunableIndex + 1) % TUNABLE_NAMES.length;
            }
            lastTuneRight = curTuneRight;

            boolean curTuneUp = gamepad2.y;
            boolean curTuneDown = gamepad2.a;
            if (curTuneUp && !lastTuneUp) {
                switch (tunableIndex) {
                    case 0: kP *= 1.1; break;
                    case 1: kI *= 1.1; break;
                    case 2: kV *= 1.1; break;
                    case 3: burstBoostFraction *= 1.1; break;
                    case 4: aimKp *= 1.1; break;
                    case 5: aimKd *= 1.1; break;
                    case 6: minTurretPower *= 1.1; break;
                }
            } else if (curTuneDown && !lastTuneDown) {
                switch (tunableIndex) {
                    case 0: kP /= 1.1; break;
                    case 1: kI /= 1.1; break;
                    case 2: kV /= 1.1; break;
                    case 3: burstBoostFraction /= 1.1; break;
                    case 4: aimKp /= 1.1; break;
                    case 5: aimKd /= 1.1; break;
                    case 6: minTurretPower /= 1.1; break;
                }
            }
            lastTuneUp = curTuneUp;
            lastTuneDown = curTuneDown;

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
            double effTarget = burstActive ? targetRPM * (1.0 + burstBoostFraction) : targetRPM;

            double error = effTarget - currentRPM;
            double dt = pidTimer.seconds();
            pidTimer.reset();

            // Derivada sobre la medición: no da picos cuando effTarget salta por el boost
            double derivative = dt > 0 ? -(currentRPM - lastRPM) / dt : 0.0;
            lastRPM = currentRPM;

            if (targetRPM <= 0) {
                shooter.setPower(0.0);
                lastShooterPower = 0.0;
                integralSum = 0.0;
                recovering = false;
            } else {
                // Histéresis de la recuperación agresiva. Solo se usa en ráfagas, cuando el shooter
                // ya llegó al objetivo (burstArmed): el arranque y los cambios de objetivo van con PID.
                if (!burstArmed) {
                    recovering = false;
                } else if (!recovering && currentRPM < effTarget * (1.0 - RECOVERY_ENTER_FRACTION)) {
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
                lastShooterPower = recovering ? RECOVERY_POWER : calculatedPower;
                shooter.setPower(lastShooterPower);
            }

            // =========================================================
            // 4. GAMEPAD 1: CHASIS (más lento con "A") E INTAKE DE DOS MOTORES (AL 80%)
            // =========================================================
            // El bumper derecho se movió a "A" porque R1/L1 ahora controlan el intake.
            double driveScale = gamepad1.a ? SHOOT_SPEED_SCALE : 1.0;

            // Field-centric siempre activo. Con "Y" (gamepad 1) y el robot mirando al frente del
            // campo, se recalibra el yaw del IMU. Se movió aquí desde left_bumper, que ahora
            // suelta los dos motores del intake.
            if (gamepad1.y) {
                imu.resetYaw();
                startHeading = 0.0;
            }
            double hRad = Math.toRadians(yaw + startHeading);
            double fieldFwd = -gamepad1.left_stick_y * driveScale;  // adelante del campo
            double fieldRight = gamepad1.left_stick_x * driveScale; // derecha del campo
            double robotFwd = fieldFwd * Math.cos(hRad) - fieldRight * Math.sin(hRad);
            double robotRight = fieldFwd * Math.sin(hRad) + fieldRight * Math.cos(hRad);
            chassis.move2(-robotFwd, robotRight, gamepad1.right_stick_x * driveScale);

            // Intake:
            // - R1 (right_bumper) agarra con mI1 y mI2 juntos; L1 (left_bumper) suelta los dos juntos.
            // - R2 (right_trigger) agarra SOLO con mI1; L2 (left_trigger) suelta SOLO con mI1.
            // Los gatillos (R2/L2) mandan sobre los bumpers en mI1, por si se presionan a la vez.
            double mi1Power = 0.0;
            double mi2Power = 0.0;

            if (gamepad1.right_bumper) {
                mi1Power = 1.0 * INTAKE_POWER_SCALE;
                mi2Power = 1.0 * INTAKE_POWER_SCALE;
            } else if (gamepad1.left_bumper) {
                mi1Power = -1.0 * INTAKE_POWER_SCALE;
                mi2Power = -1.0 * INTAKE_POWER_SCALE;
            }

            if (gamepad1.right_trigger > 0.2) {
                mi1Power = 1.0 * INTAKE_POWER_SCALE;
            } else if (gamepad1.left_trigger > 0.2) {
                mi1Power = -1.0 * INTAKE_POWER_SCALE;
            }

            robot.mI1.setPower(mi1Power);
            robot.mI2.setPower(mi2Power);

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
                else if (tagLostTimer.seconds() <= TAG_MEMORY_TIMEOUT) modo = "AUTO (Locked)";
                else if (gamepad2.b) modo = "BUSCANDO (B)";
                else modo = "SIN TAG - quieta (B para buscar)";
                telemetry.addData("Modo Torreta", modo);
                telemetry.addData("Pipeline Limelight", currentPipeline);
                telemetry.addData("LL diagnóstico", "null=%b válido=%b ta=%.3f staleness=%.0fms tags=%d",
                        lastLLResultIsNull, lastLLValid, lastLLTa, lastLLStaleness, lastLLFiducialCount);
                telemetry.addData("Aim objetivo / actual / error", "%.1f° / %.1f° / %.1f°",
                        aimTargetAngleDeg, turretAngleDeg,
                        aimTargetValid ? (aimTargetAngleDeg - turretAngleDeg) : 0.0);
                telemetry.addData("Velocidad torreta / objetivo", "%.0f / %.0f °/s",
                        turretRateDegPerSec, aimTargetRateDegPerSec);
                telemetry.addData("Ángulo Torreta", "%.1f° (Max ±%.0f°)", turretAngleDeg, MAX_TURRET_ANGLE_DEG);
                telemetry.addData("Shooter Target / Actual", "%.0f / %.0f RPM", targetRPM, currentRPM);
                telemetry.addData("Objetivo efectivo", "%.0f RPM%s", effTarget, burstActive ? " (BOOST ráfaga)" : "");
                telemetry.addData("Shooter listo", recovering ? "RECUPERANDO..." : (rpmEnRango ? "SÍ (RPM OK)" : "ESPERANDO..."));
                telemetry.addData("Shooter potencia / ticks/s / posición", "%.2f / %.0f / %d",
                        lastShooterPower, shooter.getVelocity(), shooter.getCurrentPosition());
                telemetry.addData("Ajuste en vivo (LB/RB elige, Y sube, A baja)",
                        "%s = %.6f", TUNABLE_NAMES[tunableIndex], getTunableValue(tunableIndex));
                telemetry.update();
                telemetryTimer.reset();
            }
        }

        chassis.stopChassis();
        robot.mI1.setPower(0.0);
        robot.mI2.setPower(0.0);
        shooter.setPower(0.0);
        turret.setPower(0.0);
        limelight.stop();
    }

    /**
     * Potencia para llevar la torreta al ángulo objetivo: PD sobre el encoder.
     * El término derivativo usa la velocidad real medida por el encoder (no la derivada de tx),
     * así frena al ritmo del loop y no al de la cámara.
     */
    private double aimPower(double targetAngleDeg, double currentAngleDeg, double gyroFeedForward) {
        double angleError = targetAngleDeg - currentAngleDeg;
        if (Math.abs(angleError) <= DEADZONE_TX) {
            return gyroFeedForward;
        }

        // El frenado va sobre la velocidad RELATIVA: si el objetivo se mueve, la torreta no es
        // frenada por acompañarlo, solo por ir más rápido o más lento que él. Con el objetivo
        // quieto es exactamente el frenado de antes.
        double rateError = aimTargetRateDegPerSec - turretRateDegPerSec;
        double power = (angleError * aimKp) + (rateError * aimKd) + gyroFeedForward;
        power = clamp(power, MAX_TURRET_POWER);

        // La potencia mínima solo cuando la torreta está prácticamente detenida: es para romper
        // la fricción de arranque. Si se aplicara también en movimiento taparía el frenado del
        // término derivativo y volvería a pasarse del objetivo.
        if (Math.abs(power) < minTurretPower && Math.abs(turretRateDegPerSec) < STATIC_RATE_DEG_PER_SEC) {
            power = (angleError > 0) ? minTurretPower : -minTurretPower;
        }
        return power;
    }

    /** Ángulo que tenía la torreta hace msAgo milisegundos, del historial. */
    private double turretAngleAtMsAgo(double msAgo, double fallbackAngleDeg) {
        if (angleHistoryCount == 0) {
            return fallbackAngleDeg;
        }
        double targetMs = aimClock.milliseconds() - Math.max(0.0, msAgo);
        double bestAngle = fallbackAngleDeg;
        double bestDiff = Double.MAX_VALUE;
        for (int i = 0; i < angleHistoryCount; i++) {
            double diff = Math.abs(angleHistoryMs[i] - targetMs);
            if (diff < bestDiff) {
                bestDiff = diff;
                bestAngle = angleHistoryDeg[i];
            }
        }
        return bestAngle;
    }

    private double getTunableValue(int index) {
        switch (index) {
            case 0: return kP;
            case 1: return kI;
            case 2: return kV;
            case 3: return burstBoostFraction;
            case 4: return aimKp;
            case 5: return aimKd;
            case 6: return minTurretPower;
            default: return 0.0;
        }
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
        // Valor absoluto: si el encoder cuenta al revés que la potencia (cables del motor invertidos),
        // la velocidad sale negativa y el PID creería que el shooter está parado y le daría todo.
        double ticksPerSec = Math.abs(shooter.getVelocity());
        return (ticksPerSec / ENCODER_CPR) * 60.0 / ULTRA_RATIO;
    }
}
