package org.firstinspires.ftc.teamcode.Init;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Calibración del GEAR_RATIO real de la torreta. NUNCA le da potencia al motor: solo lee el
 * encoder, para que gires la torreta a mano por un ángulo exacto (con una escuadra, por ejemplo
 * 90°) sin que el código la mueva ni la frene.
 *
 * Cómo usarlo:
 * 1. Corre este OpMode. La torreta queda libre (ZeroPowerBehavior.FLOAT), gírala a mano con cuidado.
 * 2. Ponla en un punto de referencia (0°) y anota "Posición (ticks)".
 * 3. Gírala a mano EXACTAMENTE 90° (o el ángulo que puedas medir con precisión) y anota la nueva
 *    posición.
 * 4. GEAR_RATIO real = (ticks girados / MOTOR_TICKS_PER_REV) * 360 / ángulo_real_girado / 1.0
 *    Esta pantalla ya hace esa cuenta sola con el campo "Grados asumiendo GEAR_RATIO=1.0": compara
 *    ese número contra el ángulo real que giraste a mano para sacar el factor de corrección.
 */
@TeleOp(name = "Prueba Encoder Torreta", group = "Pruebas")
public class TurretEncoderTest extends LinearOpMode {

    private static final double MOTOR_TICKS_PER_REV = 537.7;

    @Override
    public void runOpMode() {
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turret.setPower(0.0);

        telemetry.addLine("Listo. Gira la torreta A MANO. El código nunca le da potencia.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Nunca se llama turret.setPower(): el motor queda libre todo el tiempo.
            int ticks = turret.getCurrentPosition();
            double gradosConRatio1 = (ticks / MOTOR_TICKS_PER_REV) * 360.0;

            telemetry.addData("Posición (ticks)", ticks);
            telemetry.addData("Grados asumiendo GEAR_RATIO=1.0", "%.2f", gradosConRatio1);
            telemetry.addLine();
            telemetry.addLine("Gira a 0°, anota los ticks. Gira un ángulo EXACTO conocido (ej. 90°");
            telemetry.addLine("con una escuadra), anota los ticks de nuevo. GEAR_RATIO real =");
            telemetry.addLine("(grados mostrados arriba, la diferencia) / (ángulo real que giraste).");
            telemetry.update();
        }
    }
}
