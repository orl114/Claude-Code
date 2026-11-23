package org.firstinspires.ftc.teamcode.Init;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Prueba del servo lineal "servoEmpuje1", sin chasis, intake, shooter, torreta ni Limelight.
 *
 * - Botón X (gamepad 1): alterna entre metido y sacado cada vez que se presiona
 *   (no se repite si se mantiene presionado).
 *
 * Usa esto para calibrar POSICION_ADENTRO y POSICION_AFUERA antes de pasarlos al TeleOp principal.
 */
@TeleOp(name = "Prueba Servo Empuje", group = "Pruebas")
public class ServoEmpujeTest extends LinearOpMode {

    // Ajusta estos dos valores durante la prueba, hasta que el recorrido sea completo sin forzar
    // el servo contra su tope mecánico.
    private static final double POSICION_ADENTRO = 0.0;
    private static final double POSICION_AFUERA = 1.0;

    @Override
    public void runOpMode() {
        Servo empujador = hardwareMap.get(Servo.class, "servoEmpuje1");
        empujador.setPosition(POSICION_ADENTRO);

        boolean afuera = false;
        boolean lastButton = false;

        telemetry.addLine("Listo. X = alterna metido/sacado.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean currentButton = gamepad1.x;
            if (currentButton && !lastButton) {
                afuera = !afuera;
                empujador.setPosition(afuera ? POSICION_AFUERA : POSICION_ADENTRO);
            }
            lastButton = currentButton;

            telemetry.addData("Estado", afuera ? "AFUERA" : "ADENTRO");
            telemetry.addData("Posición enviada", "%.2f", empujador.getPosition());
            telemetry.update();
        }

        empujador.setPosition(POSICION_ADENTRO);
    }
}
