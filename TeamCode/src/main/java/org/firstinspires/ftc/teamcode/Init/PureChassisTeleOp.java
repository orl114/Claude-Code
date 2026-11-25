package org.firstinspires.ftc.teamcode.Init;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * TeleOp de puro chasis (mecanum), sin intake, shooter, torreta ni Limelight.
 *
 * Configuración de hardware esperada (nombres exactos): "frontLeft", "frontRight",
 * "backLeft", "backRight".
 *
 * Control (gamepad 1):
 * - Stick izquierdo: mover (adelante/atrás y de lado, strafe).
 * - Stick derecho (eje X): girar.
 * - Bumper derecho: velocidad reducida.
 */
@TeleOp(name = "Prueba Chasis Puro", group = "Pruebas")
public class PureChassisTeleOp extends LinearOpMode {

    private static final double SLOW_SPEED_SCALE = 0.45;

    @Override
    public void runOpMode() {
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // Ajusta estas direcciones si el robot se mueve al revés de lo esperado.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Sin esto, el hub intenta regular la potencia con el encoder de cada motor
        // (RUN_USING_ENCODER, el modo por defecto): si algún encoder no responde igual de bien
        // que los demás, cada rueda recibe potencia distinta aunque pidas lo mismo, y el robot
        // va lento y se desvía en vez de ir derecho.
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Listo. Chasis puro (mecanum).");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double speedScale = gamepad1.right_bumper ? SLOW_SPEED_SCALE : 1.0;

            double y = -gamepad1.left_stick_y;        // adelante/atrás
            double x = gamepad1.left_stick_x * 1.1;    // strafe (1.1 corrige que las mecanum
            double rx = gamepad1.right_stick_x;        // giro                        strafean un poco más débil)

            double frontLeftPower = (y + x + rx) * speedScale;
            double backLeftPower = (y - x + rx) * speedScale;
            double frontRightPower = (y - x - rx) * speedScale;
            double backRightPower = (y + x - rx) * speedScale;

            double max = Math.max(1.0, Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                    Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
            ));

            frontLeft.setPower(frontLeftPower / max);
            backLeft.setPower(backLeftPower / max);
            frontRight.setPower(frontRightPower / max);
            backRight.setPower(backRightPower / max);

            telemetry.addData("FL / FR", "%.2f / %.2f", frontLeftPower / max, frontRightPower / max);
            telemetry.addData("BL / BR", "%.2f / %.2f", backLeftPower / max, backRightPower / max);
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
