package org.firstinspires.ftc.teamcode.Init;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Prueba del encoder del shooter, sin Chassis, Robot, Limelight ni PID.
 *
 * - Gatillo derecho (gamepad 1): potencia al shooter (0 a 0.5).
 * - Con el robot deshabilitado del gatillo, también puedes girar el flywheel con la mano.
 *
 * Muestra la posición y velocidad del encoder de TODOS los motores configurados,
 * así se ve en qué puerto está llegando realmente la señal del encoder del shooter.
 */
@TeleOp(name = "Prueba Encoder Shooter", group = "Pruebas")
public class ShooterEncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Listo. Gatillo derecho = potencia al shooter, o gira el flywheel a mano.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double power = gamepad1.right_trigger * 0.5;
            shooter.setPower(power);

            double tps = shooter.getVelocity();
            telemetry.addData("Shooter potencia", "%.2f", power);
            telemetry.addData("Shooter posición", shooter.getCurrentPosition());
            telemetry.addData("Shooter ticks/s", "%.0f", tps);
            telemetry.addData("Shooter RPM motor (28 CPR)", "%.0f", tps / 28.0 * 60.0);
            telemetry.addLine();
            telemetry.addLine("Encoders de todos los motores:");
            for (String name : hardwareMap.getAllNames(DcMotor.class)) {
                DcMotor m = hardwareMap.get(DcMotor.class, name);
                telemetry.addData(name, "puerto %d  pos %d", m.getPortNumber(), m.getCurrentPosition());
            }
            telemetry.update();
        }
        shooter.setPower(0);
    }
}
