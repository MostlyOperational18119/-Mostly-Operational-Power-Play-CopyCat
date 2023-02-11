package TestPrograms;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="BlinkinTeleOp")
public class BlinkinTeleOp extends LinearOpMode {
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    @Override
    public void runOpMode() {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        telemetry.addLine("Initialized LED driver.");
        telemetry.update();
        waitForStart();
        pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN;

        while(opModeIsActive()) {
            if(gamepad1.a) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN;
                blinkinLedDriver.setPattern(pattern);
            } if(gamepad1.b) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN;
                blinkinLedDriver.setPattern(pattern);
            }
            // Please let me debug Blinkin.
            telemetry.addData("Pattern Name",pattern.name());
            telemetry.addData("Blinkin Connection Info",blinkinLedDriver.getConnectionInfo());
            telemetry.addData("Blinkin Device Name",blinkinLedDriver.getDeviceName());
            telemetry.update();
        }
    }
}
