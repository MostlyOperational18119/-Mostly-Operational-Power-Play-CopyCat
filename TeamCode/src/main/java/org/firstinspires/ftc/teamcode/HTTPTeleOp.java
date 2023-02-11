package org.firstinspires.ftc.teamcode;

public class HTTPTeleOp extends DriveMethods {
    private int port;
    private String ipaddr;

    @Override
    public void runOpMode() {
        ipaddr = "192.168.43.1";
        port = 6969;

        telemetry.addLine("Init");
        telemetry.update();
        waitForStart();
        TinyWebServer.startServer(ipaddr,port,"/Users/damienbrown/Desktop/web/public_html");
        while (opModeIsActive()) {
            telemetry.addLine("Still active :|");
            telemetry.addLine("Hosting at " + ipaddr + ":" + port + ".");
            telemetry.update();
        }
        TinyWebServer.stopServer();
    }
}
