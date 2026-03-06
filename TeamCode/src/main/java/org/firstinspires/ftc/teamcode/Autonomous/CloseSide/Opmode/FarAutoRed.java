package org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.Paths;
import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.PathsFarRed;
import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.PathsRed;

@Autonomous(name = "Red Far", preselectTeleOp = "TeleOp")

public class FarAutoRed extends FarAuto {
    @Override
    public Paths getPaths() {
        return new PathsFarRed(pedro);
    }
}
