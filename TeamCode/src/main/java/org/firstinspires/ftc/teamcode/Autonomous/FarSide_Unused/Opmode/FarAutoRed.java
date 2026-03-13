package org.firstinspires.ftc.teamcode.Autonomous.FarSide_Unused.Opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.FarSide_Unused.Paths.FarPaths;
import org.firstinspires.ftc.teamcode.Autonomous.FarSide_Unused.Paths.FarPathsRed;

//efn
@Autonomous(name = "Keir Far Red", preselectTeleOp = "TeleOp")
public class FarAutoRed extends FarAuto {
    @Override
    public FarPaths getPaths() {
        return new FarPathsRed(pedro);
    }
}
