package de.fhg.iais.roberta.ast.syntax.actors;

import de.fhg.iais.roberta.util.RobertaProperties;
import de.fhg.iais.roberta.util.Util1;
import de.fhg.iais.roberta.util.test.nxt.HelperNxtForTest;

public class DriveActionTest {
    HelperNxtForTest h = new HelperNxtForTest(new RobertaProperties(Util1.loadProperties(null)));

    //
    public void drive() throws Exception {
        final String a = "OnFwdReg(OUT_BC,50,OUT_REGMODE_SYNC)";

        this.h.assertCodeIsOk(a, "/ast/actions/action_MotorDiffOn.xml");
    }

    //
    public void driveFor() throws Exception {
        final String a = "\nRotateMotorEx(OUT_BC,50,Infinity*20,0,true,true)";
        this.h.assertCodeIsOk(a, "/ast/actions/action_MotorDiffOnFor.xml");
    }
}