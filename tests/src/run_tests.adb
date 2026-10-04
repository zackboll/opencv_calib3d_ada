with AUnit.Run;
with AUnit.Reporter.Text;
with Calib3D_Tests;

procedure Run_Tests is
   procedure Runner is new AUnit.Run.Test_Runner (Calib3D_Tests.Suite);
   Reporter : AUnit.Reporter.Text.Text_Reporter;
begin
   Runner (Reporter);
end Run_Tests;
