with Ada.Text_IO;
with Interfaces.C;
with OpenCV.Calib3D;

procedure Run_Refinement_Faults is
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   use type Interfaces.C.int;
   procedure Fail (Stage, Kind : Interfaces.C.int)
     with Import, Convention => C, External_Name => "opencv_calib3d_test_fail";
   procedure Return_False
     with Import, Convention => C,
       External_Name => "opencv_calib3d_test_refinement_false";
   K : constant Camera_Intrinsics := (800.0, 820.0, 320.0, 240.0);
   Truth : constant World_To_Camera_Pose :=
     (Rotation => [0.10, -0.05, 0.08], Translation => [0.20, -0.10, 6.00]);
   Initial : constant World_To_Camera_Pose :=
     (Rotation => [0.16, -0.09, 0.12], Translation => [0.45, -0.30, 6.40]);
   World : constant Object_Point_Array :=
     [[-1.0, -1.0, 0.0], [0.0, -1.0, 0.2], [1.0, -1.0, 0.4],
      [-1.2, 0.0, 0.5], [0.0, 0.0, 0.8], [1.2, 0.0, 0.3],
      [-1.0, 1.0, 1.0], [0.0, 1.0, 1.3], [1.0, 1.0, 0.7]];
   Images : constant Image_Point_Array := Project_Points (World, K, No_Distortion, Truth);
   P : World_To_Camera_Pose := Initial;
   Refined : Boolean := True;
   procedure Check (Condition : Boolean; Message : String) is
   begin
      if not Condition then
         raise Program_Error with Message;
      end if;
   end Check;
begin
   Return_False;
   Refine_Pose_Iterative (World, Images, K, Pose => P, Refined => Refined);
   Check (not Refined and then P = Initial, "false result changed Ada pose/flag");
   for Stage in Interfaces.C.int range 10 .. 12 loop
      for Kind in Interfaces.C.int range 1 .. 5 loop
         P := Initial;
         --  Boolean out parameters are by-copy; on exception Ada does not
         --  guarantee copy-back. Initialize the caller to False accordingly.
         Refined := False;
         Fail (Stage, Kind);
         begin
            Refine_Pose_Iterative (World, Images, K, Pose => P, Refined => Refined);
            raise Program_Error with "injected refinement did not raise OpenCV_Error";
         exception
            when OpenCV.OpenCV_Error => null;
         end;
         Check (P = Initial and then not Refined, "exception changed Ada pose/flag");
      end loop;
   end loop;
   Ada.Text_IO.Put_Line ("PASS: Ada refinement false + 15 exception atomicity scenarios");
   --  Reuse this helper and its macOS libc++-isolated fault dylib.
   for Stage in Interfaces.C.int range 13 .. 16 loop
      for Kind in Interfaces.C.int range 1 .. 5 loop
         Fail (Stage, Kind);
         begin
            if Stage <= 14 then
               declare
                  Values : constant Normalized_Image_Point_Array :=
                    Undistort_To_Normalized ([[320.0, 240.0]], K);
                  pragma Unreferenced (Values);
               begin
                  null;
               end;
            else
               declare
                  Matrix : constant Rotation_Matrix := Rotation_Matrix_Of (Truth);
                  pragma Unreferenced (Matrix);
               begin
                  null;
               end;
            end if;
            raise Program_Error with "camera geometry injection did not raise OpenCV_Error";
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end loop;
   end loop;
   Ada.Text_IO.Put_Line ("PASS: Ada camera geometry 20 exception translation scenarios");
end Run_Refinement_Faults;