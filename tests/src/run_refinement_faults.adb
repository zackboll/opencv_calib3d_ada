with Ada.Text_IO;
with Ada.Numerics.Generic_Elementary_Functions;
with Interfaces.C;
with OpenCV.Calib3D;

procedure Run_Refinement_Faults is
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   use type Interfaces.C.int;
   use type Interfaces.C.double;
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
   declare
      H : constant Homography_Matrix := [[1.0, 0.0, -0.12], [0.0, 1.0, 0.04], [0.0, 0.0, 1.08]];
      Identity : constant Camera_Intrinsics := (1.0, 1.0, 0.0, 0.0);
      procedure Control (Mode : Interfaces.C.int)
        with Import, Convention => C, External_Name => "opencv_calib3d_test_planar_control";
   begin
      for Stage in Interfaces.C.int range 42 .. 45 loop
         for Kind in Interfaces.C.int range 1 .. 5 loop
            Fail (Stage, Kind);
            begin
               declare
                  V : constant Planar_Motion_Hypothesis_Array := Decompose_Calibrated_Homography (H, Identity);
                  pragma Unreferenced (V);
               begin
                  raise Program_Error with "planar injection accepted";
               end;
            exception
               when OpenCV.OpenCV_Error => null;
            end;
         end loop;
      end loop;
      Control (1);
      declare
         V : constant Planar_Motion_Hypothesis_Array := Decompose_Calibrated_Homography (H, Identity);
      begin
         Check (V'First = 1 and then V'Length = 0, "test-only post-native zero result");
      end;
      for Mode in Interfaces.C.int range 2 .. 4 loop
         Control (Mode);
         begin
            declare
               V : constant Planar_Motion_Hypothesis_Array := Decompose_Calibrated_Homography (H, Identity);
               pragma Unreferenced (V);
            begin
               raise Program_Error with "malformed native planar accepted";
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end loop;
      Ada.Text_IO.Put_Line ("PASS: Ada planar 20 exceptions + test-only post-native zero/malformed controls");
   end;
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
   declare
      Source : constant Image_Point_Array :=
        [[0.0, 0.0], [100.0, 0.0], [0.0, 100.0], [100.0, 100.0],
         [40.0, 30.0], [70.0, 90.0]];
      Destination : Image_Point_Array (Source'Range);
   begin
      for I in Source'Range loop
         Destination (I) := [Source (I) (0) + 20.0, Source (I) (1) - 10.0];
      end loop;
      --  An armed checkpoint remains pending after Ada rejects four, proving
      --  rejection precedes native entry; a valid five-point call consumes it.
      Fail (17, 3);
      begin
         declare
            Estimate : constant Homography_Estimate := Estimate_Homography_RANSAC
              (Source (1 .. 4), Destination (1 .. 4));
            pragma Unreferenced (Estimate);
         begin
            raise Program_Error with "four points reached robust estimator";
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
      begin
         declare
            Estimate : constant Homography_Estimate := Estimate_Homography_RANSAC
              (Source (1 .. 5), Destination (1 .. 5));
            pragma Unreferenced (Estimate);
         begin
            raise Program_Error with "five did not consume pending native checkpoint";
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
      for Stage in Interfaces.C.int range 17 .. 22 loop
         for Kind in Interfaces.C.int range 1 .. 5 loop
            Fail (Stage, Kind);
            begin
               declare
                  Estimate : constant Homography_Estimate := Estimate_Homography_RANSAC
                    (Source, Destination, (2_000, 0.1, 0.999));
                  pragma Unreferenced (Estimate);
               begin
                  raise Program_Error with "homography injection did not raise OpenCV_Error";
               end;
            exception
               when OpenCV.OpenCV_Error => null;
            end;
         end loop;
      end loop;
   end;
   Ada.Text_IO.Put_Line ("PASS: Ada homography 30 exception translation/cleanup scenarios");
   Ada.Text_IO.Put_Line ("PASS: four-point Ada rejection before native entry; five reaches native checkpoint");
   declare
      First, Second : Image_Point_Array (1 .. 32);
   begin
      for I in First'Range loop
         declare
            J : constant Natural := I - 1;
            X : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((J*7) mod 17 - 8)*0.24;
            Y : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((J*11) mod 19 - 9)*0.19;
            Z : constant OpenCV.Float64_Value := 4.0 + OpenCV.Float64_Value ((J*13) mod 23)*0.17;
         begin
            First (I) := [800.0*X/Z+320.0, 820.0*Y/Z+240.0];
            Second (I) := [800.0*(X-0.75)/Z+320.0, 820.0*Y/Z+240.0];
         end;
      end loop;
      Fail (23, 3);
      begin
         declare
            E : constant Fundamental_Estimate := Estimate_Fundamental_RANSAC
              (First (1 .. 14), Second (1 .. 14));
            pragma Unreferenced (E);
         begin
            raise Program_Error with "14 fundamental pairs reached native entry";
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
      begin
         declare
            E : constant Fundamental_Estimate := Estimate_Fundamental_RANSAC
              (First (1 .. 15), Second (1 .. 15));
            pragma Unreferenced (E);
         begin
            raise Program_Error with "15 did not consume pending fundamental checkpoint";
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
      --  A subsequent valid call proves the pending checkpoint was consumed.
      declare
         E : constant Fundamental_Estimate := Estimate_Fundamental_RANSAC (First, Second, (0.1, 0.999));
      begin
         Check (Found (E), "checkpoint remained armed after 15");
      end;
      for Stage in Interfaces.C.int range 23 .. 28 loop
         for Kind in Interfaces.C.int range 1 .. 5 loop
            Fail (Stage, Kind);
            begin
               declare
                  E : constant Fundamental_Estimate := Estimate_Fundamental_RANSAC (First, Second, (0.1, 0.999));
                  pragma Unreferenced (E);
               begin
                  raise Program_Error with "fundamental fault did not raise OpenCV_Error";
               end;
            exception
               when OpenCV.OpenCV_Error => null;
            end;
         end loop;
      end loop;
   end;
   Ada.Text_IO.Put_Line ("PASS: Ada fundamental 30 exception translation/cleanup scenarios");
   Ada.Text_IO.Put_Line ("PASS: 14-point Ada rejection preserves checkpoint; 15 true RANSAC consumes it");
   declare
      package Math is new Ada.Numerics.Generic_Elementary_Functions (OpenCV.Float64_Value);
      procedure No_E with Import, Convention => C, External_Name => "opencv_calib3d_test_essential_no_model";
      procedure No_Pose with Import, Convention => C, External_Name => "opencv_calib3d_test_essential_no_pose";
      First, Second : Normalized_Image_Point_Array (1 .. 40);
      Options : constant Essential_RANSAC_Options := (1.0E-6, 0.999);
   begin
      for I in First'Range loop
         declare
            J : constant Integer := I - 1;
            X : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((J * 7) mod 17 - 8) * 0.24;
            Y : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((J * 11) mod 19 - 9) * 0.19;
            Z : constant OpenCV.Float64_Value := 4.0 + OpenCV.Float64_Value ((J * 13) mod 23) * 0.17;
            X2 : constant OpenCV.Float64_Value := Math.Cos (0.08) * X + Math.Sin (0.08) * Z - 0.75;
            Z2 : constant OpenCV.Float64_Value := -Math.Sin (0.08) * X + Math.Cos (0.08) * Z + 0.12;
         begin
            First (I) := [X / Z, Y / Z];
            Second (I) := [X2 / Z2, (Y + 0.08) / Z2];
         end;
      end loop;
      Fail (29, 3);
      begin
         declare
            E : constant Essential_Estimate := Estimate_Essential_RANSAC (First (1 .. 5), Second (1 .. 5), Options);
            pragma Unreferenced (E);
         begin
            raise Program_Error with "five Essential pairs reached native entry";
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
      begin
         declare
            E : constant Essential_Estimate := Estimate_Essential_RANSAC (First (1 .. 6), Second (1 .. 6), Options);
            pragma Unreferenced (E);
         begin
            raise Program_Error with "six did not consume Essential checkpoint";
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
      declare
         E : constant Essential_Estimate := Estimate_Essential_RANSAC (First, Second, Options);
      begin
         Check (Found (E) and then Pose_Recovered (E), "Essential checkpoint remained armed after six");
      end;
      No_E;
      declare
         E : constant Essential_Estimate := Estimate_Essential_RANSAC (First, Second, Options);
         EI : constant Inlier_Index_Array := Inliers (E);
         PI : constant Inlier_Index_Array := Pose_Inliers (E);
      begin
         Check (not Found (E) and then not Pose_Recovered (E) and then Inlier_Count (E) = 0 and then
           Pose_Inlier_Count (E) = 0 and then EI'First = 1 and then EI'Last = 0 and then PI'First = 1 and then PI'Last = 0,
           "Ada no-E semantics after real native call");
         begin
            declare
               Matrix : constant Essential_Matrix := Essential (E);
               pragma Unreferenced (Matrix);
            begin
               raise Program_Error with "no-E matrix accessible";
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
         begin
            declare
               Pose : constant Relative_Camera_Pose := Recovered_Pose (E);
               pragma Unreferenced (Pose);
            begin
               raise Program_Error with "no-E pose accessible";
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end;
      No_Pose;
      declare
         E : constant Essential_Estimate := Estimate_Essential_RANSAC (First, Second, Options);
         Matrix : constant Essential_Matrix := Essential (E);
         EI : constant Inlier_Index_Array := Inliers (E);
         PI : constant Inlier_Index_Array := Pose_Inliers (E);
         pragma Unreferenced (Matrix);
      begin
         Check (Found (E) and then EI'Length >= 5 and then Inlier_Count (E) = EI'Length and then
           not Pose_Recovered (E) and then Pose_Inlier_Count (E) = 0 and then PI'First = 1 and then PI'Last = 0,
           "Ada Essential retained after real recoverPose with support suppression");
         begin
            declare
               Pose : constant Relative_Camera_Pose := Recovered_Pose (E);
               pragma Unreferenced (Pose);
            begin
               raise Program_Error with "suppressed pose accessible";
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end;
      for Stage in Interfaces.C.int range 29 .. 36 loop
         for Kind in Interfaces.C.int range 1 .. 5 loop
            Fail (Stage, Kind);
            begin
               declare
                  E : constant Essential_Estimate := Estimate_Essential_RANSAC (First, Second, Options);
                  pragma Unreferenced (E);
               begin
                  raise Program_Error with "Essential injection did not raise OpenCV_Error";
               end;
            exception
               when OpenCV.OpenCV_Error => null;
            end;
         end loop;
      end loop;
   end;
   Ada.Text_IO.Put_Line ("PASS: Ada Essential 40 exception translation/cleanup scenarios");
   Ada.Text_IO.Put_Line ("PASS: five Ada rejection preserves armed checkpoint; six consumes; later valid call succeeds");
   Ada.Text_IO.Put_Line ("PASS: Ada no-E and E-found/no-pose semantics after real native calls (test-only controls)");
   declare
      pragma Suppress (Validity_Check);
      procedure Inject (X, Y, Z, W : Interfaces.C.double)
        with Import, Convention => C, External_Name => "opencv_calib3d_test_triangulation_h";
      procedure Unknown
        with Import, Convention => C, External_Name => "opencv_calib3d_test_triangulation_unknown";
      function Live return Interfaces.C.int
        with Import, Convention => C, External_Name => "opencv_calib3d_test_triangulation_live";
      function Nonfinite (Kind : Interfaces.C.int) return Interfaces.C.double
        with Import, Convention => C, External_Name => "calib3d_test_nonfinite";
      Pose : constant Relative_Camera_Pose :=
        ([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [-1.0, 0.0, 0.0]);
      First : constant Normalized_Image_Point_Array := [[0.4, 0.2], [0.4, 0.2]];
      Second : constant Normalized_Image_Point_Array := [[0.2, 0.2], [0.2, 0.2]];
      procedure Outcome (Expected : Triangulation_Status) is
         Values : constant Triangulated_Point_Array := Triangulate_Normalized (First, Second, Pose);
      begin
         Check (Values'Length = 2 and then Values (1).Status = Expected and then
           Values (2).Status = Usable, "injected point status/mixed-batch Ada mapping");
         Check (Live = 0, "Ada result guard leaked injected batch");
      end Outcome;
   begin
      for Stage in Interfaces.C.int range 37 .. 41 loop
         for Kind in Interfaces.C.int range 1 .. 5 loop
            Fail (Stage, Kind);
            begin
               declare
                  Values : constant Triangulated_Point_Array := Triangulate_Normalized (First, Second, Pose);
                  pragma Unreferenced (Values);
               begin
                  raise Program_Error with "triangulation fault not translated";
               end;
            exception
               when OpenCV.OpenCV_Error => null;
            end;
            Check (Live = 0, "triangulation exception leaked handle");
         end loop;
      end loop;
      Inject (2.0, 1.0, 5.0, 0.0); Outcome (At_Infinity);
      Inject (0.4, 0.2, 1.0, 1.0E-300); Outcome (Usable);
      Inject (2.0, 1.0, -5.0, 1.0); Outcome (Non_Positive_Depth);
      Inject (100.0, 1.0, 1.0E-307, 1.0); Outcome (Undefined_Reprojection);
      for Axis in 0 .. 3 loop
         case Axis is
            when 0 => Inject (Nonfinite (0), 1.0, 5.0, 1.0);
            when 1 => Inject (2.0, Nonfinite (1), 5.0, 1.0);
            when 2 => Inject (2.0, 1.0, Nonfinite (2), 1.0);
            when 3 => Inject (2.0, 1.0, 5.0, Nonfinite (0));
         end case;
         Outcome (Unrepresentable_Point);
      end loop;
      Unknown;
      begin
         declare
            Values : constant Triangulated_Point_Array := Triangulate_Normalized (First, Second, Pose);
            pragma Unreferenced (Values);
         begin
            raise Program_Error with "unknown status not rejected";
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
      Check (Live = 0, "unknown-status rejection leaked handle");
      Ada.Text_IO.Put_Line ("PASS: Ada triangulation 25 faults, all five status mappings, mixed batches, unknown-status rejection; live handles=0");
   end;
end Run_Refinement_Faults;
