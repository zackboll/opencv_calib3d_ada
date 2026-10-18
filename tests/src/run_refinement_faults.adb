with Ada.Text_IO;
with Ada.Numerics.Generic_Elementary_Functions;
with Interfaces.C;
with OpenCV.Calib3D;

procedure Run_Refinement_Faults is
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   use type Interfaces.C.int;
   use type Interfaces.C.double;
   use type Object_Point;
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
      A : constant World_To_Camera_Pose := ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
      B : World_To_Camera_Pose := ([0.0, 0.0, 0.0], [-2.0, 0.0, 0.0]);
      Identity : constant Camera_Intrinsics := (1.0, 1.0, 0.0, 0.0);
      Saved : Metric_Point_Refinement_Result_Array (1 .. 2);
      type Stages is array (Positive range <>) of Interfaces.C.int;
      procedure Inject (X, Y, Z, W : Interfaces.C.double)
        with Import, Convention => C, External_Name => "opencv_calib3d_test_triangulation_h";
      Tiny : Interfaces.C.double := 1.0E-300;
      function Reconstruct
        (First : Image_Point := [0.5, 0.126]; Second : Image_Point := [0.25, 0.124])
        return Metric_Point_Refinement_Result_Array is
        (Refine_Metric_World_Points
          ([First, [0.5, 0.126]], [Second, [0.25, 0.124]], Identity, Identity,
           No_Distortion, No_Distortion, A, B,
           (0.0, OpenCV.Float64_Value'Last, OpenCV.Float64_Value'Last, OpenCV.Float64_Value'Last),
           (20, 0.0), (1.0, 1.0)));
   begin
      for Stage of Stages'[1, 2, 13, 14, 15, 16, 37, 38, 39, 40, 41] loop
         for Kind in Interfaces.C.int range 1 .. 5 loop
            Fail (Stage, Kind);
            begin
               Saved := Reconstruct;
               raise Program_Error with "metric refinement swallowed native failure";
            exception
               when OpenCV.OpenCV_Error => null;
            end;
            Check (Disposition (Saved (1)) = Initial_No_Baseline and then
                   Disposition (Saved (2)) = Initial_No_Baseline,
                   "metric refinement native failure partially published batch");
         end loop;
      end loop;
      --  SYNTHETIC controls: native triangulation runs, then its first seed is
      --  replaced. These are not assertions about real SVD output rounding.
      Inject (0.0, 0.0, 2.0, 1.0);
      Saved := Reconstruct ([0.0, 0.0], [-0.5, 0.0]);
      Check (Disposition (Saved (1)) = Refinement_No_Improving_Step and then
             Normalized_Refinement (Saved (1)).Value.Accepted_Steps = 0 and then
             Original_Reconstruction (Saved (1)).Geometry.World_Position (2) = 4.0,
             "synthetic exact zero objective did not preserve original metric point");
      B.Translation := [-5.0E307, 0.0, 0.0];
      Inject (1.0, 0.0, 1.0, 1.0);
      Saved := Reconstruct;
      Check (Original_Reconstruction (Saved (1)).Geometry.Available and then
             Normalized_Refinement (Saved (1)).Value.Outcome = Improved and then
             Disposition (Saved (1)) = Refined_Conversion_Unavailable and then
             not Accepted (Saved (1)), "synthetic improved metric overflow not contained");
      B.Translation := [-1.0E-300, 0.0, 0.0];
      Inject (0.0, 0.0, 2.0, 1.0);
      Saved := Reconstruct ([1.0E-30, 0.0], [-0.5, 0.0]);
      Check (Original_Reconstruction (Saved (1)).Geometry.Available and then
             Normalized_Refinement (Saved (1)).Value.Outcome = Improved and then
             Disposition (Saved (1)) = Refined_Conversion_Unavailable,
             "synthetic improved nonzero component underflow not contained");
      Tiny := Tiny / 1.0E10;
      Inject (0.0, 0.0, Tiny, 1.0);
      B.Translation := [0.0, 0.0, 1.0E300];
      Saved := Reconstruct;
      Check (Original_Reconstruction (Saved (1)).Geometry.Available and then
             Original_Reconstruction (Saved (1)).Pixels.Available and then
             Disposition (Saved (1)) = Refinement_Numerically_Unavailable and then
             not Refined_Diagnostics (Saved (1)).Geometry.Available,
             "synthetic optimizer numerical failure did not preserve original metric reconstruction");
      B.Translation := [-2.0, 0.0, 0.0];
      Inject (0.0, 0.0, 1.0, 0.0);
      Saved := Reconstruct;
      Check (Disposition (Saved (1)) = Initial_Source_Unusable and then Accepted (Saved (2)),
             "synthetic mixed unavailable/success batch corrupted another point");
      Ada.Text_IO.Put_Line ("PASS: metric refinement 55 native exceptions/atomicity; synthetic zero objective, refined overflow/underflow, mixed batch");
   end;
   declare
      A : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, 0.0], Translation => [0.0, 0.0, 0.0]);
      B : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, 0.0], Translation => [-2.0, 0.0, 0.0]);
      Identity : constant Camera_Intrinsics := (1.0, 1.0, 0.0, 0.0);
      Policy : constant Metric_Triangulation_Options := (0.0, 1.0, 1.0, 1.0);
      Saved : Metric_Triangulated_Point_Array (1 .. 1);
      type Stages is array (Positive range <>) of Interfaces.C.int;
   begin
      for Stage of Stages'[1, 2, 13, 14, 15, 16, 37, 38, 39, 40, 41] loop
         for Kind in Interfaces.C.int range 1 .. 5 loop
            Fail (Stage, Kind);
            begin
               Saved := Triangulate_World_Points
                 ([1 => [0.5, 0.125]], [1 => [0.25, 0.125]], Identity, Identity,
                  No_Distortion, No_Distortion, A, B, Policy);
               raise Program_Error with "metric native failure silently reclassified";
            exception
               when OpenCV.OpenCV_Error => null;
            end;
            Check (Saved (1).Disposition = No_Physical_Baseline and then
                   not Saved (1).Source.Available, "metric failure partially published");
         end loop;
      end loop;
      --  An armed triangulation checkpoint must survive exact-zero baseline.
      Fail (37, 3);
      Saved := Triangulate_World_Points
        ([1 => [0.5, 0.125]], [1 => [0.25, 0.125]], Identity, Identity,
         No_Distortion, No_Distortion, A, A, Policy);
      Check (Saved (1).Disposition = No_Physical_Baseline, "zero baseline entered triangulation");
      begin
         Saved := Triangulate_World_Points
           ([1 => [0.5, 0.125]], [1 => [0.25, 0.125]], Identity, Identity,
            No_Distortion, No_Distortion, A, B, Policy);
         raise Program_Error with "zero baseline consumed armed native checkpoint";
      exception
         when OpenCV.OpenCV_Error => null;
      end;
      Ada.Text_IO.Put_Line ("PASS: metric orchestration 55 native exceptions, propagation and atomicity; zero baseline preserves checkpoint");
   end;

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
      declare
         A : World_To_Camera_Pose :=
           (Rotation => [0.0, 0.0, 0.0], Translation => [0.0, 0.0, 0.0]);
         B : World_To_Camera_Pose :=
           (Rotation => [0.0, 0.0, 0.0], Translation => [-1.0, 0.0, 0.0]);
         Calibration : Camera_Intrinsics := (1.0, 1.0, 0.0, 0.0);
         Pixels : Image_Point_Array := [1 => [0.4, 0.2]];
         function Reconstruct return Metric_Triangulated_Point is
            V : constant Metric_Triangulated_Point_Array := Triangulate_World_Points
              (Pixels, Pixels, Calibration, Calibration, No_Distortion, No_Distortion,
               A, B, (0.0, OpenCV.Float64_Value'Last,
                      OpenCV.Float64_Value'Last, OpenCV.Float64_Value'Last));
         begin
            Check (Live = 0, "metric control leaked source handle");
            return V (1);
         end Reconstruct;
         procedure Unavailable is
            V : constant Metric_Triangulated_Point := Reconstruct;
         begin
            Check (V.Disposition = Metric_Conversion_Unavailable and then
                   V.Source.Point.Status = Usable and then not V.Geometry.Available,
                   "metric numerical control did not preserve Usable/unavailable");
         end Unavailable;
      begin
         Inject (100.0, 1.0, 1.0E-307, 1.0);
         declare
            V : constant Metric_Triangulated_Point := Reconstruct;
         begin
            Check (V.Disposition = Source_Undefined_Reprojection and then
                   V.Source.Point.Status = Undefined_Reprojection and then not V.Geometry.Available,
                   "metric undefined source mapping");
         end;
         Inject (Nonfinite (0), 1.0, 5.0, 1.0);
         declare
            V : constant Metric_Triangulated_Point := Reconstruct;
         begin
            Check (V.Disposition = Source_Unrepresentable_Point and then
                   V.Source.Point.Status = Unrepresentable_Point and then not V.Geometry.Available,
                   "metric unrepresentable source mapping");
         end;
         --  Actual triangulation executes before explicitly test-only replacements.
         B.Translation := [-1.0E-300, 0.0, 0.0];
         Inject (0.0, 0.0, 1.0E-100, 1.0);
         Unavailable; --  positive unit depth scales to exact zero
         Inject (1.0, 0.0, 1.0, 1.0E-300);
         declare
            V : constant Metric_Triangulated_Point := Reconstruct;
         begin
            Check (V.Geometry.Available and then
                   abs (V.Geometry.First_Depth - 1.0) < 1.0E-12,
                   "large finite unit coordinates safely scaled");
         end;
         A.Translation := [-1.0E308, 0.0, 0.0];
         B.Translation := [-1.0E308, -1.0, 0.0];
         Inject (1.0E308, 0.0, 1.0, 1.0);
         Unavailable; --  X_first - t1 overflows before world rotation
         A.Translation := [0.0, 0.0, 0.0];
         B.Translation := [-1.0, 0.0, 0.0];
         Calibration.Focal_X := 1.0E308;
         Pixels (1) := [-1.0E308, 0.0];
         Inject (1.0, 0.0, 1.0, 1.0);
         declare
            V : constant Metric_Triangulated_Point := Reconstruct;
         begin
            Check (V.Disposition = Metric_Conversion_Unavailable and then
                   V.Geometry.Available and then not V.Pixels.Available,
                   "unrepresentable pixel subtraction not a valid diagnostic");
         end;
         Ada.Text_IO.Put_Line ("PASS: metric source mappings, depth underflow, huge unit point, world overflow, pixel residual overflow controls");
      end;

   end;
   declare
      Candidate : constant Planar_Motion_Hypothesis :=
        ([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
         [0.1, 0.0, 0.0], [0.0, 0.0, 1.0], False);
   begin
      Fail (46, 3);
      declare
         Pure : constant Planar_Motion_Hypothesis :=
           (Candidate.Rotation_First_To_Second, [others => 0.0], [others => 0.0], True);
         Values : constant Planar_Hypothesis_Visibility_Array :=
           Assess_Planar_Visibility ([Pure], [[0.2, 0.1]], [[0.25, 0.1]]);
         None : constant Planar_Hypothesis_Visibility_Array :=
           Assess_Planar_Visibility ([Candidate], [[0.2, 0.1]], [[0.25, 0.1]], []);
      begin
         Check (Values (1).Disposition = Not_Applicable_Pure_Rotation and then
           None (1).Disposition = No_Selected_References, "visibility special cases");
      end;
      begin
         declare
            Values : constant Planar_Hypothesis_Visibility_Array :=
              Assess_Planar_Visibility ([Candidate], [[0.2, 0.1]], [[0.25, 0.1]]);
            pragma Unreferenced (Values);
         begin
            raise Program_Error with "special cases consumed armed checkpoint";
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
      Ada.Text_IO.Put_Line ("PASS: visibility pure/no-reference cases preserve armed native checkpoint");
      for Stage in Interfaces.C.int range 46 .. 49 loop
         for Kind in Interfaces.C.int range 1 .. 5 loop
            Fail (Stage, Kind);
            begin
               declare
                  Values : constant Planar_Hypothesis_Visibility_Array :=
                    Assess_Planar_Visibility ([Candidate], [[0.2, 0.1]], [[0.25, 0.1]]);
                  pragma Unreferenced (Values);
               begin
                  raise Program_Error with "visibility fault not translated";
               end;
            exception
               when OpenCV.OpenCV_Error => null;
            end;
         end loop;
      end loop;
      Ada.Text_IO.Put_Line ("PASS: Ada visibility 20 exception translation scenarios");
   end;
    declare
       Calibration : constant Camera_Intrinsics := (800.0, 820.0, 640.0, 360.0);
       World : constant Object_Point_Array := [[0.3, -0.2, 3.5], [1.4, 0.1, 4.0], [-0.5, 1.2, 5.3]];
       Pixels : constant Image_Point_Array := [[700.0, 320.0], [910.0, 390.0], [520.0, 560.0]];
       procedure Control (Mode : Interfaces.C.int)
         with Import, Convention => C, External_Name => "opencv_calib3d_test_ap3p_control";
    begin
       Fail (50, 3);
       begin
          declare
             Found : constant AP3P_Pose_Hypothesis_Array :=
               Solve_AP3P_Hypotheses (World (1 .. 2), Pixels (1 .. 2), Calibration);
             pragma Unreferenced (Found);
          begin
             raise Program_Error with "two correspondences reached AP3P";
          end;
       exception
          when OpenCV.OpenCV_Error => null;
       end;
       begin
          declare
             Found : constant AP3P_Pose_Hypothesis_Array := Solve_AP3P_Hypotheses (World, Pixels, Calibration);
             pragma Unreferenced (Found);
          begin
             raise Program_Error with "valid AP3P did not consume armed checkpoint";
          end;
       exception
          when OpenCV.OpenCV_Error => null;
       end;
       for Stage in Interfaces.C.int range 50 .. 53 loop
          for Kind in Interfaces.C.int range 1 .. 5 loop
             Fail (Stage, Kind);
             begin
                declare
                   Found : constant AP3P_Pose_Hypothesis_Array :=
                     Solve_AP3P_Hypotheses (World, Pixels, Calibration);
                   pragma Unreferenced (Found);
                begin
                   raise Program_Error with "AP3P fault not translated";
                end;
             exception
                when OpenCV.OpenCV_Error => null;
             end;
          end loop;
       end loop;
       Control (1);
       declare
          Found : constant AP3P_Pose_Hypothesis_Array := Solve_AP3P_Hypotheses (World, Pixels, Calibration);
       begin
          Check (Found'First = 1 and then Found'Length = 0, "test-only post-native AP3P zero");
       end;
       for Mode in Interfaces.C.int range 2 .. 5 loop
          Control (Mode);
          begin
             declare
                Found : constant AP3P_Pose_Hypothesis_Array := Solve_AP3P_Hypotheses (World, Pixels, Calibration);
                pragma Unreferenced (Found);
             begin
                raise Program_Error with "malformed AP3P native result accepted";
             end;
          exception
             when OpenCV.OpenCV_Error => null;
          end;
       end loop;
       Ada.Text_IO.Put_Line ("PASS: Ada AP3P 20 exception translations and post-native controls");
       for Stage in Interfaces.C.int range 1 .. 2 loop
          for Kind in Interfaces.C.int range 1 .. 5 loop
             Fail (Stage, Kind);
             begin
                declare
                   Values : constant AP3P_Reference_Result_Matrix :=
                     Evaluate_AP3P_References
                       ([(Pose => (Rotation => [others => 0.0],
                                   Translation => [others => 0.0]))],
                        [[0.0, 0.0, 5.0]], [[10.0, 20.0]], Calibration);
                   pragma Unreferenced (Values);
                begin
                   raise Program_Error with "projection fault became an AP3P disposition";
                end;
             exception
                when OpenCV.OpenCV_Error => null;
             end;
          end loop;
       end loop;
       declare
          Quiet : constant AP3P_Reference_Result_Matrix :=
            Evaluate_AP3P_References
              ([(Pose => (Rotation => [others => 0.0], Translation => [others => 0.0]))],
               [[0.0, 0.0, 0.0]], [[10.0, 20.0]], Calibration);
       begin
          Check (Quiet (1, 1).Disposition = Non_Positive_Depth,
            "nonpositive depth must not enter native projection");
       end;
       Ada.Text_IO.Put_Line ("PASS: AP3P reference diagnostics propagate 10 native projection faults");
       declare
          Pose : constant World_To_Camera_Pose :=
            (Rotation => [others => 0.0], Translation => [0.0, 0.0, 1.0]);
          Seeds : constant Object_Point_Array :=
            [[0.1, 0.0, 2.0], [0.0, 0.2, 2.5], [-0.2, 0.1, 3.0]];
          Seed_Pixels : constant Image_Point_Array :=
            Project_Points (Seeds, Calibration, No_Distortion, Pose);
          Policy : constant AP3P_Selection_Options := (1.0, 1.0, 0.0);
       begin
          for Stage in Interfaces.C.int range 1 .. 2 loop
             Fail (Stage, 1);
             begin
                declare
                   Chosen : constant AP3P_Pose_Selection :=
                     Select_AP3P_Pose
                       (Seeds, Seed_Pixels, [[0.0, 0.0, 4.0]], [[20.0, 30.0]],
                        Calibration, No_Distortion, Policy);
                begin
                   raise Program_Error with
                     "projection fault returned an AP3P selection "
                     & AP3P_Selection_Disposition'Image (Disposition (Chosen));
                end;
             exception
                when OpenCV.OpenCV_Error => null;
             end;
          end loop;
       end;
       Ada.Text_IO.Put_Line ("PASS: AP3P selection propagates native projection faults without a result");
       declare
          Pose : constant World_To_Camera_Pose :=
            (Rotation => [others => 0.0], Translation => [0.0, 0.0, 1.0]);
          Seeds : constant Object_Point_Array :=
            [[0.2, -0.1, 3.0], [0.8, 0.2, 3.6], [-0.3, 0.5, 4.2]];
          Selection_Points : constant Object_Point_Array := [[0.4, -0.4, 5.0]];
          Holdout_Points : constant Object_Point_Array := [[-0.5, 0.3, 6.0]];
          Seed_Pixels : constant Image_Point_Array :=
            Project_Points (Seeds, Calibration, No_Distortion, Pose);
          Selection_Pixels : constant Image_Point_Array :=
            Project_Points (Selection_Points, Calibration, No_Distortion, Pose);
          Holdout_Pixels : constant Image_Point_Array :=
            Project_Points (Holdout_Points, Calibration, No_Distortion, Pose);
          Select_Policy : constant AP3P_Selection_Options := (1.0E-3, 1.0E-3, 0.0);
          Accept_Policy : constant AP3P_Refinement_Acceptance_Options := (1.0, 2.0, 0.5);
       begin
          Return_False;
          declare
             Synthetic : constant AP3P_Refined_Pose :=
               Select_Refine_AP3P_Pose
                 (Seeds, Seed_Pixels, Selection_Points, Selection_Pixels,
                  Holdout_Points, Holdout_Pixels, Calibration, No_Distortion,
                  Select_Policy, Accept_Policy);
          begin
             Check (Refinement_Disposition (Synthetic) = Native_Refinement_False
               and then Refinement_Attempted (Synthetic)
               and then not Native_Refinement_Succeeded (Synthetic)
               and then not Accepted (Synthetic)
               and then Initial_Holdout_Valid (Synthetic)
               and then not Refined_Holdout_Valid (Synthetic),
               "synthetic native false must preserve the initial pose only");
             Check (Initial_Pose (Synthetic).Translation (2) > 0.0,
               "synthetic false still exposes the owned initial pose");
             begin
                declare
                   Hidden : constant World_To_Camera_Pose := Accepted_Pose (Synthetic);
                   pragma Unreferenced (Hidden);
                begin
                   raise Program_Error with "synthetic false exposed an accepted pose";
                end;
             exception
                when OpenCV.OpenCV_Error => null;
             end;
          end;
          --  Checkpoint 1 is the held-out projection before refinement.
          --  Checkpoint 10 is the iterative refinement call. Neither publishes.
          for Stage in Interfaces.C.int'(1) .. 1 loop
             Fail (Stage, 1);
             begin
                declare
                   Value : constant AP3P_Refined_Pose :=
                     Select_Refine_AP3P_Pose
                       (Seeds, Seed_Pixels, Selection_Points, Selection_Pixels,
                        Holdout_Points, Holdout_Pixels, Calibration, No_Distortion,
                        Select_Policy, Accept_Policy);
                begin
                   raise Program_Error with
                     "projection fault returned a refinement result "
                     & AP3P_Refinement_Disposition'Image (Refinement_Disposition (Value));
                end;
             exception
                when OpenCV.OpenCV_Error => null;
             end;
          end loop;
          for Stage in Interfaces.C.int range 10 .. 12 loop
             Fail (Stage, 1);
             begin
                declare
                   Value : constant AP3P_Refined_Pose :=
                     Select_Refine_AP3P_Pose
                       (Seeds, Seed_Pixels, Selection_Points, Selection_Pixels,
                        Holdout_Points, Holdout_Pixels, Calibration, No_Distortion,
                        Select_Policy, Accept_Policy);
                begin
                   raise Program_Error with
                     "refinement fault returned a result "
                     & AP3P_Refinement_Disposition'Image (Refinement_Disposition (Value));
                end;
             exception
                when OpenCV.OpenCV_Error => null;
             end;
          end loop;
          declare
             Recovered : constant AP3P_Refined_Pose :=
               Select_Refine_AP3P_Pose
                 (Seeds, Seed_Pixels, Selection_Points, Selection_Pixels,
                  Holdout_Points, Holdout_Pixels, Calibration, No_Distortion,
                  Select_Policy, Accept_Policy);
          begin
             Check (Accepted (Recovered), "refinement recovers after injected faults");
          end;
       end;
       Ada.Text_IO.Put_Line
         ("PASS: synthetic AP3P refinement false and projection/refinement faults publish no partial result");
     end;

    declare
       First  : constant World_To_Camera_Pose :=
         (Rotation => [0.0, 0.0, 1.5], Translation => [2.0, -1.0, 0.5]);
       Second : constant World_To_Camera_Pose :=
         (Rotation => [1.0, 0.0, 0.0], Translation => [1.0, 3.0, -2.0]);
       Expected : constant World_To_Camera_Pose := Compose_Poses (First, Second);
       Count : Natural := 0;
    begin
       for Stage in Interfaces.C.int range 54 .. 57 loop
          for Kind in Interfaces.C.int range 1 .. 5 loop
             Fail (Stage, Kind);
             begin
                declare
                   Value : constant World_To_Camera_Pose := Compose_Poses (First, Second);
                   pragma Unreferenced (Value);
                begin
                   raise Program_Error with "pose composition fault not translated";
                end;
             exception
                when OpenCV.OpenCV_Error => Count := Count + 1;
             end;
             Check (Compose_Poses (First, Second) = Expected,
                    "pose composition did not recover after injected fault");
          end loop;
       end loop;
       Check (Count = 20, "pose composition expected 20 translated faults");
       Ada.Text_IO.Put_Line ("PASS: Ada pose composition 20 exception translations and recovery");
    end;

    declare
       procedure Control (Mode : Interfaces.C.int)
         with Import, Convention => C, External_Name => "opencv_calib3d_test_correction_control";
       F : constant Fundamental_Matrix := [[0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]];
       First : constant Image_Point_Array := [[100.0, 20.0], [300.0, 40.0]];
       Second : constant Image_Point_Array := [[90.0, 24.0], [250.0, 30.0]];
       Saved1 : constant Image_Point_Array := First;
       Saved2 : constant Image_Point_Array := Second;
       Policy : constant Epipolar_Correction_Options := (1.0E-3, 1.0E-9, 1.0E-12, 1.0E-9, 1.0E-9, 1.0E-9);
       Expected : constant Corrected_Epipolar_Matches := Correct_Epipolar_Matches (F, First, Second, Policy);
       Count : Natural := 0;
       procedure Attempt (What : String) is
       begin
          declare
             Value : constant Corrected_Epipolar_Matches := Correct_Epipolar_Matches (F, First, Second, Policy);
             pragma Unreferenced (Value);
          begin
             raise Program_Error with What & " not translated";
          end;
       exception
          when OpenCV.OpenCV_Error => Count := Count + 1;
       end Attempt;
    begin
       for Stage in Interfaces.C.int range 58 .. 61 loop
          for Kind in Interfaces.C.int range 1 .. 5 loop
             Fail (Stage, Kind);
             Attempt ("epipolar correction fault");
             Check (First = Saved1 and then Second = Saved2, "correction fault changed inputs");
             Check (Correct_Epipolar_Matches (F, First, Second, Policy) = Expected,
                    "epipolar correction did not recover after injected fault");
          end loop;
       end loop;
       Check (Count = 20, "epipolar correction expected 20 translated faults");
       --  Post-native output gate after a REAL native call: NaN, residual,
       --  excessive displacement, wrong shape, infinity. All raise; none publish.
       for Mode in Interfaces.C.int range 1 .. 5 loop
          Control (Mode);
          Attempt ("epipolar correction output control");
          Check (Correct_Epipolar_Matches (F, First, Second, Policy) = Expected, "recovery after output control");
       end loop;
       Check (Count = 25, "epipolar correction expected 5 rejected native outputs");
       Ada.Text_IO.Put_Line ("PASS: Ada epipolar correction 20 exception translations + 5 native-output "
                             & "rejections, inputs unchanged, recovery");
    end;

   declare
      H : constant Object_Point_Array := [[0.7, -0.4, 3.0], [-0.8, 0.6, 2.8]];
      HP : constant Image_Point_Array := Project_Points (H, K, No_Distortion, Truth);
      Policy : constant Relocalization_Verification_Options := (4, 1.0, 1.0, 1.0, 1.0);
      Options : constant RANSAC_Options := (1000, 1.0, 0.999);
      --  Count is fixed only for this deliberately exact, clean nine-point
      --  fault fixture. Native false/exception controls are SYNTHETIC.
      Saved : Camera_Relocalization_Result (World'Length) := Relocalize_Camera
        (World, Images, H, HP, K, No_Distortion, Options, Policy);
      Original : constant World_To_Camera_Pose := Initial_Pose (Saved);
      Center : constant Object_Point := Camera_Center (Saved);
      type Stage_Array is array (Positive range <>) of Interfaces.C.int;
      Count : Natural := 0;
   begin
      Check (Accepted (Saved), "relocalization fault fixture accepted");
      Return_False;
      declare
         Value : constant Camera_Relocalization_Result := Relocalize_Camera
           (World, Images, H, HP, K, No_Distortion, Options, Policy);
      begin
         Check (Disposition (Value) = Native_Refinement_False and then
                Found (Value) and then Refinement_Attempted (Value) and then
                not Native_Refinement_Succeeded (Value) and then
                Initial_Pose (Value) = Original and then Initial_Holdout_Valid (Value) and then
                not Refined_Holdout_Valid (Value) and then not Accepted (Value),
                "synthetic native false retains initial and exposes no accepted pose");
         begin
            declare
               Unexpected : constant World_To_Camera_Pose := Accepted_Pose (Value);
               pragma Unreferenced (Unexpected);
            begin
               raise Program_Error with "synthetic false exposed accepted pose";
            end;
         exception when OpenCV.OpenCV_Error => null; end;
      end;
      --  RANSAC allocation/publication, Rodrigues, projection, refinement and
      --  final accepted camera-center stages, all five native exception kinds.
      for Stage of Stage_Array'[1, 2, 3, 4, 5, 6, 7, 10, 11, 12, 15, 16] loop
         for Kind in Interfaces.C.int range 1 .. 5 loop
            Fail (Stage, Kind);
            begin
               Saved := Relocalize_Camera (World, Images, H, HP, K, No_Distortion, Options, Policy);
               raise Program_Error with "relocalization swallowed native exception at" & Stage'Image;
            exception when OpenCV.OpenCV_Error => Count := Count + 1; end;
            Check (Accepted (Saved) and then Initial_Pose (Saved) = Original and then
                   Camera_Center (Saved) = Center, "exception published partial relocalization");
         end loop;
      end loop;
      Check (Count = 60, "relocalization translated all synthetic faults");
      Saved := Relocalize_Camera (World, Images, H, HP, K, No_Distortion, Options, Policy);
      Check (Accepted (Saved), "relocalization recovers after native exceptions");
      --  Too few fitting points must not consume the native RANSAC checkpoint.
      for N in 4 .. 5 loop
         Fail (5, 1);
         declare
            Value : constant Camera_Relocalization_Result := Relocalize_Camera
              (World (1 .. N), Images (1 .. N), H, HP, K, No_Distortion, Options, Policy);
         begin
            Check (Disposition (Value) = Insufficient_Fitting_Landmarks, "4/5 rejected before native solve");
         end;
         begin
            Saved := Relocalize_Camera (World, Images, H, HP, K, No_Distortion, Options, Policy);
            raise Program_Error with "4/5 consumed the RANSAC fault";
         exception when OpenCV.OpenCV_Error => null; end;
      end loop;
      Ada.Text_IO.Put_Line
        ("PASS: relocalization SYNTHETIC false, 60 native exception/atomicity checks, 4/5 no native entry, recovery");
   end;
   declare
      A : constant World_To_Camera_Pose := ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
      B : constant World_To_Camera_Pose := ([0.0, 0.0, 0.2], [-2.0, 0.3, 0.0]);
      Identity : constant Camera_Intrinsics := (1.0, 1.0, 0.0, 0.0);
      Policy : constant Epipolar_Verification_Options := (1, 1.0, 1.0);
      type Stage_Array is array (Positive range <>) of Interfaces.C.int;
      function Verify (N : Natural := 2; Minimum : Positive := 1) return Known_Pose_Epipolar_Result is
        (Verify_Known_Pose_Epipolar
          ([1 .. N => [0.2, 0.1]], [1 .. N => [-0.2, 0.1]], Identity, Identity,
           No_Distortion, No_Distortion, A, B, (Minimum, 1.0, 1.0)));
      Saved : Known_Pose_Epipolar_Result (2) := Verify;
      Original : constant Essential_Matrix := Geometry (Saved).Matrix;
      Count : Natural := 0;
   begin
      --  SYNTHETIC exception controls, not naturally observed native failures.
      --  Sampson has NO exception checkpoint; none is added by this task.
      for Stage of Stage_Array'[3, 4, 13, 14, 15, 16, 54, 55, 56, 57] loop
         for Kind in Interfaces.C.int range 1 .. 5 loop
            Fail (Stage, Kind);
            begin
               Saved := Verify;
               raise Program_Error with "epipolar swallowed synthetic native exception at" & Stage'Image;
            exception when OpenCV.OpenCV_Error => Count := Count + 1; end;
            Check (Accepted (Saved) and then Geometry (Saved).Matrix = Original,
                   "failed assignment partially published epipolar result");
            Saved := Verify;
            Check (Accepted (Saved), "epipolar healthy call did not recover");
         end loop;
      end loop;
      --  Armed first native Rodrigues checkpoint survives both early returns.
      for N in 0 .. 1 loop
         Fail (15, 3);
         declare V : constant Known_Pose_Epipolar_Result := Verify (N, 2);
         begin
            Check (Disposition (V) = Insufficient_Correspondences and then not Geometry_Evaluated (V),
                   "insufficient epipolar request entered native evaluation");
         end;
         begin
            Saved := Verify;
            raise Program_Error with "insufficient request consumed armed native fault";
         exception when OpenCV.OpenCV_Error => null; end;
      end loop;
      --  Zero baseline does not enter undistortion or Sampson evaluation.
      Fail (13, 3);
      declare V : constant Known_Pose_Epipolar_Result := Verify_Known_Pose_Epipolar
        ([[0.0, 0.0]], [[0.0, 0.0]], Identity, Identity, No_Distortion, No_Distortion, A, A, Policy);
      begin
         Check (Disposition (V) = No_Physical_Baseline, "zero baseline not guarded");
      end;
      begin
         Saved := Verify;
         raise Program_Error with "zero baseline consumed undistortion checkpoint";
      exception when OpenCV.OpenCV_Error => null; end;
      Saved := Verify;
      Check (Count = 50 and then Accepted (Saved), "epipolar fault count and recovery");
      Ada.Text_IO.Put_Line
        ("PASS: known-pose epipolar 50 SYNTHETIC native exceptions, assignment atomicity, per-fault recovery, empty/insufficient no native entry, zero-baseline no undistortion; Sampson has no fault checkpoint");
   end;
end Run_Refinement_Faults;
