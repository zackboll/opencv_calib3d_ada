with Ada.Text_IO; use Ada.Text_IO;
with OpenCV.Calib3D; use OpenCV.Calib3D;

procedure Metric_Refinement_Synthetic is
   use type OpenCV.Float64_Value;
   A : constant World_To_Camera_Pose := ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
   B : constant World_To_Camera_Pose := ([0.0, 0.0, 0.0], [-2.0, 0.0, 0.0]);
   K1 : constant Camera_Intrinsics := (800.0, 900.0, 320.0, 240.0);
   K2 : constant Camera_Intrinsics := (1200.0, 700.0, 300.0, 200.0);
   --  Independent pinhole oracle: truth=(4,1,8), normalized observations
   --  (0.5,0.125), (0.25,0.125). Noise is deliberately inconsistent in y.
   P : constant Image_Point_Array := [1 => [720.0, 353.4]];
   Q : constant Image_Point_Array := [1 => [600.0, 286.8]];
   R : constant Metric_Point_Refinement_Result_Array := Refine_Metric_World_Points
     (P, Q, K1, K2, No_Distortion, No_Distortion, A, B,
      (0.01, 0.01, 5.0, 5.0), (20, 0.0), (5.0, 5.0));
   Initial : constant Metric_Triangulated_Point := Original_Reconstruction (R (1));
   Final : constant Metric_Triangulated_Point := Refined_Diagnostics (R (1));
   Optimization : constant Optional_Point_Refinement := Normalized_Refinement (R (1));
   Truth : constant Object_Point := [4.0, 1.0, 8.0];
   Error : OpenCV.Float64_Value := 0.0;
begin
   Put_Line ("SYNTHETIC observations, not actual sensor data. Known world point=(4,1,8).");
   Put_Line ("Fixed poses, baseline=2 caller length units, fixed calibration.");
   Put_Line ("Acceptance: " & Disposition (R (1))'Image);
   if not Accepted (R (1)) or else not Optimization.Available or else
     Optimization.Value.Outcome /= Improved or else
     Optimization.Value.Final_Normalized_RMS >= Optimization.Value.Initial_Normalized_RMS
   then
      raise Program_Error with "synthetic refinement acceptance/objective assertion";
   end if;
   Put_Line ("Initial normalized RMS: " & Optimization.Value.Initial_Normalized_RMS'Image);
   Put_Line ("Final normalized RMS: " & Optimization.Value.Final_Normalized_RMS'Image);
   Put_Line ("Initial first/second pixel errors: " & Initial.Pixels.First_Error_Pixels'Image &
             Initial.Pixels.Second_Error_Pixels'Image);
   Put_Line ("Final first/second pixel errors: " & Final.Pixels.First_Error_Pixels'Image &
             Final.Pixels.Second_Error_Pixels'Image);
   for Axis in 0 .. 2 loop
      Put_Line ("Initial world component: " & Initial.Geometry.World_Position (Axis)'Image &
                "; refined unit: " & Optimization.Value.Point.Position_In_First_Camera (Axis)'Image &
                "; refined world: " & Accepted_World_Position (R (1)) (Axis)'Image);
      Error := OpenCV.Float64_Value'Max
        (Error, abs (Accepted_World_Position (R (1)) (Axis) - Truth (Axis)));
   end loop;
   Put_Line ("Maximum known-world-point component error: " & Error'Image);
   --  Synthetic noise, NOT a universal world accuracy bound.
   if Error > 0.02 then
      raise Program_Error with "independent synthetic geometry assertion";
   end if;
end Metric_Refinement_Synthetic;
