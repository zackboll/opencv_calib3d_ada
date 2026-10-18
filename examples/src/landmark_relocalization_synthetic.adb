with Ada.Text_IO; use Ada.Text_IO;
with Ada.Numerics.Generic_Elementary_Functions;
with OpenCV.Calib3D; use OpenCV.Calib3D;

procedure Landmark_Relocalization_Synthetic is
   use type OpenCV.Float64_Value;
   package Math is new Ada.Numerics.Generic_Elementary_Functions (OpenCV.Float64_Value);
   K : constant Camera_Intrinsics := (800.0, 820.0, 320.0, 240.0);
   D : constant Distortion_Coefficients := (0.02, -0.003, 0.001, -0.002, 0.0004);
   --  Two known cameras could form a map in this common Cartesian frame.
   Map_First : constant World_To_Camera_Pose := ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
   Map_Second : constant World_To_Camera_Pose := ([0.0, 0.0, -0.1], [-2.0, 0.0, 0.0]);
   --  Third camera's independent truth: Rz(.3), t=(.2,-.1,1).
   C : constant OpenCV.Float64_Value := Math.Cos (0.3);
   S : constant OpenCV.Float64_Value := Math.Sin (0.3);
   Truth_R : constant Rotation_Matrix := [[C, -S, 0.0], [S, C, 0.0], [0.0, 0.0, 1.0]];
   Truth_T : constant Translation_Vector := [0.2, -0.1, 1.0];
   Truth_Center : constant Object_Point := [-C * 0.2 + S * 0.1, S * 0.2 + C * 0.1, -1.0];
   World : Object_Point_Array (1 .. 36);
   Pixels : Image_Point_Array (1 .. 36);

   --  Scalar analytic forward projection, no production pose transform,
   --  Rodrigues or Project_Points used to manufacture expected observations.
   function Observe (P : Object_Point) return Image_Point is
      X : constant OpenCV.Float64_Value := (C * P (0) - S * P (1) + 0.2) / (P (2) + 1.0);
      Y : constant OpenCV.Float64_Value := (S * P (0) + C * P (1) - 0.1) / (P (2) + 1.0);
      R2 : constant OpenCV.Float64_Value := X * X + Y * Y;
      Radial : constant OpenCV.Float64_Value := 1.0 + D.K1 * R2 + D.K2 * R2 ** 2 + D.K3 * R2 ** 3;
   begin
      return [K.Focal_X * (X * Radial + 2.0 * D.P1 * X * Y + D.P2 * (R2 + 2.0 * X * X)) + K.Center_X,
              K.Focal_Y * (Y * Radial + D.P1 * (R2 + 2.0 * Y * Y) + 2.0 * D.P2 * X * Y) + K.Center_Y];
   end Observe;
begin
   Put_Line ("SYNTHETIC observations, not physical sensor data or real navigation accuracy.");
   Put_Line ("Map cameras share a caller Cartesian frame; map baseline: " &
             Measure_Camera_Motion (Map_First, Map_Second).Baseline_Distance'Image);
   Put_Line ("Known landmarks are supplied directly here; no map reconstruction accuracy is claimed.");
   for I in World'Range loop
      World (I) := [OpenCV.Float64_Value (I mod 6) * 0.5 - 1.3,
                    OpenCV.Float64_Value (I / 6) * 0.4 - 0.9,
                    5.0 + OpenCV.Float64_Value (I mod 7) * 0.23];
      Pixels (I) := Observe (World (I));
      --  Only fitting observations 1..30 receive deterministic gross mismatches.
      if I <= 30 and then I mod 7 = 0 then
         Pixels (I) := [2000.0, -1500.0];
      end if;
   end loop;
   declare
      Result : constant Camera_Relocalization_Result := Relocalize_Camera
        (World (1 .. 30), Pixels (1 .. 30), World (31 .. 36), Pixels (31 .. 36),
         K, D, (1000, 1.0, 0.999), (12, 0.01, 0.01, 0.02, 0.01));
      Rotation_Error, Translation_Error, Center_Error : OpenCV.Float64_Value := 0.0;
      Trace : OpenCV.Float64_Value := 0.0;
   begin
      Put_Line ("Candidate correspondence count: 30");
      Put_Line ("Native consensus inlier count: " & Inlier_Count (Result)'Image);
      Put_Line ("Acceptance disposition: " & Disposition (Result)'Image);
      if not Accepted (Result) then
         raise Program_Error with "synthetic relocalization not accepted";
      end if;
      Put_Line ("Held-out RMS before: " & Initial_Holdout (Result).RMS_Error_Pixels'Image);
      Put_Line ("Held-out RMS after: " & Refined_Holdout (Result).RMS_Error_Pixels'Image);
      Put_Line ("Final native-consensus-inlier RMS: " & Final_Inlier_Summary (Result).RMS_Error_Pixels'Image);
      declare
         Pose : constant World_To_Camera_Pose := Accepted_Pose (Result);
         R : constant Rotation_Matrix := Rotation_Matrix_Of (Pose);
         Center : constant Object_Point := Camera_Center (Result);
      begin
         for Row in 0 .. 2 loop
            for Column in 0 .. 2 loop
               Trace := Trace + R (Row, Column) * Truth_R (Row, Column);
            end loop;
            Translation_Error := Translation_Error + (Pose.Translation (Row) - Truth_T (Row)) ** 2;
            Center_Error := Center_Error + (Center (Row) - Truth_Center (Row)) ** 2;
         end loop;
      end;
      Rotation_Error := Math.Arccos (OpenCV.Float64_Value'Max
        (-1.0, OpenCV.Float64_Value'Min (1.0, (Trace - 1.0) / 2.0)));
      Translation_Error := Math.Sqrt (Translation_Error);
      Center_Error := Math.Sqrt (Center_Error);
      Put_Line ("Ground-truth rotation difference (radians): " & Rotation_Error'Image);
      Put_Line ("Ground-truth translation difference: " & Translation_Error'Image);
      Put_Line ("Camera-center error in caller world units: " & Center_Error'Image);
      for Position of Inliers (Result) loop
         if Position mod 7 = 0 then
            raise Program_Error with "gross mismatch entered native consensus";
         end if;
      end loop;
      if Inlier_Count (Result) < 12 or else Rotation_Error > 1.0E-6 or else
        Translation_Error > 1.0E-6 or else Center_Error > 1.0E-6
      then
         raise Program_Error with "independent analytic geometric assertion failed";
      end if;
   end;
end Landmark_Relocalization_Synthetic;