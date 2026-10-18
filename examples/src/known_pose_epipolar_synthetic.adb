with Ada.Text_IO; use Ada.Text_IO;
with Ada.Numerics.Generic_Elementary_Functions;
with OpenCV.Calib3D; use OpenCV.Calib3D;

procedure Known_Pose_Epipolar_Synthetic is
   use type OpenCV.Float64_Value;
   package Math is new Ada.Numerics.Generic_Elementary_Functions (OpenCV.Float64_Value);
   K1 : constant Camera_Intrinsics := (800.0, 820.0, 320.0, 240.0);
   K2 : constant Camera_Intrinsics := (630.0, 710.0, 280.0, 190.0);
   D1 : constant Distortion_Coefficients := (0.02, -0.003, 0.001, -0.002, 0.0004);
   D2 : constant Distortion_Coefficients := (-0.03, 0.004, -0.002, 0.001, -0.0005);
   First : constant World_To_Camera_Pose := ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
   Second : constant World_To_Camera_Pose := ([0.0, 0.0, 0.2], [-2.0, 0.3, 0.0]);
   Pure_Rotation : constant World_To_Camera_Pose := ([0.0, 0.0, 0.2], [0.0, 0.0, 0.0]);
   C : constant OpenCV.Float64_Value := Math.Cos (0.2);
   S : constant OpenCV.Float64_Value := Math.Sin (0.2);
   Policy : constant Epipolar_Verification_Options := (6, 1.0E-10, 1.0E-10);
   P, Q, Corrupted : Image_Point_Array (1 .. 24);

   --  Independent scalar analytic projection: no production Rodrigues,
   --  transforms or Project_Points generate these synthetic observations.
   function Observe (W : Object_Point; Is_Second : Boolean) return Image_Point is
      K : constant Camera_Intrinsics := (if Is_Second then K2 else K1);
      D : constant Distortion_Coefficients := (if Is_Second then D2 else D1);
      X : constant OpenCV.Float64_Value :=
        (if Is_Second then (C * W (0) - S * W (1) - 2.0) / W (2) else W (0) / W (2));
      Y : constant OpenCV.Float64_Value :=
        (if Is_Second then (S * W (0) + C * W (1) + 0.3) / W (2) else W (1) / W (2));
      R2 : constant OpenCV.Float64_Value := X * X + Y * Y;
      Radial : constant OpenCV.Float64_Value := 1.0 + D.K1 * R2 + D.K2 * R2 ** 2 + D.K3 * R2 ** 3;
   begin
      return [K.Focal_X * (X * Radial + 2.0 * D.P1 * X * Y + D.P2 * (R2 + 2.0 * X * X)) + K.Center_X,
              K.Focal_Y * (Y * Radial + D.P1 * (R2 + 2.0 * Y * Y) + 2.0 * D.P2 * X * Y) + K.Center_Y];
   end Observe;

   procedure Report (Label : String; Result : Known_Pose_Epipolar_Result) is
      G : constant Known_Pose_Essential_Result := Geometry (Result);
      Defined_Count : Natural := 0;
   begin
      Put_Line (Label);
      Put_Line ("Correspondence count: " & Result.Count'Image);
      Put_Line ("Physical baseline (caller world units): " & G.Baseline_Distance'Image);
      if G.Defined then
         Put_Line ("Relative first-to-second rotation:");
         for I in 0 .. 2 loop
            Put_Line (G.Rotation_First_To_Second (I, 0)'Image & " " &
                      G.Rotation_First_To_Second (I, 1)'Image & " " & G.Rotation_First_To_Second (I, 2)'Image);
         end loop;
         Put_Line ("Relative translation direction in SECOND camera frame:");
         Put_Line (G.Translation_Direction (0)'Image & " " & G.Translation_Direction (1)'Image & " " &
                   G.Translation_Direction (2)'Image);
         Put_Line ("Essential matrix, x2^T E x1 = 0:");
         for I in 0 .. 2 loop
            Put_Line (G.Matrix (I, 0)'Image & " " & G.Matrix (I, 1)'Image & " " & G.Matrix (I, 2)'Image);
         end loop;
      else
         Put_Line ("No valid Essential matrix: pure rotation has no physical baseline.");
      end if;
      for Error of Correspondence_Errors (Result) loop
         if Error.Defined then Defined_Count := Defined_Count + 1; end if;
      end loop;
      Put_Line ("Individually defined errors: " & Defined_Count'Image);
      if Entire_Set_Evaluable (Result) then
         Put_Line ("Whole-set normalized Sampson RMS: " & Summary (Result).RMS_Normalized_Sampson'Image);
         Put_Line ("Maximum normalized Sampson error: " & Summary (Result).Maximum_Normalized_Sampson'Image);
      else
         Put_Line ("Whole-set RMS and maximum unavailable (not numeric zeros).");
      end if;
      Put_Line ("Verification disposition: " & Disposition (Result)'Image);
   end Report;
begin
   Put_Line ("SYNTHETIC observations only: not sensor or navigation evidence.");
   for I in P'Range loop
      declare
         W : constant Object_Point := [OpenCV.Float64_Value (I mod 6) * 0.5 - 1.3,
           OpenCV.Float64_Value (I / 6) * 0.4 - 0.9, 7.0 + OpenCV.Float64_Value (I mod 7) * 0.23];
      begin
         P (I) := Observe (W, False);
         Q (I) := Observe (W, True);
      end;
   end loop;
   Corrupted := Q;
   for I in 1 .. 4 loop
      Corrupted (I) := Q (24 - I);
   end loop;
   declare
      Clean : constant Known_Pose_Epipolar_Result := Verify_Known_Pose_Epipolar
        (P, Q, K1, K2, D1, D2, First, Second, Policy);
      Bad : constant Known_Pose_Epipolar_Result := Verify_Known_Pose_Epipolar
        (P, Corrupted, K1, K2, D1, D2, First, Second, Policy);
      Pure : constant Known_Pose_Epipolar_Result := Verify_Known_Pose_Epipolar
        (P, Q, K1, K2, D1, D2, First, Pure_Rotation, Policy);
      G : constant Known_Pose_Essential_Result := Geometry (Clean);
      Norm : constant OpenCV.Float64_Value := Math.Sqrt (4.09);
      --  Hand-derived E for Rz(.2), t=(-2,.3,0), independent of builder.
      Expected : constant Essential_Matrix :=
        [[0.0, 0.0, 0.3 / Norm], [0.0, 0.0, 2.0 / Norm],
         [(-0.3 * C - 2.0 * S) / Norm, (0.3 * S - 2.0 * C) / Norm, 0.0]];
   begin
      Report ("Clean set", Clean); Report ("Deliberately mismatched set", Bad); Report ("Pure rotation", Pure);
      if not Accepted (Clean) or else Disposition (Bad) /= Rejected_By_Policy or else
        Disposition (Pure) /= No_Physical_Baseline or else Geometry (Pure).Defined or else
        Entire_Set_Evaluable (Pure) or else abs (G.Baseline_Distance - Norm) > 1.0E-12
      then
         raise Program_Error with "synthetic verification assertions failed";
      end if;
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            if abs (G.Matrix (I, J) - Expected (I, J)) > 1.0E-12 then
               raise Program_Error with "independent signed Essential oracle failed";
            end if;
         end loop;
      end loop;
   end;
end Known_Pose_Epipolar_Synthetic;