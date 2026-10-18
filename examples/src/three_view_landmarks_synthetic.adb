with Ada.Text_IO; use Ada.Text_IO;
with Ada.Numerics.Generic_Elementary_Functions;
with OpenCV.Calib3D; use OpenCV.Calib3D;

procedure Three_View_Landmarks_Synthetic is
   use type OpenCV.Float64_Value;
   package Math is new Ada.Numerics.Generic_Elementary_Functions (OpenCV.Float64_Value);
   type Camera_Number is range 1 .. 3;
   K : constant array (Camera_Number) of Camera_Intrinsics :=
     [(800.0, 820.0, 320.0, 240.0), (630.0, 710.0, 280.0, 190.0), (750.0, 690.0, 300.0, 220.0)];
   D : constant array (Camera_Number) of Distortion_Coefficients :=
     [(0.02, -0.003, 0.001, -0.002, 0.0004), (-0.03, 0.004, -0.002, 0.001, -0.0005),
      (0.01, -0.002, 0.0005, -0.001, 0.0002)];
   Poses : constant array (Camera_Number) of World_To_Camera_Pose :=
     [([0.0, 0.0, 0.15], [0.0, 0.0, 0.0]),
      ([0.0, 0.0, 0.15], [-2.0, 0.0, 0.0]),
      ([0.0, -0.2, 0.0], [0.3, -0.4, 0.5])];
   Truth : Object_Point_Array (1 .. 16);
   P, Q, H : Image_Point_Array (Truth'Range);
   Policy : constant Three_View_Landmark_Options :=
     (1.0E-9, (0.001, 1.0E-8, 1.0E-5, 1.0E-5), (20, 0.0), (1.0E-5, 1.0E-5),
      Prefer_Refined_With_Original_Fallback, 1.0E-4);
   Pass, Reject, Initial, Refined, Depth, Pixel, Total : Natural := 0;
   Maximum_Error : OpenCV.Float64_Value := 0.0;

   --  Independent analytic rotations and scalar Brown forward projection.
   --  No production transform/projection generates the synthetic observations.
   function Observe (W : Object_Point; Camera : Camera_Number) return Image_Point is
      C : constant OpenCV.Float64_Value := Math.Cos (if Camera = 3 then -0.2 else 0.15);
      S : constant OpenCV.Float64_Value := Math.Sin (if Camera = 3 then -0.2 else 0.15);
      X : constant OpenCV.Float64_Value := (if Camera = 3 then C * W (0) + S * W (2) else C * W (0) - S * W (1));
      Y : constant OpenCV.Float64_Value := (if Camera = 3 then W (1) else S * W (0) + C * W (1));
      Z : constant OpenCV.Float64_Value := (if Camera = 3 then -S * W (0) + C * W (2) else W (2));
      U : constant OpenCV.Float64_Value := (X + Poses (Camera).Translation (0)) / (Z + Poses (Camera).Translation (2));
      V : constant OpenCV.Float64_Value := (Y + Poses (Camera).Translation (1)) / (Z + Poses (Camera).Translation (2));
      R2 : constant OpenCV.Float64_Value := U * U + V * V;
      Model : constant Distortion_Coefficients := D (Camera);
      Radial : constant OpenCV.Float64_Value := 1.0 + Model.K1 * R2 + Model.K2 * R2 ** 2 + Model.K3 * R2 ** 3;
   begin
      return [K (Camera).Focal_X * (U * Radial + 2.0 * Model.P1 * U * V + Model.P2 * (R2 + 2.0 * U * U)) + K (Camera).Center_X,
              K (Camera).Focal_Y * (V * Radial + Model.P1 * (R2 + 2.0 * V * V) + 2.0 * Model.P2 * U * V) + K (Camera).Center_Y];
   end Observe;
begin
   Put_Line ("SYNTHETIC three-view geometry only; no sensor or navigation-accuracy claim.");
   for I in Truth'Range loop
      Truth (I) := [OpenCV.Float64_Value (I mod 4) * 0.4 - 0.7,
        OpenCV.Float64_Value (I / 4) * 0.3 - 0.5, 7.0 + OpenCV.Float64_Value (I mod 5) * 0.3];
      P (I) := Observe (Truth (I), 1);
      Q (I) := Observe (Truth (I), 2);
      H (I) := Observe (Truth (I), 3);
   end loop;
   Q (2) (1) := Q (2) (1) + 100.0;
   Q (5) (1) := Q (5) (1) - 80.0;
   --  Scaling a world point on its first-camera ray preserves its first
   --  observation. Stereo y still agrees, but the implied depth is twice truth.
   Q (8) := Observe ([for Component of Truth (8) => 2.0 * Component], 2);
   H (11) (0) := H (11) (0) + 50.0;
   declare
      Results : constant Three_View_Landmark_Result_Array := Qualify_Three_View_Landmarks
        (P, Q, H, K (1), K (2), K (3), D (1), D (2), D (3), Poses (1), Poses (2), Poses (3), Policy);
      Landmarks : constant Accepted_Landmark_Set := Three_View_Landmark_Observations (Results, H);
   begin
      for I in Results'Range loop
         if Epipolar_Disposition (Results (I)) = Epipolar_Passed then Pass := Pass + 1;
         else Reject := Reject + 1; end if;
         case Candidate_Source (Results (I)) is
            when Initial_Metric_Candidate => Initial := Initial + 1;
            when Refined_Metric_Candidate => Refined := Refined + 1;
            when No_Selected_Candidate => null;
         end case;
         case Disposition (Results (I)) is
            when Third_Non_Positive_Depth | Third_Numerically_Unavailable => Depth := Depth + 1;
            when Third_Reprojection_Rejected => Pixel := Pixel + 1;
            when others => null;
         end case;
         if Accepted (Results (I)) then
            if I = 2 or else I = 5 or else I = 8 or else I = 11 then
               raise Program_Error with "independently known incorrect track accepted";
            end if;
            Total := Total + 1;
            for Axis in 0 .. 2 loop
               Maximum_Error := OpenCV.Float64_Value'Max (Maximum_Error,
                 abs (Accepted_World_Position (Results (I)) (Axis) - Truth (I) (Axis)));
            end loop;
         elsif I /= 2 and then I /= 5 and then I /= 8 and then I /= 11 then
            raise Program_Error with "clean well-conditioned track unexpectedly rejected";
         end if;
      end loop;
      if Epipolar_Disposition (Results (8)) /= Epipolar_Passed or else
        Disposition (Results (8)) /= Third_Reprojection_Rejected or else Maximum_Error > 1.0E-7
      then
         raise Program_Error with "independent wrong-depth or world-coordinate oracle failed";
      end if;
      Put_Line ("Original matched-track count:" & Results'Length'Image);
      Put_Line ("Epipolar screening pass/reject:" & Pass'Image & " /" & Reject'Image);
      Put_Line ("Accepted fitting-stage initial/refined candidates:" & Initial'Image & " /" & Refined'Image);
      Put_Line ("Third depth-or-numerical/pixel rejections:" & Depth'Image & " /" & Pixel'Image);
      Put_Line ("Final accepted world-landmark count:" & Total'Image);
      Put ("Original accepted correspondence positions:");
      for Position of Original_Positions (Landmarks) loop Put (Position'Image); end loop;
      New_Line;
      Put_Line ("Maximum clean world-coordinate component error:" & Maximum_Error'Image);
   end;
end Three_View_Landmarks_Synthetic;