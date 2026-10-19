with Ada.Numerics;
with Ada.Numerics.Generic_Elementary_Functions;
with Ada.Text_IO;
with OpenCV.Calib3D;

procedure Plane_Homography_Consistency_Synthetic is
   use Ada.Text_IO;
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   package Math is new Ada.Numerics.Generic_Elementary_Functions (OpenCV.Float64_Value);
   Policy : constant Plane_Homography_Consistency_Options := (3000, 1.0E-5, 0.999, 18, 2.0E-6, 2.0E-6, 2.0E-6);
   KA : constant Camera_Intrinsics := (900.0, 870.0, 320.0, 240.0);
   KB : constant Camera_Intrinsics := (760.0, 810.0, 300.0, 210.0);
   A : constant World_To_Camera_Pose := ([Ada.Numerics.Pi, 0.0, 0.0], [0.0, 0.0, 10.0]);
   B : constant World_To_Camera_Pose := ([Ada.Numerics.Pi, 0.0, 0.0], [-2.0, 0.0, 10.0]);
   Ground : constant World_Plane := ([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
   F1, F2 : Image_Point_Array (1 .. 24);
   H1, H2 : Image_Point_Array (1 .. 8);
   procedure Check (Condition : Boolean; Message : String) is
   begin
      if not Condition then raise Program_Error with Message; end if;
   end Check;

   --  Independent scalar axis-angle/perspective/Brown forward observation
   --  generator, not a production projection or homography call.
   function Observe (P : Object_Point; Pose : World_To_Camera_Pose;
                     K : Camera_Intrinsics; D : Distortion_Coefficients) return Image_Point is
      Angle : constant OpenCV.Float64_Value := Math.Sqrt
        (Pose.Rotation (0)**2 + Pose.Rotation (1)**2 + Pose.Rotation (2)**2);
      Axis : constant Object_Point :=
        [Pose.Rotation (0) / Angle, Pose.Rotation (1) / Angle, Pose.Rotation (2) / Angle];
      Dot : constant OpenCV.Float64_Value := Axis (0) * P (0) + Axis (1) * P (1) + Axis (2) * P (2);
      Cross : constant Object_Point := [Axis (1) * P (2) - Axis (2) * P (1),
        Axis (2) * P (0) - Axis (0) * P (2), Axis (0) * P (1) - Axis (1) * P (0)];
      C : Object_Point;
      X, Y, R2, Radial : OpenCV.Float64_Value;
   begin
      for I in 0 .. 2 loop
         C (I) := P (I) * Math.Cos (Angle) + Cross (I) * Math.Sin (Angle)
           + Axis (I) * Dot * (1.0 - Math.Cos (Angle)) + Pose.Translation (I);
      end loop;
      X := C (0) / C (2); Y := C (1) / C (2);
      R2 := X * X + Y * Y;
      Radial := 1.0 + D.K1 * R2 + D.K2 * R2**2 + D.K3 * R2**3;
      return [K.Center_X + K.Focal_X * (X * Radial + 2.0 * D.P1 * X * Y + D.P2 * (R2 + 2.0 * X * X)),
              K.Center_Y + K.Focal_Y * (Y * Radial + D.P1 * (R2 + 2.0 * Y * Y) + 2.0 * D.P2 * X * Y)];
   end Observe;

   procedure Generate (P, Q : out Image_Point_Array; Offset : OpenCV.Float64_Value;
                       First, Second : World_To_Camera_Pose;
                       DA, DB : Distortion_Coefficients; Tilted : Boolean) is
   begin
      for I in P'Range loop
         declare
            X : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((I - 1) mod 6) * 0.23 - 0.6 + Offset;
            Y : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((I - 1) / 6) * 0.25 - 0.4 + Offset;
            W : constant Object_Point := [X, Y, (if Tilted then 4.0 + 0.2 * X - 0.5 * Y else 0.0)];
         begin P (I) := Observe (W, First, KA, DA); Q (I) := Observe (W, Second, KB, DB); end;
      end loop;
   end Generate;

   procedure Report (Label : String; Expected : Boolean;
                     First : World_To_Camera_Pose := A; Second : World_To_Camera_Pose := B;
                     Plane : World_Plane := Ground;
                     DA : Distortion_Coefficients := No_Distortion;
                     DB : Distortion_Coefficients := No_Distortion) is
      R : constant Plane_Homography_Consistency_Result := Verify_Known_Plane_Homography_Consistency
        (F1, F2, H1, H2, KA, KB, DA, DB, First, Second, Plane, Policy);
      Count : Natural := 0;
      Maxima : array (Plane_Homography_Consistency_Metric) of OpenCV.Float64_Value := [others => 0.0];
   begin
      Put_Line ("SYNTHETIC " & Label);
      Put_Line ("Predicted: " & Known_Plane_Homography_Disposition'Image (Disposition (Predicted_Model (R))));
      Put_Line ("Empirical found: " & Boolean'Image (Empirical_Found (R)) &
                "; final fitting inliers:" & Natural'Image (Inlier_Count (R)) & "; held-out:" & Natural'Image (Holdout_Count (R)));
      for D of Holdouts (R) loop
         Put_Line ("Held-out" & Positive'Image (Original_Position (D)) & " " & Plane_Homography_Holdout_Disposition'Image (Disposition (D)));
         if Accepted (D) then Count := Count + 1; end if;
         for M in Plane_Homography_Consistency_Metric loop
            if Metric_Available (D, M) then
               Put_Line ("  " & Plane_Homography_Consistency_Metric'Image (M) & " normalized:" & OpenCV.Float64_Value'Image (Normalized_Error (D, M)));
               if Accepted (D) then Maxima (M) := OpenCV.Float64_Value'Max (Maxima (M), Normalized_Error (D, M)); end if;
            end if;
         end loop;
      end loop;
      Put_Line ("Held-out accepted:" & Natural'Image (Count));
      Put_Line ("Aggregate: " & Plane_Homography_Consistency_Disposition'Image (Disposition (R)));
      if Count > 0 then
         for M in Plane_Homography_Consistency_Metric loop
            Put_Line ("Maximum accepted " & Plane_Homography_Consistency_Metric'Image (M) & ":" & OpenCV.Float64_Value'Image (Maxima (M)));
         end loop;
      else Put_Line ("No accepted discrepancies available"); end if;
      Check (Empirical_Found (R) and then Accepted (R) = Expected, "synthetic expected acceptance/rejection");
      if Plane.Normal (0) = 0.0 and then Plane.Normal (1) = 0.0 then
         declare
            Map : constant Normalized_Plane_Transfer_Array := Transfer_Normalized_Plane_Points (Predicted_Model (R), [[0.1, 0.2]]);
         begin Check (Map (1).Finite and then abs (Map (1).Point (0) + 0.1) < 1.0E-11,
                      "independent downward H: x2=x1-.2"); end;
      end if;
   end Report;
begin
   Put_Line ("All data SYNTHETIC: no physical navigation-accuracy or statistical independence claim.");
   Generate (F1, F2, 0.0, A, B, No_Distortion, No_Distortion, False);
   Generate (H1, H2, 0.073, A, B, No_Distortion, No_Distortion, False);
   Report ("clean independent fitting and withheld world grids", True);
   F2 (2) := [8000.0, -7000.0]; F2 (7) := F2 (2); F2 (15) := F2 (2);
   Report ("three fitting outliers; unchanged held-out observations", True);
   Generate (F1, F2, 0.0, A, B, No_Distortion, No_Distortion, False);
   for P of F2 loop P (0) := P (0) + KB.Focal_X * 0.1; end loop;
   Report ("internally consistent shifted empirical fit, wrong supplied geometry", False);
   Generate (F1, F2, 0.0, A, B, No_Distortion, No_Distortion, False);
   H2 (3) (1) := H2 (3) (1) + 100.0;
   Report ("one corrupted held-out destination; models still agree", False);
   declare
      First : constant World_To_Camera_Pose := ([0.31, -0.22, 0.14], [0.2, -0.3, 1.0]);
      Second : constant World_To_Camera_Pose := ([-0.18, 0.29, -0.24], [-0.7, 0.4, 0.5]);
      DA : constant Distortion_Coefficients := (0.012, -0.004, 0.001, -0.002, 0.0003);
      DB : constant Distortion_Coefficients := (-0.009, 0.003, -0.002, 0.001, -0.0002);
   begin
      Generate (F1, F2, 0.0, First, Second, DA, DB, True);
      Generate (H1, H2, 0.073, First, Second, DA, DB, True);
      Report ("tilted plane, noncommuting rotations, independent five-term Brown models", True,
              First, Second, ([0.0, 0.0, 4.0], [-0.2, 0.5, 1.0]), DA, DB);
   end;
end Plane_Homography_Consistency_Synthetic;