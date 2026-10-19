with Ada.Numerics;
with Ada.Numerics.Generic_Elementary_Functions;
with Ada.Text_IO;
with OpenCV.Calib3D;

procedure Known_Plane_Homography_Synthetic is
   use Ada.Text_IO;
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   package Math is new Ada.Numerics.Generic_Elementary_Functions (OpenCV.Float64_Value);
   KA : constant Camera_Intrinsics := (1000.0, 1000.0, 320.0, 240.0);
   KB : constant Camera_Intrinsics := (800.0, 700.0, 300.0, 200.0);
   A : constant World_To_Camera_Pose := ([Ada.Numerics.Pi, 0.0, 0.0], [0.0, 0.0, 10.0]);
   B : constant World_To_Camera_Pose := ([Ada.Numerics.Pi, 0.0, 0.0], [-2.0, 0.0, 10.0]);
   Ground : constant World_Plane := ([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
   Maximum_Residual : OpenCV.Float64_Value := 0.0;

   procedure Check (Condition : Boolean; Message : String) is
   begin
      if not Condition then raise Program_Error with Message; end if;
   end Check;

   procedure Print_Matrix (Label : String; H : Homography_Matrix) is
   begin
      Put_Line (Label & " (arbitrary nonzero homogeneous scale):");
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop Put (OpenCV.Float64_Value'Image (H (I, J)) & " "); end loop;
         New_Line;
      end loop;
   end Print_Matrix;

   procedure Report (Label : String; G : Known_Plane_Homography_Result) is
   begin
      Put_Line (Label & ": " & Known_Plane_Homography_Disposition'Image (Disposition (G)));
      if Defined (G) then
         Print_Matrix ("Normalized H12", Normalized_Homography (G));
         Print_Matrix ("IDEAL UNDISTORTED pinhole pixel H12, not raw Brown pixels", Ideal_Pixel_Homography (G, KA, KB));
      end if;
   end Report;

   procedure Transfer (G : Known_Plane_Homography_Result; P, Expected : Normalized_Image_Point) is
      R : constant Normalized_Plane_Transfer_Array := Transfer_Normalized_Plane_Points (G, [P]);
   begin
      Check (R (1).Finite, "expected finite synthetic transfer");
      Put_Line ("Normalized input: " & OpenCV.Float64_Value'Image (P (0)) & OpenCV.Float64_Value'Image (P (1)));
      Put_Line ("Normalized output:" & OpenCV.Float64_Value'Image (R (1).Point (0)) & OpenCV.Float64_Value'Image (R (1).Point (1)));
      for I in 0 .. 1 loop
         Maximum_Residual := OpenCV.Float64_Value'Max (Maximum_Residual, abs (R (1).Point (I) - Expected (I)));
         Check (abs (R (1).Point (I) - Expected (I)) < 1.0E-9, "independent synthetic projection oracle");
      end loop;
   end Transfer;

   --  Independent scalar axis-angle and five-term Brown equations, EXAMPLE
   --  oracle only; never used by the production construction/transfer path.
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
      X := C (0) / C (2);
      Y := C (1) / C (2);
      R2 := X * X + Y * Y;
      Radial := 1.0 + D.K1 * R2 + D.K2 * R2**2 + D.K3 * R2**3;
      return [K.Center_X + K.Focal_X * (X * Radial + 2.0 * D.P1 * X * Y + D.P2 * (R2 + 2.0 * X * X)),
              K.Center_Y + K.Focal_Y * (Y * Radial + D.P1 * (R2 + 2.0 * Y * Y) + 2.0 * D.P2 * X * Y)];
   end Observe;
begin
   Put_Line ("SYNTHETIC observations only: no physical terrain registration or navigation accuracy established.");
   Put_Line ("X_camera=R*X_world+t; X2=R21*X1+t21. t21 is in CAMERA 2, not world center displacement.");
   declare
      G : constant Known_Plane_Homography_Result := Construct_Known_Plane_Homography (A, B, Ground);
      Pixel : constant Homography_Point_Result := Map_With_Homography (Ideal_Pixel_Homography (G, KA, KB), [420.0, 440.0]);
   begin
      Report ("Rectified downward cameras, Z=0", G);
      Check (Disposition (G) = General_Plane_Homography, "general plane disposition");
      Transfer (G, [0.1, 0.2], [-0.1, 0.2]);
      Check (Pixel.Finite and then abs (Pixel.Point (0) - 220.0) < 1.0E-9
             and then abs (Pixel.Point (1) - 340.0) < 1.0E-9, "independent different-intrinsics pixel oracle");
   end;
   declare
      P1 : constant World_To_Camera_Pose := ([0.31, -0.22, 0.14], [0.2, -0.3, 1.0]);
      P2 : constant World_To_Camera_Pose := ([-0.18, 0.29, -0.24], [-0.7, 0.4, 0.5]);
      Plane : constant World_Plane := ([0.0, 0.0, 4.0], [-0.2, 0.5, 1.0]);
      Points : constant Object_Point_Array := [[1.0, 2.0, 3.2], [-2.0, 1.0, 3.1], [0.3, -0.4, 4.26]];
      D1 : constant Distortion_Coefficients := (0.08, -0.02, 0.003, -0.004, 0.005);
      D2 : constant Distortion_Coefficients := (-0.06, 0.013, -0.002, 0.003, -0.001);
      G : constant Known_Plane_Homography_Result := Construct_Known_Plane_Homography (P1, P2, Plane);
   begin
      Report ("Noncommuting rotations, tilted plane, independent Brown observations", G);
      for P of Points loop
         declare
            U : constant Normalized_Image_Point_Array := Undistort_To_Normalized ([Observe (P, P1, KA, D1)], KA, D1);
            V : constant Normalized_Image_Point_Array := Undistort_To_Normalized ([Observe (P, P2, KB, D2)], KB, D2);
            Expected : constant Image_Point := Observe (P, P2, (1.0, 1.0, 0.0, 0.0), No_Distortion);
         begin
            Transfer (G, U (1), Normalized_Image_Point (Expected));
            Transfer (G, U (1), V (1));
         end;
      end loop;
   end;
   declare
      G : constant Known_Plane_Homography_Result := Construct_Known_Plane_Homography
        ((others => <>), ([0.0, 0.0, Ada.Numerics.Pi / 2.0], [others => 0.0]), Ground);
   begin
      Report ("Pure rotation, supplied plane contains common center", G);
      Check (Disposition (G) = Pure_Rotation_Homography, "pure rotation disposition");
      Transfer (G, [0.1, 0.2], [-0.2, 0.1]);
      Put_Line ("Pure rotation is algebraic: does not establish forward ray-plane intersection or positive depth.");
   end;
   declare
      G : constant Known_Plane_Homography_Result := Construct_Known_Plane_Homography
        ((others => <>), ([others => 0.0], [-2.0, 0.0, 0.0]), Ground);
      H : constant Known_Plane_Homography_Result := Construct_Known_Plane_Homography
        ((others => <>), ([others => 0.0], [0.0, 0.0, -1.0]), ([0.0, 0.0, 2.0], [2.0, 0.0, 1.0]));
      R : constant Normalized_Plane_Transfer_Array := Transfer_Normalized_Plane_Points (H, [[0.5, 0.0]]);
   begin
      Report ("Plane through first center, nonzero baseline", G);
      Check (not Defined (G) and then Disposition (G) = Plane_Through_First_Camera_Center, "degenerate plane guard");
      Check (not R (1).Finite, "projective horizon unavailable");
      Put_Line ("Projective transfer at (0.5,0): UNAVAILABLE, zero homogeneous denominator; no coordinates published.");
   end;
   Put_Line ("Maximum independent synthetic mapping residual:" & OpenCV.Float64_Value'Image (Maximum_Residual));
   Put_Line ("PASS: known-plane homography synthetic assertions");
end Known_Plane_Homography_Synthetic;