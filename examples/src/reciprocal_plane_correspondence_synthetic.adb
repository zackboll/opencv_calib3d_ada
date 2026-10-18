with Ada.Numerics;
with Ada.Text_IO;
with OpenCV.Calib3D;

procedure Reciprocal_Plane_Correspondence_Synthetic is
   use Ada.Text_IO;
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   KA : constant Camera_Intrinsics := (1000.0, 1000.0, 320.0, 240.0);
   KB : constant Camera_Intrinsics := (800.0, 700.0, 300.0, 200.0);
   A : constant World_To_Camera_Pose :=
     ([OpenCV.Float64_Value (Ada.Numerics.Pi), 0.0, 0.0], [0.0, 0.0, 10.0]);
   B : constant World_To_Camera_Pose :=
     ([OpenCV.Float64_Value (Ada.Numerics.Pi), 0.0, 0.0], [-2.0, 0.0, 10.0]);
   Ground : constant World_Plane := ([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
   Direction_Policy : constant Plane_Correspondence_Verification_Options := ((0.01, 0.0, 1000.0, 1.0E-8), 15.0);
   Policy : constant Reciprocal_Plane_Verification_Options := (Direction_Policy, Direction_Policy, 0.05);
   Tracks, Forward_Count, Reverse_Count, Both_Sources, Separation_Rejections, Final_Count : Natural := 0;
   Maximum_Accepted_Separation : OpenCV.Float64_Value := 0.0;

   procedure Check (Condition : Boolean; Message : String) is
   begin
      if not Condition then raise Program_Error with Message; end if;
   end Check;

   procedure Check_Point (Actual, Expected : Object_Point) is
   begin
      for Axis in 0 .. 2 loop
         Check (abs (Actual (Axis) - Expected (Axis)) < 1.0E-8, "independent synthetic world oracle");
      end loop;
   end Check_Point;

   --  Independent analytic Rx(pi) and Brown projection, example only.
   function Brown (P : Object_Point; K : Camera_Intrinsics; D : Distortion_Coefficients;
                   Center_X : OpenCV.Float64_Value) return Image_Point is
      X : constant OpenCV.Float64_Value := (P (0) - Center_X) / (10.0 - P (2));
      Y : constant OpenCV.Float64_Value := -P (1) / (10.0 - P (2));
      R2 : constant OpenCV.Float64_Value := X * X + Y * Y;
      Radial : constant OpenCV.Float64_Value := 1.0 + D.K1 * R2 + D.K2 * R2**2 + D.K3 * R2**3;
   begin
      return [K.Center_X + K.Focal_X * (X * Radial + 2.0 * D.P1 * X * Y + D.P2 * (R2 + 2.0 * X * X)),
              K.Center_Y + K.Focal_Y * (Y * Radial + D.P1 * (R2 + 2.0 * Y * Y) + 2.0 * D.P2 * X * Y)];
   end Brown;

   procedure Report (Label : String; Values : Reciprocal_Plane_Result_Array) is
   begin
      Put_Line ("SYNTHETIC fixture: " & Label);
      Tracks := Tracks + Values'Length;
      for R of Values loop
         Put_Line ("Original position" & Positive'Image (Original_Position (R)) & ": " &
                   Reciprocal_Plane_Disposition'Image (Disposition (R)));
         if Accepted (Forward_Verification (R)) then Forward_Count := Forward_Count + 1; end if;
         if Accepted (Reverse_Verification (R)) then Reverse_Count := Reverse_Count + 1; end if;
         if Both_Source_Intersections_Accepted (R) then Both_Sources := Both_Sources + 1; end if;
         if Disposition (R) = World_Separation_Rejected then Separation_Rejections := Separation_Rejections + 1; end if;
         if Accepted (R) then
            Final_Count := Final_Count + 1;
            Maximum_Accepted_Separation := OpenCV.Float64_Value'Max (Maximum_Accepted_Separation, World_Separation (R));
            Put_Line ("Accepted original correspondence position:" & Positive'Image (Original_Position (R)));
         end if;
      end loop;
   end Report;
begin
   Put_Line ("SYNTHETIC observations only; no physical terrain-registration or navigation-accuracy claim.");
   Put_Line ("X_camera=R*X_world+t; translation is not camera position. No world-point fusion.");
   declare
      R : constant Reciprocal_Plane_Result_Array := Verify_Reciprocal_Plane_Correspondences
        ([[420.0, 440.0], [420.0, 440.0]], [[220.0, 340.0], [228.0, 340.0]],
         KA, KB, No_Distortion, No_Distortion, A, B, Ground, Policy);
      Asymmetric : Reciprocal_Plane_Verification_Options := Policy;
   begin
      Report ("clean match and B +8 pixel mismatch / separation-only rejection", R);
      Check (Accepted (R (1)) and then Disposition (R (2)) = World_Separation_Rejected, "separation gate");
      Check_Point (Accepted_First_Source_World_Position (R (1)), [1.0, -2.0, 0.0]);
      Check_Point (Accepted_Second_Source_World_Position (R (1)), [1.0, -2.0, 0.0]);
      Check_Point (First_Source_World_Position (R (2)), [1.0, -2.0, 0.0]);
      Check_Point (Second_Source_World_Position (R (2)), [1.1, -2.0, 0.0]);
      Check (abs (World_Separation (R (2)) - 0.1) < 1.0E-12 and then
             abs (Second_Pixel_Error (Forward_Verification (R (2))) - 8.0) < 1.0E-10 and then
             abs (Second_Pixel_Error (Reverse_Verification (R (2))) - 10.0) < 1.0E-10, "8/10/0.1 oracle");
      Asymmetric.Forward.Maximum_Second_Pixel_Error := 5.0;
      declare
         Rejected : constant Reciprocal_Plane_Result_Array := Verify_Reciprocal_Plane_Correspondences
           ([[420.0, 440.0]], [[228.0, 340.0]], KA, KB, No_Distortion, No_Distortion, A, B, Ground, Asymmetric);
      begin
         Report ("independent directional policy rejection", Rejected);
         Check (Disposition (Rejected (1)) = Forward_Verification_Rejected and then
                Accepted (Reverse_Verification (Rejected (1))) and then World_Separation_Evaluated (Rejected (1)),
                "direction-specific rejection must retain separation");
      end;
   end;
   declare
      R : constant Reciprocal_Plane_Result_Array := Verify_Reciprocal_Plane_Correspondences
        ([[-0.999, 0.0]], [[0.0, 0.0]], (1.0, 1.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0),
         No_Distortion, No_Distortion, (others => <>), (others => <>),
         ([0.0, 0.0, 1.0], [1.0, 0.0, 1.0]), Policy);
   begin
      Report ("source grazing rejection", R);
      Check (Disposition (R (1)) = Forward_Source_Rejected and then
             Disposition (Source_Intersection (Forward_Verification (R (1)))) = Grazing_Incidence_Rejected and then
             not World_Separation_Evaluated (R (1)), "rejected source cannot create world separation");
   end;
   declare
      DA : constant Distortion_Coefficients := (0.08, -0.02, 0.003, -0.004, 0.005);
      DB : constant Distortion_Coefficients := (-0.06, 0.013, -0.002, 0.003, -0.001);
      Plane : constant World_Plane := ([0.0, 0.0, 0.0], [-0.2, 0.1, 1.0]);
      Known : constant Object_Point_Array := [[1.0, -2.0, 0.4], [-3.0, 1.0, -0.7]];
      First, Second : Image_Point_Array (Known'Range);
   begin
      for I in Known'Range loop
         First (I) := Brown (Known (I), KA, DA, 0.0);
         Second (I) := Brown (Known (I), KB, DB, 2.0);
      end loop;
      declare
         R : constant Reciprocal_Plane_Result_Array := Verify_Reciprocal_Plane_Correspondences
           (First, Second, KA, KB, DA, DB, A, B, Plane, Policy);
      begin
         Report ("tilted plane / independent five-coefficient Brown cameras", R);
         for I in Known'Range loop
            Check (Accepted (R (I)), "tilted Brown acceptance");
            Check_Point (Accepted_First_Source_World_Position (R (I)), Known (I));
            Check_Point (Accepted_Second_Source_World_Position (R (I)), Known (I));
         end loop;
      end;
   end;
   declare
      --  Both centers are zero; B=Rz(pi/2), so P=(1,-2,4) -> (2,1,4).
      R : constant Reciprocal_Plane_Result_Array := Verify_Reciprocal_Plane_Correspondences
        ([[570.0, -260.0]], [[700.0, 375.0]], KA, KB, No_Distortion, No_Distortion,
         (others => <>), ([0.0, 0.0, Ada.Numerics.Pi / 2.0], [others => 0.0]),
         ([0.0, 0.0, 4.0], [0.0, 0.0, 1.0]), Policy);
   begin
      Report ("same physical center / different rotations", R);
      Check (Accepted (R (1)), "same-center acceptance");
      Check_Point (Accepted_First_Source_World_Position (R (1)), [1.0, -2.0, 4.0]);
      Check_Point (Accepted_Second_Source_World_Position (R (1)), [1.0, -2.0, 4.0]);
   end;
   Put_Line ("Original track count:" & Natural'Image (Tracks));
   Put_Line ("Forward accepted count:" & Natural'Image (Forward_Count));
   Put_Line ("Reverse accepted count:" & Natural'Image (Reverse_Count));
   Put_Line ("Count with both source world positions:" & Natural'Image (Both_Sources));
   Put_Line ("World-separation rejection count:" & Natural'Image (Separation_Rejections));
   Put_Line ("Final reciprocal accepted count:" & Natural'Image (Final_Count));
   Put_Line ("Maximum accepted world separation:" & OpenCV.Float64_Value'Image (Maximum_Accepted_Separation));
   Check (Tracks = 7 and then Forward_Count = 5 and then Reverse_Count = 7 and then Both_Sources = 6 and then
          Separation_Rejections = 1 and then Final_Count = 4, "summary counts");
   Put_Line ("PASS: synthetic reciprocal plane correspondence assertions");
end Reciprocal_Plane_Correspondence_Synthetic;