with Ada.Numerics;
with Ada.Text_IO;
with OpenCV.Calib3D;

procedure Plane_Correspondence_Synthetic is
   use Ada.Text_IO;
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   KA : constant Camera_Intrinsics := (1000.0, 950.0, 320.0, 240.0);
   KB : constant Camera_Intrinsics := (800.0, 700.0, 300.0, 200.0);
   DA : constant Distortion_Coefficients := (0.08, -0.02, 0.003, -0.004, 0.005);
   DB : constant Distortion_Coefficients := (-0.06, 0.013, -0.002, 0.003, -0.001);
   A : constant World_To_Camera_Pose :=
     ([OpenCV.Float64_Value (Ada.Numerics.Pi), 0.0, 0.0], [0.0, 0.0, 10.0]);
   B : constant World_To_Camera_Pose :=
     ([OpenCV.Float64_Value (Ada.Numerics.Pi), 0.0, 0.0], [-2.0, 0.0, 10.0]);
   --  Tilted plane Z=0.2X-0.1Y. All observations below are computed
   --  independently from analytic Rx(pi) and scalar Brown, never Project_Points.
   Plane : constant World_Plane := ([0.0, 0.0, 0.0], [-0.2, 0.1, 1.0]);
   Known : constant Object_Point_Array :=
     [[0.0, 0.0, 0.0], [1.0, -2.0, 0.4], [-3.0, 1.0, -0.7], [0.5, 0.5, 0.05]];
   Policy : constant Plane_Correspondence_Verification_Options := ((0.01, 0.0, 1000.0, 1.0E-8), 0.01);
   First, Second : Image_Point_Array (Known'Range);
   Original_Count, Source_Count, Evaluated_Count, Positive_Count, Pixel_Rejections, Final_Count : Natural := 0;
   Maximum_World_Error : OpenCV.Float64_Value := 0.0;

   procedure Check (Condition : Boolean; Message : String) is
   begin
      if not Condition then
         raise Program_Error with Message;
      end if;
   end Check;

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

   procedure Report (Label : String; Values : Plane_Correspondence_Result_Array) is
   begin
      Put_Line (Label);
      Original_Count := Original_Count + Values'Length;
      for R of Values loop
         Put_Line ("position" & Positive'Image (Original_Position (R)) & ": " &
                   Plane_Correspondence_Disposition'Image (Disposition (R)));
         if Accepted (Source_Intersection (R)) then Source_Count := Source_Count + 1; end if;
         if Second_View_Evaluated (R) then
            Evaluated_Count := Evaluated_Count + 1;
            if Second_View_Disposition (R) = Projected_Positive_Depth then
               Positive_Count := Positive_Count + 1;
            end if;
         end if;
         if Disposition (R) = Second_Reprojection_Rejected then Pixel_Rejections := Pixel_Rejections + 1; end if;
         if Accepted (R) then
            Final_Count := Final_Count + 1;
            Put_Line ("accepted original position:" & Positive'Image (Original_Position (R)));
         end if;
      end loop;
   end Report;
begin
   Put_Line ("SYNTHETIC observations: do not establish physical terrain-registration accuracy.");
   Put_Line ("Supplied Cartesian poses/plane; X_camera=R*X_world+t; translation is not camera position.");
   for I in Known'Range loop
      First (I) := Brown (Known (I), KA, DA, 0.0);
      Second (I) := Brown (Known (I), KB, DB, 2.0);
   end loop;
   Second (4) (0) := Second (4) (0) + 10.0;
   declare
      Values : constant Plane_Correspondence_Result_Array := Verify_Plane_Constrained_Correspondences
        (First, Second, KA, KB, DA, DB, A, B, Plane, Policy);
   begin
      Report ("Tilted plane, independent intrinsics and all-five distortion:", Values);
      for I in Values'Range loop
         Check (Original_Position (Values (I)) = I, "original position changed");
         Check (Disposition (Values (I)) =
                  (if I = 4 then Second_Reprojection_Rejected else Plane_Correspondence_Accepted),
                "unexpected tilted-plane disposition");
         --  Source geometry remains available even for the mismatched target.
         for Axis in 0 .. 2 loop
            declare Error : constant OpenCV.Float64_Value :=
              abs (Accepted_World_Position (Source_Intersection (Values (I))) (Axis) - Known (I) (Axis));
            begin
               Check (Error < 1.0E-8, "independent tilted world oracle violated");
               Maximum_World_Error := OpenCV.Float64_Value'Max (Maximum_World_Error, Error);
            end;
         end loop;
      end loop;
   end;
   declare
      --  Identity source and plane X+Z=1: x=-0.999 is a forward
      --  near-parallel ray, while x=1 intersects at (0.5,0,0.5).
      Values : constant Plane_Correspondence_Result_Array := Verify_Plane_Constrained_Correspondences
        ([[-0.999, 0.0], [1.0, 0.0]], [[0.0, 0.0], [0.0, 0.0]],
         (1.0, 1.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0), No_Distortion, No_Distortion,
         (others => <>), ([0.0, 0.0, 0.0], [0.0, 0.0, -0.75]),
         ([0.0, 0.0, 1.0], [1.0, 0.0, 1.0]), Policy);
   begin
      Report ("Separate grazing/nonpositive-target-depth fixture:", Values);
      Check (Disposition (Values (1)) = Source_Intersection_Rejected and then
             Disposition (Source_Intersection (Values (1))) = Grazing_Incidence_Rejected and then
             not Second_View_Evaluated (Values (1)), "grazing rejection violated");
      Check (Disposition (Values (2)) = Second_Non_Positive_Depth and then
             abs (Second_Camera_Depth (Values (2)) + 0.25) < 1.0E-12,
             "nonpositive-depth target oracle violated");
   end;
   Put_Line ("Original correspondence count:" & Natural'Image (Original_Count));
   Put_Line ("Source-plane acceptance count:" & Natural'Image (Source_Count));
   Put_Line ("Target-camera evaluated count:" & Natural'Image (Evaluated_Count));
   Put_Line ("Target-positive-depth count:" & Natural'Image (Positive_Count));
   Put_Line ("Target pixel-error rejection count:" & Natural'Image (Pixel_Rejections));
   Put_Line ("Final accepted count:" & Natural'Image (Final_Count));
   Put_Line ("Maximum clean world-coordinate error:" & OpenCV.Float64_Value'Image (Maximum_World_Error));
   Check (Original_Count = 6 and then Source_Count = 5 and then Evaluated_Count = 5 and then
          Positive_Count = 4 and then Pixel_Rejections = 1 and then Final_Count = 3, "summary oracle violated");
   Put_Line ("PASS: synthetic plane-constrained correspondence assertions");
end Plane_Correspondence_Synthetic;