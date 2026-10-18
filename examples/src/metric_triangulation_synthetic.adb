with Ada.Text_IO;
with Ada.Numerics.Generic_Elementary_Functions;
with OpenCV.Calib3D;

procedure Metric_Triangulation_Synthetic is
   use Ada.Text_IO;
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   package Math is new Ada.Numerics.Generic_Elementary_Functions (OpenCV.Float64_Value);
   A : constant World_To_Camera_Pose :=
     (Rotation => [0.0, 0.0, 0.3], Translation => [0.2, -0.1, 0.4]);
   B : constant World_To_Camera_Pose :=
     (Rotation => [-0.2, 0.0, 0.0], Translation => [-1.8, 0.3, 0.2]);
   KA : constant Camera_Intrinsics := (800.0, 820.0, 320.0, 240.0);
   KB : constant Camera_Intrinsics := (600.0, 640.0, 310.0, 220.0);
   DA : constant Distortion_Coefficients := (-0.12, 0.03, 0.001, -0.0015, 0.004);
   DB : constant Distortion_Coefficients := (0.08, -0.02, -0.002, 0.001, -0.003);
   Truth : constant Object_Point := [1.4, 0.6, 8.0];
   --  Independent analytic Rz(0.3) and Rx(-0.2), not binding transforms.
   CA : constant Object_Point :=
     [Math.Cos (0.3) * Truth (0) - Math.Sin (0.3) * Truth (1) + 0.2,
      Math.Sin (0.3) * Truth (0) + Math.Cos (0.3) * Truth (1) - 0.1, 8.4];
   CB : constant Object_Point :=
     [Truth (0) - 1.8, Math.Cos (-0.2) * Truth (1) - Math.Sin (-0.2) * Truth (2) + 0.3,
      Math.Sin (-0.2) * Truth (1) + Math.Cos (-0.2) * Truth (2) + 0.2];
   function Brown (P : Object_Point; K : Camera_Intrinsics; D : Distortion_Coefficients)
     return Image_Point
   is
      X : constant OpenCV.Float64_Value := P (0) / P (2);
      Y : constant OpenCV.Float64_Value := P (1) / P (2);
      R2 : constant OpenCV.Float64_Value := X * X + Y * Y;
      Radial : constant OpenCV.Float64_Value :=
        1.0 + D.K1 * R2 + D.K2 * R2 ** 2 + D.K3 * R2 ** 3;
   begin
      return [K.Focal_X * (X * Radial + 2.0 * D.P1 * X * Y + D.P2 * (R2 + 2.0 * X * X)) + K.Center_X,
              K.Focal_Y * (Y * Radial + D.P1 * (R2 + 2.0 * Y * Y) + 2.0 * D.P2 * X * Y) + K.Center_Y];
   end Brown;
   P : constant Image_Point_Array := [1 => Brown (CA, KA, DA)];
   Q : constant Image_Point_Array := [1 => Brown (CB, KB, DB)];
   N1 : constant Image_Point_Array := Project_Points ([1 => Truth], KA, DA, A);
   N2 : constant Image_Point_Array := Project_Points ([1 => Truth], KB, DB, B);
   Motion : constant Camera_Motion_Result := Measure_Camera_Motion (A, B);
   Result : constant Metric_Triangulated_Point_Array := Triangulate_World_Points
     (P, Q, KA, KB, DA, DB, A, B, (0.01, 1.0E-8, 1.0E-6, 1.0E-6));
   V : Metric_Triangulated_Point renames Result (1);
   Maximum : OpenCV.Float64_Value := 0.0;
   procedure Print (Label : String; Point : Object_Point) is
   begin
      Put (Label);
      for Component of Point loop
         Put (Component'Image & " ");
      end loop;
      New_Line;
   end Print;
begin
   Put_Line ("Synthetic Cartesian coordinates, NOT actual sensor measurements.");
   if V.Disposition /= Metric_Accepted then
      raise Program_Error with "metric reconstruction rejected";
   end if;
   for Axis in 0 .. 1 loop
      if abs (P (1) (Axis) - N1 (1) (Axis)) > 1.0E-10 or else
        abs (Q (1) (Axis) - N2 (1) (Axis)) > 1.0E-10
      then
         raise Program_Error with "independent distorted projection oracle violated";
      end if;
   end loop;
   for Axis in 0 .. 2 loop
      Maximum := OpenCV.Float64_Value'Max (Maximum, abs (V.Geometry.World_Position (Axis) - Truth (Axis)));
   end loop;
   Print ("Known world point: ", Truth);
   Print ("Unit-baseline first-camera point: ", V.Source.Point.Position_In_First_Camera);
   Print ("Metric first-camera point: ", V.Geometry.Position_In_First_Camera);
   Print ("Reconstructed world point: ", V.Geometry.World_Position);
   Put_Line ("Known-pose baseline: " & Motion.Baseline_Distance'Image);
   Put_Line ("Maximum world component error: " & Maximum'Image);
   Put_Line ("First pixel error: " & V.Pixels.First_Error_Pixels'Image);
   Put_Line ("Second pixel error: " & V.Pixels.Second_Error_Pixels'Image);
   Put_Line ("Acute parallax radians: " & V.Source.Quality.Parallax.Acute_Line_Angle_Radians'Image);
   if Maximum > 1.0E-9 then
      raise Program_Error with "independent Cartesian oracle violated";
   end if;
end Metric_Triangulation_Synthetic;
