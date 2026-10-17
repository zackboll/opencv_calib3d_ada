with Ada.Numerics;
with Ada.Numerics.Generic_Elementary_Functions;
with Ada.Text_IO;
with OpenCV.Calib3D;

procedure Relative_Motion_Synthetic is
   use Ada.Text_IO;
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;

   package Math is new Ada.Numerics.Generic_Elementary_Functions (OpenCV.Float64_Value);

   --  Independent scalar Brown projection, not a second native call.
   function Project_Oracle
     (Point : Object_Point; K : Camera_Intrinsics; D : Distortion_Coefficients)
      return Image_Point
   is
      X : constant OpenCV.Float64_Value := Point (0) / Point (2);
      Y : constant OpenCV.Float64_Value := Point (1) / Point (2);
      Radius_2 : constant OpenCV.Float64_Value := X * X + Y * Y;
      Radial : constant OpenCV.Float64_Value :=
        1.0 + D.K1 * Radius_2 + D.K2 * Radius_2 ** 2 + D.K3 * Radius_2 ** 3;
   begin
      return [K.Focal_X * (X * Radial + 2.0 * D.P1 * X * Y
                            + D.P2 * (Radius_2 + 2.0 * X * X)) + K.Center_X,
              K.Focal_Y * (Y * Radial + D.P1 * (Radius_2 + 2.0 * Y * Y)
                            + 2.0 * D.P2 * X * Y) + K.Center_Y];
   end Project_Oracle;

   Intrinsics : constant Camera_Intrinsics :=
     (Focal_X => 800.0, Focal_Y => 820.0, Center_X => 320.0, Center_Y => 240.0);
   --  Known synthetic world-to-camera poses in one caller Cartesian frame.
   --  These are not measurements and are not produced by a pose estimator.
   Pi : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Ada.Numerics.Pi);
   First : constant World_To_Camera_Pose :=
     (Rotation => [0.0, 0.0, Pi / 3.0], Translation => [1.0, 0.0, 0.0]);
   Second : constant World_To_Camera_Pose :=
     (Rotation => [Pi / 4.0, 0.0, 0.0], Translation => [0.0, 2.0, 0.0]);
   Relative : constant World_To_Camera_Pose :=
     Camera_To_Camera_Pose (First => First, Second => Second);
   Motion : constant Camera_Motion_Result :=
     Measure_Camera_Motion (First => First, Second => Second);
   World_Point : constant Object_Point := [0.6, -0.4, 2.5];
   In_First : constant Object_Point := World_To_Camera_Point (First, World_Point);
   Through_Relative : constant Object_Point := World_To_Camera_Point (Relative, In_First);
   Direct_Second : constant Object_Point := World_To_Camera_Point (Second, World_Point);
   Brown : constant Distortion_Coefficients :=
     (K1 => -0.12, K2 => 0.03, P1 => 0.001, P2 => -0.0015, K3 => 0.004);
   First_Pixels : constant Image_Point_Array :=
     Project_Points ([World_Point], Intrinsics, Brown, First);
   --  Analytic Rz(pi/3), independent of the production point transform.
   Analytic_First : constant Object_Point :=
     [0.5 * World_Point (0) - Math.Sqrt (3.0) / 2.0 * World_Point (1) + 1.0,
      Math.Sqrt (3.0) / 2.0 * World_Point (0) + 0.5 * World_Point (1),
      World_Point (2)];
   First_Frame_Pixel : constant Image_Point := Project_Oracle (Analytic_First, Intrinsics, Brown);
   Direct_Pixels : constant Image_Point_Array :=
     Project_Points ([World_Point], Intrinsics, Brown, Second);
   Relative_Pixels : constant Image_Point_Array :=
     Project_Points ([Through_Relative], Intrinsics, Brown,
                     (Rotation => [0.0, 0.0, 0.0], Translation => [0.0, 0.0, 0.0]));
   Worst : OpenCV.Float64_Value := 0.0;

   procedure Put_Vector (Name : String; Value : Object_Point) is
   begin
      Put_Line (Name & " = (" & OpenCV.Float64_Value'Image (Value (0)) & ","
                & OpenCV.Float64_Value'Image (Value (1)) & ","
                & OpenCV.Float64_Value'Image (Value (2)) & ")");
   end Put_Vector;
begin
   Put_Line ("Two known world-to-camera poses in one caller Cartesian frame.");
   Put_Line ("These poses were not estimated and are not camera measurements.");
   Put_Vector ("first rotation vector", Object_Point (First.Rotation));
   Put_Vector ("first translation t", Object_Point (First.Translation));
   Put_Vector ("first camera center", Motion.First_Center_World);
   Put_Vector ("second rotation vector", Object_Point (Second.Rotation));
   Put_Vector ("second translation t", Object_Point (Second.Translation));
   Put_Vector ("second camera center", Motion.Second_Center_World);
   Put_Line ("Relative transform maps first-camera coordinates to second-camera coordinates.");
   Put_Vector ("relative rotation vector", Object_Point (Relative.Rotation));
   Put_Vector ("relative translation in second camera", Object_Point (Relative.Translation));
   Put_Vector ("world displacement C2-C1", Motion.Displacement_World);
   Put_Line ("baseline length:" & OpenCV.Float64_Value'Image (Motion.Baseline_Distance));
   Put_Line ("baseline direction defined: " & Boolean'Image (Motion.Baseline_Direction_Defined));
   if Motion.Baseline_Direction_Defined then
      Put_Vector ("baseline direction", Motion.Baseline_Direction_World);
   end if;
   Put_Vector ("world point in first camera", In_First);
   Put_Vector ("point through relative transform", Through_Relative);
   Put_Vector ("world point in second camera", Direct_Second);
   for Axis in 0 .. 2 loop
      Worst := OpenCV.Float64_Value'Max
        (Worst, abs (Through_Relative (Axis) - Direct_Second (Axis)));
   end loop;
   for Axis in 0 .. 1 loop
      Worst := OpenCV.Float64_Value'Max
        (Worst, abs (First_Pixels (First_Pixels'First) (Axis)
                     - First_Frame_Pixel (Axis)));
      Worst := OpenCV.Float64_Value'Max
        (Worst, abs (Direct_Pixels (Direct_Pixels'First) (Axis)
                     - Relative_Pixels (Relative_Pixels'First) (Axis)));
   end loop;
   Put_Line ("maximum transform/projection difference:" & OpenCV.Float64_Value'Image (Worst));
   if Worst > 1.0E-8 then
      raise Program_Error with "relative transform disagrees with the second camera";
   end if;
   Put_Line ("t is not a camera position. Relative translation is not C2-C1.");
   Put_Line ("Baseline uses the shared caller world-frame length unit.");
   Put_Line ("No Earth frame, geographic conversion, or navigation uncertainty is implied.");
   Put_Line ("The same comparison applies to poses from Solve_PnP_RANSAC,");
   Put_Line ("Select_AP3P_Pose, or Select_Refine_AP3P_Pose without using those estimators.");
end Relative_Motion_Synthetic;
