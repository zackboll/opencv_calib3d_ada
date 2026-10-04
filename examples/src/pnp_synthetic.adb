with Ada.Text_IO;
with OpenCV.Calib3D;

procedure PnP_Synthetic is
   use Ada.Text_IO;
   use OpenCV.Calib3D;

   Intrinsics : constant Camera_Intrinsics :=
     (Focal_X => 800.0, Focal_Y => 820.0, Center_X => 320.0, Center_Y => 240.0);
   Truth : constant World_To_Camera_Pose :=
     (Rotation => (0 => 0.10, 1 => -0.05, 2 => 0.08),
      Translation => (0 => 0.20, 1 => -0.10, 2 => 6.00));
   Points : constant Object_Point_Array :=
     [(0 => -1.0, 1 => -1.0, 2 => 0.0), (0 => 0.0, 1 => -1.0, 2 => 0.3),
      (0 => 1.0, 1 => -1.0, 2 => 0.6), (0 => -1.2, 1 => 0.0, 2 => 0.4),
      (0 => 0.0, 1 => 0.0, 2 => 0.9), (0 => 1.2, 1 => 0.0, 2 => 0.2),
      (0 => -1.0, 1 => 1.0, 2 => 1.0), (0 => 0.0, 1 => 1.0, 2 => 1.4),
      (0 => 1.0, 1 => 1.0, 2 => 0.8), (0 => -0.5, 1 => -0.4, 2 => 1.7),
      (0 => 0.6, 1 => -0.3, 2 => 1.9), (0 => 0.3, 1 => 0.7, 2 => 2.1),
      (0 => -1.4, 1 => 0.6, 2 => 1.5), (0 => 1.5, 1 => 0.5, 2 => 1.2),
      (0 => -0.8, 1 => -1.4, 2 => 1.1), (0 => 0.9, 1 => -1.3, 2 => 1.6),
      (0 => -1.5, 1 => -0.5, 2 => 2.0), (0 => 1.4, 1 => -0.6, 2 => 2.2),
      (0 => -0.2, 1 => 1.5, 2 => 1.8), (0 => 0.8, 1 => 1.4, 2 => 2.4)];
   Pixels : Image_Point_Array := Project_Points (Points, Intrinsics, No_Distortion, Truth);

   procedure Put_Vector (Name : String; V : Object_Point) is
   begin
      Put_Line (Name & " = (" & OpenCV.Float64_Value'Image (V (0)) & "," &
                OpenCV.Float64_Value'Image (V (1)) & "," & OpenCV.Float64_Value'Image (V (2)) & ")");
   end Put_Vector;

begin
   --  Deliberately corrupt three correspondences. These values are synthetic
   --  fixture policy, not a navigation accuracy claim.
   declare
      type Index_List is array (Positive range <>) of Positive;
      Outliers : constant Index_List := [2, 7, 15];
   begin
      for I of Outliers loop
         Pixels (I) (0) := Pixels (I) (0) + 5_000.0;
         Pixels (I) (1) := Pixels (I) (1) - 4_000.0;
      end loop;
   end;

   declare
      Estimate : constant Pose_Estimate := Solve_PnP_RANSAC
        (Points, Pixels, Intrinsics, Options =>
           (Maximum_Iterations => 1_000, Reprojection_Error_Pixels => 2.0, Confidence => 0.999));
   begin
      Put_Line ("correspondences:" & Natural'Image (Points'Length));
      Put_Line ("found:" & Boolean'Image (Found (Estimate)));
      Put_Line ("inliers:" & Natural'Image (Inlier_Count (Estimate)));
      if Found (Estimate) then
         declare
            P : constant World_To_Camera_Pose := Pose (Estimate);
         begin
            Put_Vector ("rvec", P.Rotation);
            Put_Vector ("tvec", P.Translation);
            Put_Vector ("camera center", Camera_Center (P));
         end;
      end if;
   end;
end PnP_Synthetic;
