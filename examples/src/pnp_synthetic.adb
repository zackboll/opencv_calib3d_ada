with Ada.Text_IO;
with Ada.Numerics.Generic_Elementary_Functions;
with OpenCV.Calib3D;

procedure PnP_Synthetic is
   use Ada.Text_IO;
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   package Math is new Ada.Numerics.Generic_Elementary_Functions (OpenCV.Float64_Value);

   Intrinsics : constant Camera_Intrinsics :=
     (Focal_X => 800.0, Focal_Y => 820.0, Center_X => 320.0, Center_Y => 240.0);
   Truth : constant World_To_Camera_Pose :=
     (Rotation => [0 => 0.10, 1 => -0.05, 2 => 0.08],
      Translation => [0 => 0.20, 1 => -0.10, 2 => 6.00]);
   Points : constant Object_Point_Array :=
     [[0 => -1.0, 1 => -1.0, 2 => 0.0], [0 => 0.0, 1 => -1.0, 2 => 0.3],
      [0 => 1.0, 1 => -1.0, 2 => 0.6], [0 => -1.2, 1 => 0.0, 2 => 0.4],
      [0 => 0.0, 1 => 0.0, 2 => 0.9], [0 => 1.2, 1 => 0.0, 2 => 0.2],
      [0 => -1.0, 1 => 1.0, 2 => 1.0], [0 => 0.0, 1 => 1.0, 2 => 1.4],
      [0 => 1.0, 1 => 1.0, 2 => 0.8], [0 => -0.5, 1 => -0.4, 2 => 1.7],
      [0 => 0.6, 1 => -0.3, 2 => 1.9], [0 => 0.3, 1 => 0.7, 2 => 2.1],
      [0 => -1.4, 1 => 0.6, 2 => 1.5], [0 => 1.5, 1 => 0.5, 2 => 1.2],
      [0 => -0.8, 1 => -1.4, 2 => 1.1], [0 => 0.9, 1 => -1.3, 2 => 1.6],
      [0 => -1.5, 1 => -0.5, 2 => 2.0], [0 => 1.4, 1 => -0.6, 2 => 2.2],
      [0 => -0.2, 1 => 1.5, 2 => 1.8], [0 => 0.8, 1 => 1.4, 2 => 2.4]];
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
            Reprojected : constant Image_Point_Array :=
              Project_Points (Points, Intrinsics, No_Distortion, P);
            Max_Error : OpenCV.Float64_Value := 0.0;
            Sum_Squared : OpenCV.Float64_Value := 0.0;
         begin
            Put_Vector ("rvec", P.Rotation);
            Put_Vector ("tvec", P.Translation);
            Put_Vector ("camera center", Camera_Center (P));
            for I of Inliers (Estimate) loop
               declare
                  DX : constant OpenCV.Float64_Value := Reprojected (I) (0) - Pixels (I) (0);
                  DY : constant OpenCV.Float64_Value := Reprojected (I) (1) - Pixels (I) (1);
                  Squared : constant OpenCV.Float64_Value := DX * DX + DY * DY;
               begin
                  Sum_Squared := Sum_Squared + Squared;
                  Max_Error := OpenCV.Float64_Value'Max (Max_Error, Math.Sqrt (Squared));
               end;
            end loop;
            Put_Line ("inlier reprojection RMS pixels:" & OpenCV.Float64_Value'Image
              (Math.Sqrt (Sum_Squared / OpenCV.Float64_Value (Inlier_Count (Estimate)))));
            Put_Line ("inlier reprojection max pixels:" & OpenCV.Float64_Value'Image (Max_Error));
         end;
      end if;
   end;
end PnP_Synthetic;
