with Ada.Text_IO;
with OpenCV.Calib3D;

procedure PnP_Synthetic is
   use Ada.Text_IO;
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;

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
            Accepted : constant Inlier_Index_Array := Inliers (Estimate);
            Objects : Object_Point_Array (1 .. Accepted'Length);
            Images : Image_Point_Array (1 .. Accepted'Length);
            P : World_To_Camera_Pose := Pose (Estimate);
            Refined : Boolean;
         begin
            for I in Accepted'Range loop
               Objects (I) := Points (Accepted (I));
               Images (I) := Pixels (Accepted (I));
            end loop;
            declare
               Before : constant Reprojection_Summary := Summarize_Reprojection
                 (Reprojection_Errors (Objects, Images, Intrinsics, No_Distortion, P));
            begin
               Put_Line ("RANSAC inlier count:" & Natural'Image (Before.Count));
               Put_Line ("RANSAC RMS pixels:" & OpenCV.Float64_Value'Image (Before.RMS_Error_Pixels));
               Put_Line ("RANSAC max error pixels:" & OpenCV.Float64_Value'Image (Before.Maximum_Error_Pixels));
            end;
            Refine_Pose_Iterative (Objects, Images, Intrinsics, Pose => P, Refined => Refined);
            declare
               After : constant Reprojection_Summary := Summarize_Reprojection
                 (Reprojection_Errors (Objects, Images, Intrinsics, No_Distortion, P));
            begin
               Put_Line ("refinement succeeded:" & Boolean'Image (Refined));
               Put_Line ("refined RMS pixels:" & OpenCV.Float64_Value'Image (After.RMS_Error_Pixels));
               Put_Line ("refined max error pixels:" & OpenCV.Float64_Value'Image (After.Maximum_Error_Pixels));
            end;
            Put_Vector ("refined rotation vector", P.Rotation);
            Put_Vector ("refined translation vector", P.Translation);
            Put_Vector ("refined camera center", Camera_Center (P));
            Put_Line ("X_camera = R * X_world + t; Translation is t, NOT camera position.");
            Put_Line ("C_world = -R^T * t; low pixel error is not navigation accuracy.");
            declare
               Samples : constant Image_Point_Array :=
                 [[Intrinsics.Center_X, Intrinsics.Center_Y],
                  [Intrinsics.Center_X + Intrinsics.Focal_X, Intrinsics.Center_Y]];
               Normalized : constant Normalized_Image_Point_Array :=
                 Undistort_To_Normalized (Samples, Intrinsics);
               Camera : constant Camera_Direction_Array := Camera_Bearing_Rays (Samples, Intrinsics);
               Rays : constant World_Ray_Array :=
                 World_Bearing_Rays (Samples, Intrinsics, No_Distortion, P);
            begin
               for I in Samples'Range loop
                  Put_Line ("pixel (pixels) = (" & OpenCV.Float64_Value'Image (Samples (I) (0)) & "," &
                    OpenCV.Float64_Value'Image (Samples (I) (1)) & ")");
                  Put_Line ("normalized (dimensionless) = (" &
                    OpenCV.Float64_Value'Image (Normalized (I) (0)) & "," &
                    OpenCV.Float64_Value'Image (Normalized (I) (1)) & ")");
                  Put_Vector ("camera unit direction", Object_Point (Camera (I)));
                  Put_Vector ("world ray origin", Rays (I).Origin);
                  Put_Vector ("world unit direction", Object_Point (Rays (I).Direction));
               end loop;
               Put_Line ("Principal camera bearing is approximately (0,0,1).");
               Put_Line ("World rays are metric-frame geometric rays only; no DTED/geodetic interpretation is performed.");
               Put_Line ("X_world(s) = C_world + s * direction, s > 0; s uses caller world-coordinate units.");
            end;
         end;
      end if;
   end;
end PnP_Synthetic;
