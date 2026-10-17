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
             declare
                Landmarks : constant Object_Point_Array :=
                  [[0.3, -0.2, 3.5], [1.4, 0.1, 4.0], [-0.5, 1.2, 5.3]];
                Fourth : constant Object_Point := [0.7, -0.8, 6.1];
                Observations : constant Image_Point_Array :=
                  Project_Points (Landmarks, Intrinsics, No_Distortion, Truth);
                Reference : constant Image_Point_Array :=
                  Project_Points ([Fourth], Intrinsics, No_Distortion, Truth);
                Hypotheses : constant AP3P_Pose_Hypothesis_Array :=
                  Solve_AP3P_Hypotheses (Landmarks, Observations, Intrinsics);
             begin
                Put_Line ("AP3P correspondences: 3");
                Put_Line ("AP3P hypotheses:" & Natural'Image (Hypotheses'Length));
                for I in Hypotheses'Range loop
                   declare
                      Candidate : constant World_To_Camera_Pose := Hypotheses (I).Pose;
                      Three : constant Reprojection_Summary := Summarize_Reprojection
                        (Reprojection_Errors (Landmarks, Observations, Intrinsics, No_Distortion, Candidate));
                      Extra : constant Reprojection_Error_Array :=
                        Reprojection_Errors ([Fourth], Reference, Intrinsics, No_Distortion, Candidate);
                   begin
                      Put_Line ("AP3P candidate" & Positive'Image (I));
                      Put_Vector ("candidate rotation vector", Candidate.Rotation);
                      Put_Vector ("candidate translation vector", Candidate.Translation);
                      Put_Vector ("candidate camera center", Camera_Center (Candidate));
                      Put_Line ("candidate three-point max pixels:" &
                        OpenCV.Float64_Value'Image (Three.Maximum_Error_Pixels));
                      Put_Line ("candidate independent fourth-point pixels:" &
                        OpenCV.Float64_Value'Image (Extra (Extra'First)));
                   end;
                end loop;
                Put_Line ("Three point correspondences may produce multiple mathematically valid poses.");
                Put_Line ("A fourth independent observation can help discriminate between hypotheses.");
                Put_Line ("No physical pose has been automatically selected by this binding.");
                Put_Line ("World-point units determine translation scale. No geographic reference is inferred.");
                 declare
                    Extra_World : constant Object_Point_Array :=
                      [[0.7, -0.8, 6.1], [1.1, 0.4, 5.2], [-0.9, -0.6, 4.4],
                       [0.2, 1.3, 7.0], [-1.5, 0.5, 3.8]];
                    Extra_Pixels : constant Image_Point_Array :=
                      Project_Points (Extra_World, Intrinsics, No_Distortion, Truth);
                    Diagnostics : constant AP3P_Reference_Result_Matrix :=
                      Evaluate_AP3P_References
                        (Hypotheses, Extra_World, Extra_Pixels, Intrinsics);
                    Summaries : constant AP3P_Hypothesis_Reference_Summary_Array :=
                      Summarize_AP3P_References (Diagnostics);
                 begin
                    Put_Line ("AP3P reference diagnostics hypotheses:" &
                      Natural'Image (Hypotheses'Length));
                    Put_Line ("AP3P additional references:" &
                      Natural'Image (Extra_World'Length));
                    for I in Summaries'Range loop
                       Put_Line ("AP3P diagnostic candidate" & Positive'Image (I));
                       Put_Line ("projected references:" &
                         Natural'Image (Summaries (I).Projected_References));
                       Put_Line ("non-positive-depth references:" &
                         Natural'Image (Summaries (I).Non_Positive_Depth_References));
                       Put_Line ("numerically unavailable references:" &
                         Natural'Image (Summaries (I).Numerically_Unavailable_References));
                       Put_Line ("RMS pixel error:" &
                         OpenCV.Float64_Value'Image (Summaries (I).Reprojection.RMS_Error_Pixels));
                       Put_Line ("maximum pixel error:" &
                         OpenCV.Float64_Value'Image
                           (Summaries (I).Reprojection.Maximum_Error_Pixels));
                       Put_Line ("entire set evaluable: " &
                         Boolean'Image (Summaries (I).Entire_Set_Evaluable));
                    end loop;
                    Put_Line ("These diagnostics compare AP3P hypotheses with additional observations.");
                    Put_Line ("A small reprojection error does not prove globally correct localization.");
                    Put_Line ("Pose translation uses the units of the supplied Cartesian world coordinates.");
                    declare
                       Strict : constant AP3P_Selection_Options := (1.0, 1.0, 1.0);
                       Wide : constant AP3P_Selection_Options := (1.0E6, 1.0E6, 1_000.0);
                       Unique : constant AP3P_Selection_Result :=
                         Select_AP3P_Hypothesis (Diagnostics, Strict);
                       Ambiguous : constant AP3P_Selection_Result :=
                         Select_AP3P_Hypothesis (Diagnostics, Wide);
                    begin
                       Put_Line ("Selection policy is caller-controlled; these thresholds are demonstration values.");
                       Put_Line ("Unique-policy disposition: " & AP3P_Selection_Disposition'Image (Unique.Disposition));
                       for I in Unique.Candidates'Range loop
                          Put_Line ("candidate" & Positive'Image (I)
                            & " eligible=" & Boolean'Image (Unique.Candidates (I).Eligible)
                            & " rank=" & Natural'Image (Unique.Candidates (I).Rank)
                            & " rms=" & OpenCV.Float64_Value'Image (Unique.Candidates (I).RMS_Error_Pixels));
                       end loop;
                       if Selected (Unique) then
                          declare
                             Index : constant Positive := Selected_Hypothesis_Index (Unique);
                             Pose : constant World_To_Camera_Pose :=
                               Selected_Pose (Unique, Hypotheses, Index);
                          begin
                             Put_Line ("Selected hypothesis position:" & Positive'Image (Index));
                             Put_Vector ("selected rotation vector", Pose.Rotation);
                             Put_Vector ("selected translation t", Pose.Translation);
                             Put_Vector ("selected camera center", Camera_Center (Pose));
                             Put_Line ("RMS separation pixels:"
                               & OpenCV.Float64_Value'Image (Unique.Separation_Pixels));
                          end;
                       end if;
                       Put_Line ("Wide-policy disposition: "
                         & AP3P_Selection_Disposition'Image (Ambiguous.Disposition));
                       Put_Line ("Wide-policy separation pixels:"
                         & OpenCV.Float64_Value'Image (Ambiguous.Separation_Pixels));
                       Put_Line ("This pose, when selected, is expressed in the supplied Cartesian world frame.");
                       Put_Line ("Translation is t in X_camera = R*X_world + t, not the camera position.");
                       Put_Line ("Selection is conditional on these references and this numerical policy.");
                       Put_Line ("It is not proof of globally correct camera localization.");
                    end;
                 end;

             end;

         end;
      end if;
   end;

   --  Task 014: native cv::composeRT coordinate-frame composition. This is NOT
   --  navigation fusion or geolocation; it chains two rigid frame transforms.
   declare
      Mount : constant World_To_Camera_Pose :=
        (Rotation => [0.05, -0.04, 0.10], Translation => [0.10, -0.05, 0.30]);
      Camera : constant World_To_Camera_Pose :=
        (Rotation => [-0.03, 0.06, 0.02], Translation => [-0.20, 0.10, 5.00]);
      Composed : constant World_To_Camera_Pose := Compose_Poses (Mount, Camera);
      Sample : constant Object_Point_Array :=
        [[0.4, -0.3, 1.0], [-0.8, 0.5, 0.2], [1.1, 0.9, 0.7], [-0.2, -0.9, 1.4]];
      Maximum_Difference : OpenCV.Float64_Value := 0.0;
      Projection_Error : OpenCV.Float64_Value := 0.0;
      Composed_Pixels : constant Image_Point_Array :=
        Project_Points (Sample, Intrinsics, No_Distortion, Composed);
      Identity_Pose : constant World_To_Camera_Pose := (others => <>);
   begin
      New_Line;
      Put_Line ("Pose composition: world -> intermediate mount frame -> final camera frame");
      Put_Line ("Compose_Poses (First, Second) applies First, then Second.");
      Put_Vector ("first rotation vector", Object_Point (Mount.Rotation));
      Put_Vector ("first translation vector", Object_Point (Mount.Translation));
      Put_Vector ("second rotation vector", Object_Point (Camera.Rotation));
      Put_Vector ("second translation vector", Object_Point (Camera.Translation));
      Put_Vector ("composed rotation vector", Object_Point (Composed.Rotation));
      Put_Vector ("composed translation vector", Object_Point (Composed.Translation));
      Put_Vector ("composed camera center", Camera_Center (Composed));
      for I in Sample'Range loop
         declare
            Sequential : constant Object_Point :=
              World_To_Camera_Point (Camera, World_To_Camera_Point (Mount, Sample (I)));
            Combined : constant Object_Point := World_To_Camera_Point (Composed, Sample (I));
            Reference : constant Image_Point_Array :=
              Project_Points ([Sequential], Intrinsics, No_Distortion, Identity_Pose);
         begin
            if I = Sample'First then
               Put_Vector ("sequential point transform", Sequential);
               Put_Vector ("composed point transform", Combined);
            end if;
            for Axis in 0 .. 2 loop
               Maximum_Difference := OpenCV.Float64_Value'Max
                 (Maximum_Difference, abs (Sequential (Axis) - Combined (Axis)));
            end loop;
            for Axis in 0 .. 1 loop
               Projection_Error := OpenCV.Float64_Value'Max
                 (Projection_Error, abs (Composed_Pixels (I) (Axis) - Reference (1) (Axis)));
            end loop;
         end;
      end loop;
      Put_Line ("maximum point-transform difference:" &
                OpenCV.Float64_Value'Image (Maximum_Difference));
      Put_Line ("sequential versus composed projection error (pixels):" &
                OpenCV.Float64_Value'Image (Projection_Error));
      Put_Line ("This is coordinate-frame composition, not navigation fusion or geolocation.");
      Put_Line ("Both translations must use the same Cartesian units.");
   end;
end PnP_Synthetic;
