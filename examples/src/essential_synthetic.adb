with Ada.Numerics.Generic_Elementary_Functions;
with Ada.Text_IO;
with OpenCV.Calib3D;

procedure Essential_Synthetic is
   use Ada.Text_IO;
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   package Math is new Ada.Numerics.Generic_Elementary_Functions (OpenCV.Float64_Value);
   First, Second : Normalized_Image_Point_Array (1 .. 40);
   Truth : constant Rotation_Matrix :=
     [[Math.Cos (0.08), 0.0, Math.Sin (0.08)], [0.0, 1.0, 0.0],
      [-Math.Sin (0.08), 0.0, Math.Cos (0.08)]];
   Translation : constant Camera_Direction := [-0.75, 0.08, 0.12];
   Norm : constant OpenCV.Float64_Value := Math.Sqrt (0.75**2 + 0.08**2 + 0.12**2);
   Options : constant Essential_RANSAC_Options := (1.0E-6, 0.999);
   function Contains (Indices : Inlier_Index_Array; Index : Positive) return Boolean is
   begin
      for I of Indices loop
         if I = Index then
            return True;
         end if;
      end loop;
      return False;
   end Contains;
begin
   --  Independent direct pinhole divisions; no projection binding or native
   --  projection is used to construct this file-free calibrated fixture.
   for I in First'Range loop
      declare
         J : constant Integer := I - 1;
         X : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((J * 7) mod 17 - 8) * 0.24;
         Y : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((J * 11) mod 19 - 9) * 0.19;
         Z : constant OpenCV.Float64_Value := 4.0 + OpenCV.Float64_Value ((J * 13) mod 23) * 0.17;
         X2 : constant OpenCV.Float64_Value := Truth (0, 0) * X + Truth (0, 2) * Z + Translation (0);
         Z2 : constant OpenCV.Float64_Value := Truth (2, 0) * X + Truth (2, 2) * Z + Translation (2);
      begin
         if Z2 <= 0.0 then
            raise Program_Error with "fixture second depth is not positive";
         end if;
         First (I) := [X / Z, Y / Z];
         Second (I) := [X2 / Z2, (Y + Translation (1)) / Z2];
      end;
   end loop;
   for I of Inlier_Index_Array'[2, 7, 15] loop
      Second (I) (0) := Second (I) (0) + 2.0;
      Second (I) (1) := Second (I) (1) - 1.5;
   end loop;
   Put_Line ("Calibrated two-view relative geometry, not metric navigation position.");
   Put_Line ("Translation magnitude is unknown. Translation term direction is NOT camera position.");
   Put_Line ("Correspondence count:" & Natural'Image (First'Length));
   declare
      Estimate : constant Essential_Estimate := Estimate_Essential_RANSAC (First, Second, Options);
   begin
      Put_Line ("Essential Found: " & Boolean'Image (Found (Estimate)));
      Put_Line ("Essential inlier count:" & Natural'Image (Inlier_Count (Estimate)));
      Put_Line ("Pose_Recovered: " & Boolean'Image (Pose_Recovered (Estimate)));
      Put_Line ("Pose inlier count:" & Natural'Image (Pose_Inlier_Count (Estimate)));
      if not Found (Estimate) or else not Pose_Recovered (Estimate) then
         raise Program_Error with "synthetic Essential/relative pose not recovered";
      end if;
      declare
         E : constant Essential_Matrix := Essential (Estimate);
         Pose : constant Relative_Camera_Pose := Recovered_Pose (Estimate);
         R : constant Rotation_Matrix := Pose.Rotation_First_To_Second;
         Center : constant Camera_Direction := Second_Camera_Center_Direction_In_First (Pose);
         Maximum, Trace, Dot, Center_Error : OpenCV.Float64_Value := 0.0;
         Angle : OpenCV.Float64_Value;
      begin
         Put_Line ("Essential matrix (arbitrary nonzero scale/sign):");
         for Row in 0 .. 2 loop
            for Col in 0 .. 2 loop
               Put (OpenCV.Float64_Value'Image (E (Row, Col)) & " ");
            end loop;
            New_Line;
         end loop;
         Put_Line ("Recovered R first->second:");
         for Row in 0 .. 2 loop
            for Col in 0 .. 2 loop
               Put (OpenCV.Float64_Value'Image (R (Row, Col)) & " ");
               Trace := Trace + R (Row, Col) * Truth (Row, Col);
            end loop;
            New_Line;
         end loop;
         Put_Line ("Recovered unit translation TERM direction:");
         for Axis in 0 .. 2 loop
            Put (OpenCV.Float64_Value'Image (Pose.Translation_Direction (Axis)) & " ");
            Dot := Dot + Pose.Translation_Direction (Axis) * Translation (Axis) / Norm;
         end loop;
         New_Line;
         Put_Line ("Recovered second-camera-center direction in first frame (-R^T*t_hat):");
         for Axis in 0 .. 2 loop
            Put (OpenCV.Float64_Value'Image (Center (Axis)) & " ");
            declare
               Expected : OpenCV.Float64_Value := 0.0;
            begin
               for J in 0 .. 2 loop
                  Expected := Expected - Truth (J, Axis) * Translation (J) / Norm;
               end loop;
               Center_Error := Center_Error + (Center (Axis) - Expected)**2;
            end;
         end loop;
         New_Line;
         for I of Inliers (Estimate) loop
            declare
               Error : constant Normalized_Sampson_Error_Result := Normalized_Sampson_Error (E, First (I), Second (I));
            begin
               if not Error.Defined or else Error.Error > Options.Normalized_Epipolar_Threshold then
                  raise Program_Error with "published inlier violates final-E Sampson threshold";
               end if;
               Maximum := OpenCV.Float64_Value'Max (Maximum, Error.Error);
            end;
         end loop;
         for I of Inlier_Index_Array'[2, 7, 15] loop
            Put_Line ("Known gross outlier" & Positive'Image (I) & " Essential survives=" &
              Boolean'Image (Contains (Inliers (Estimate), I)) & " Pose survives=" &
              Boolean'Image (Contains (Pose_Inliers (Estimate), I)));
            if Contains (Inliers (Estimate), I) or else Contains (Pose_Inliers (Estimate), I) then
               raise Program_Error with "known gross outlier survived";
            end if;
         end loop;
         Angle := Math.Arccos (OpenCV.Float64_Value'Max (-1.0, OpenCV.Float64_Value'Min (1.0, (Trace - 1.0) / 2.0)));
         Put_Line ("Maximum accepted normalized Sampson error: " & OpenCV.Float64_Value'Image (Maximum));
         Put_Line ("Rotation angular error (radians): " & OpenCV.Float64_Value'Image (Angle));
         Put_Line ("Translation-direction signed dot: " & OpenCV.Float64_Value'Image (Dot));
         Put_Line ("Camera-center direction error: " & OpenCV.Float64_Value'Image (Math.Sqrt (Center_Error)));
         if Angle >= 1.0E-5 or else Dot <= 1.0 - 1.0E-8 or else Math.Sqrt (Center_Error) >= 1.0E-5 then
            raise Program_Error with "relative frame/sign oracle failed";
         end if;
      end;
   end;
end Essential_Synthetic;