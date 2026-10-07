with Ada.Text_IO;
with OpenCV.Calib3D;

procedure Fundamental_Synthetic is
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   First, Second : Image_Point_Array (1 .. 32);
begin
   Ada.Text_IO.Put_Line ("Fundamental matrix constrains two-view epipolar geometry.");
   Ada.Text_IO.Put_Line ("It does not itself recover metric scale, camera position, or 3-D terrain location.");
   for I in First'Range loop
      declare
         J : constant Natural := I - 1;
         X : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((J*7) mod 17 - 8)*0.24;
         Y : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((J*11) mod 19 - 9)*0.19;
         Z : constant OpenCV.Float64_Value := 4.0 + OpenCV.Float64_Value ((J*13) mod 23)*0.17;
      begin
         --  Direct pinhole arithmetic, not the Calib3D projection binding.
         First (I) := [800.0*X/Z+320.0, 820.0*Y/Z+240.0];
         Second (I) := [800.0*(X-0.75)/Z+320.0, 820.0*Y/Z+240.0];
         if I = 2 or else I = 7 or else I = 15 then
            Second (I) := [Second (I) (0) + 50.0, Second (I) (1) + 500.0];
         end if;
      end;
   end loop;
   declare
      Estimate : constant Fundamental_Estimate := Estimate_Fundamental_RANSAC (First, Second, (0.1, 0.999));
   begin
      Ada.Text_IO.Put_Line ("Correspondence count:" & Natural'Image (First'Length));
      Ada.Text_IO.Put_Line ("Found: " & Boolean'Image (Found (Estimate)));
      Ada.Text_IO.Put_Line ("Final-model inlier count:" & Natural'Image (Inlier_Count (Estimate)));
      if not Found (Estimate) then
         raise Program_Error with "synthetic fundamental not found";
      end if;
      declare
         F : constant Fundamental_Matrix := Fundamental (Estimate);
         Maximum : OpenCV.Float64_Value := 0.0;
         Sample : constant Epipolar_Error_Result := Maximum_Epipolar_Error (F, First (1), Second (1));
      begin
         Ada.Text_IO.Put_Line ("Fundamental matrix (scale/sign ambiguous):");
         for Row in 0 .. 2 loop
            for Col in 0 .. 2 loop
               Ada.Text_IO.Put (OpenCV.Float64_Value'Image (F (Row, Col)) & " ");
            end loop;
            Ada.Text_IO.New_Line;
         end loop;
         for I in First'Range loop
            if I = 2 or else I = 7 or else I = 15 then
               declare
                  Survived : Boolean := False;
               begin
                  for Index of Inliers (Estimate) loop
                     Survived := Survived or else Index = I;
                  end loop;
                  Ada.Text_IO.Put_Line ("Known gross-outlier" & Positive'Image (I) & " survived: " & Boolean'Image (Survived));
                  if Survived then
                     raise Program_Error with "gross vertical outlier survived";
                  end if;
               end;
            end if;
         end loop;
         for I of Inliers (Estimate) loop
            declare
               E : constant Epipolar_Error_Result := Maximum_Epipolar_Error (F, First (I), Second (I));
            begin
               if not E.Defined or else E.Maximum_Error_Pixels > 0.1 then
                  raise Program_Error with "accepted pair fails final-model threshold";
               end if;
               Maximum := OpenCV.Float64_Value'Max (Maximum, E.Maximum_Error_Pixels);
            end;
         end loop;
         Ada.Text_IO.Put_Line ("Maximum accepted public epipolar error:" & OpenCV.Float64_Value'Image (Maximum));
         Ada.Text_IO.Put_Line ("Sample first: (" & OpenCV.Float64_Value'Image (First (1) (0)) &
           "," & OpenCV.Float64_Value'Image (First (1) (1)) & ")");
         Ada.Text_IO.Put_Line ("Sample second: (" & OpenCV.Float64_Value'Image (Second (1) (0)) &
           "," & OpenCV.Float64_Value'Image (Second (1) (1)) & ")");
         if not Sample.Defined then
            raise Program_Error with "sample epipolar error undefined";
         end if;
         Ada.Text_IO.Put_Line ("Sample epipolar error:" & OpenCV.Float64_Value'Image (Sample.Maximum_Error_Pixels));
      end;
   end;
end Fundamental_Synthetic;
