with Ada.Text_IO;
with Ada.Numerics.Generic_Elementary_Functions;
with OpenCV.Calib3D;

procedure Homography_Synthetic is
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   package Math is new Ada.Numerics.Generic_Elementary_Functions (OpenCV.Float64_Value);
   Source, Destination : Image_Point_Array (1 .. 24);
   Sample : constant Image_Point := [35.0, 25.0];
   function Truth (P : Image_Point) return Image_Point is
      W : constant OpenCV.Float64_Value := 0.001 * P (0) - 0.002 * P (1) + 1.0;
   begin
      return [(1.2 * P (0) + 0.1 * P (1) + 20.0) / W,
              (-0.05 * P (0) + 0.9 * P (1) + 10.0) / W];
   end Truth;
begin
   Ada.Text_IO.Put_Line ("Planar/projective correspondence verification, not camera pose or terrain localization.");
   for I in Source'Range loop
      Source (I) := [OpenCV.Float64_Value ((I - 1) mod 6) * 40.0 - 100.0,
                     OpenCV.Float64_Value ((I - 1) / 6) * 35.0 - 50.0];
      Destination (I) := Truth (Source (I));
      if I = 2 or else I = 7 or else I = 15 then
         Destination (I) := [Destination (I) (0) + 5_000.0, Destination (I) (1) - 4_000.0];
      end if;
   end loop;
   declare
      Estimate : constant Homography_Estimate := Estimate_Homography_RANSAC
        (Source, Destination, (2_000, 0.1, 0.999));
   begin
      Ada.Text_IO.Put_Line ("Correspondence count:" & Natural'Image (Source'Length));
      Ada.Text_IO.Put_Line ("Found: " & Boolean'Image (Found (Estimate)));
      Ada.Text_IO.Put_Line ("Final-model inlier count:" & Natural'Image (Inlier_Count (Estimate)));
      if not Found (Estimate) then
         raise Program_Error with "synthetic homography not found";
      end if;
      declare
         H : constant Homography_Matrix := Homography (Estimate);
         M : constant Homography_Point_Result := Map_With_Homography (H, Sample);
         Expected : constant Image_Point := Truth (Sample);
         Error : OpenCV.Float64_Value;
      begin
         Ada.Text_IO.Put_Line ("Homography matrix (scale ambiguous):");
         for Row in 0 .. 2 loop
            for Col in 0 .. 2 loop
               Ada.Text_IO.Put (OpenCV.Float64_Value'Image (H (Row, Col)) & " ");
            end loop;
            Ada.Text_IO.New_Line;
         end loop;
         for I in Source'Range loop
            if I = 2 or else I = 7 or else I = 15 then
               declare
                  Survived : Boolean := False;
               begin
                  for Index of Inliers (Estimate) loop
                     Survived := Survived or else Index = I;
                  end loop;
                  Ada.Text_IO.Put_Line ("Known gross-outlier" & Positive'Image (I) & " survived: " & Boolean'Image (Survived));
                  if Survived then
                     raise Program_Error with "gross outlier survived";
                  end if;
               end;
            end if;
         end loop;
         if not M.Finite then
            raise Program_Error with "sample maps to infinity";
         end if;
         Error := Math.Sqrt ((M.Point (0) - Expected (0))**2 + (M.Point (1) - Expected (1))**2);
         Ada.Text_IO.Put_Line ("Mapped sample point: (" & OpenCV.Float64_Value'Image (M.Point (0)) &
           "," & OpenCV.Float64_Value'Image (M.Point (1)) & ")");
         Ada.Text_IO.Put_Line ("Sample mapping error:" & OpenCV.Float64_Value'Image (Error));
         if Error >= 1.0E-3 then
            raise Program_Error with "sample mapping error too large";
         end if;
      end;
   end;
end Homography_Synthetic;