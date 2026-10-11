with Ada.Text_IO;
with Ada.Numerics.Generic_Elementary_Functions;
with OpenCV.Calib3D;

procedure Homography_Synthetic is
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   package Math is new Ada.Numerics.Generic_Elementary_Functions (OpenCV.Float64_Value);
   Source, Destination : Image_Point_Array (1 .. 24);
   Sample : constant Image_Point := [35.0, 25.0];
   procedure Demonstrate_Planar (Pure : Boolean) is
      K : constant Camera_Intrinsics := (800.0, 820.0, 640.0, 360.0);
      type Matrix is array (Natural range 0 .. 2, Natural range 0 .. 2) of OpenCV.Float64_Value;
      Camera : constant Matrix := [[800.0, 0.0, 640.0], [0.0, 820.0, 360.0], [0.0, 0.0, 1.0]];
      Inverse : constant Matrix := [[1.0 / 800.0, 0.0, -0.8],
        [0.0, 1.0 / 820.0, -360.0 / 820.0], [0.0, 0.0, 1.0]];
      function Product (A, B : Matrix) return Matrix is
         C : Matrix := [others => [others => 0.0]];
      begin
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               for L in 0 .. 2 loop
                  C (I, J) := C (I, J) + A (I, L) * B (L, J);
               end loop;
            end loop;
         end loop;
         return C;
      end Product;
      procedure Print (Label : String; M : Matrix) is
      begin
         Ada.Text_IO.Put_Line (Label);
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               Ada.Text_IO.Put (OpenCV.Float64_Value'Image (M (I, J)) & " ");
            end loop;
            Ada.Text_IO.New_Line;
         end loop;
      end Print;
      Angle : constant OpenCV.Float64_Value := (if Pure then 0.12 else 0.08);
      C : constant OpenCV.Float64_Value := Math.Cos (Angle);
      S : constant OpenCV.Float64_Value := Math.Sin (Angle);
      Normalized : constant Matrix := (if Pure then [[C, -S, 0.0], [S, C, 0.0], [0.0, 0.0, 1.0]]
        else [[C, 0.0, S - 0.12], [0.0, 1.0, 0.04], [-S, 0.0, C + 0.08]]);
      H : constant Homography_Matrix := Homography_Matrix (Product (Product (Camera, Normalized), Inverse));
      Candidates : constant Planar_Motion_Hypothesis_Array := Decompose_Calibrated_Homography (H, K);
      First : constant Normalized_Image_Point_Array := [[0.1, 0.1], [0.2, 0.1], [-100.0, 0.0]];
      Second : constant Normalized_Image_Point_Array := [[0.08, 0.13], [0.17, 0.13], [-100.0, 0.0]];
      Visibility : constant Planar_Hypothesis_Visibility_Array :=
        Assess_Planar_Visibility (Candidates, First, Second, [1, 2]);
      All_Visibility : constant Planar_Hypothesis_Visibility_Array :=
        Assess_Planar_Visibility (Candidates, First, Second);
      Candidate_Index : Natural := 0;
   begin
      Print ("Shared intrinsics:", Camera);
      declare
         Input : Matrix;
      begin
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               Input (I, J) := H (I, J);
            end loop;
         end loop;
         Print ("Input homography:", Input);
      end;
      Ada.Text_IO.Put_Line ("Hypothesis count:" & Natural'Image (Candidates'Length));
      for V of Candidates loop
         Candidate_Index := Candidate_Index + 1;
         Ada.Text_IO.Put_Line ("Candidate index:" & Natural'Image (Candidate_Index));
         Ada.Text_IO.Put_Line ("Selected visibility: " &
           Planar_Visibility_Disposition'Image (Visibility (Candidate_Index).Disposition));
         Ada.Text_IO.Put_Line ("All-reference visibility: " &
           Planar_Visibility_Disposition'Image (All_Visibility (Candidate_Index).Disposition));
         declare
            Rotation : Matrix;
         begin
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  Rotation (I, J) := V.Rotation_First_To_Second (I, J);
               end loop;
            end loop;
            Print ("Candidate first-to-second rotation:", Rotation);
         end;
         Ada.Text_IO.Put_Line ("Translation over UNKNOWN plane distance (not metric translation):");
         for X of V.Translation_Over_Plane_Distance loop
            Ada.Text_IO.Put (OpenCV.Float64_Value'Image (X) & " ");
         end loop;
         Ada.Text_IO.New_Line;
         Ada.Text_IO.Put_Line ("Plane normal in first camera:");
         for X of V.Plane_Normal_In_First loop
            Ada.Text_IO.Put (OpenCV.Float64_Value'Image (X) & " ");
         end loop;
         Ada.Text_IO.New_Line;
         Ada.Text_IO.Put_Line ("Pure rotation: " & Boolean'Image (V.Pure_Rotation));
         declare
            Implied : Matrix;
            Dot, Norm, Factor, Error : OpenCV.Float64_Value := 0.0;
         begin
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  Implied (I, J) := V.Rotation_First_To_Second (I, J) +
                    V.Translation_Over_Plane_Distance (I) * V.Plane_Normal_In_First (J);
                  Dot := Dot + Implied (I, J) * Normalized (I, J);
                  Norm := Norm + Implied (I, J)**2;
               end loop;
            end loop;
            Factor := Dot / Norm;
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  Error := OpenCV.Float64_Value'Max (Error, abs (Factor * Implied (I, J) - Normalized (I, J)));
               end loop;
            end loop;
            Ada.Text_IO.Put_Line ("Projective reconstruction error:" & OpenCV.Float64_Value'Image (Error));
            if Error > 1.0E-10 then
               raise Program_Error with "planar reconstruction failed";
            end if;
         end;
      end loop;
      Ada.Text_IO.Put_Line ("Alternative mathematical hypotheses; none automatically selected as physical motion.");
      Ada.Text_IO.Put_Line ("Supplied references: 3; selected references: 2 (contradictory third excluded)");
      Ada.Text_IO.Put ("Passing candidate indices:");
      for Index of Visibility_Passing_Hypothesis_Indices (Visibility) loop
         Ada.Text_IO.Put (Positive'Image (Index));
      end loop;
      Ada.Text_IO.New_Line;
      Ada.Text_IO.Put_Line ("Visibility filtering applies source-defined plane-normal sign constraints.");
      Ada.Text_IO.Put_Line ("It is not a full cheirality proof. More than one motion hypothesis may remain.");
      Ada.Text_IO.Put_Line ("No metric translation or navigation position is recovered.");
   end Demonstrate_Planar;

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
   Demonstrate_Planar (True);
   Demonstrate_Planar (False);
end Homography_Synthetic;