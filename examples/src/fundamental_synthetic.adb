with Ada.Exceptions;
with Ada.Numerics.Generic_Elementary_Functions;
with Ada.Text_IO;
with OpenCV.Calib3D;
with OpenCV.Core;
with OpenCV.Core.Float64_Access;

procedure Fundamental_Synthetic is
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   package Math is new Ada.Numerics.Generic_Elementary_Functions (OpenCV.Float64_Value);
   First, Second : Image_Point_Array (1 .. 32);

   --  EXAMPLE application policy, chosen from measured geometry in this
   --  program. It is not a library default and not an optimality guarantee.
   Example_Policy : constant Epipolar_Correction_Options :=
     (Minimum_Second_Singular_Ratio       => 1.0E-3,
      Maximum_Third_Singular_Ratio        => 1.0E-6,
      Minimum_Epipolar_Line_Ratio         => 1.0E-6,
      Maximum_Corrected_Error_Pixels      => 1.0E-6,
      Absolute_Displacement_Excess_Pixels => 1.0E-9,
      Relative_Displacement_Excess        => 1.0E-9);
   Accepted_Batches, Rejected_Batches : Natural := 0;

   procedure Report (Label : String; F : Fundamental_Matrix;
                     P1, P2 : Image_Point_Array; R : Corrected_Epipolar_Matches) is
      Max_In, Max_Out, Max_Disp, Sum_Sq : OpenCV.Float64_Value := 0.0;
   begin
      for I in 1 .. R.Count loop
         declare
            In_Error : constant Epipolar_Error_Result :=
              Maximum_Epipolar_Error (F, P1 (P1'First + I - 1), P2 (P2'First + I - 1));
         begin
            if In_Error.Defined then
               Max_In := OpenCV.Float64_Value'Max (Max_In, In_Error.Maximum_Error_Pixels);
            end if;
         end;
         Max_Out := OpenCV.Float64_Value'Max (Max_Out, R.Diagnostics (I).Corrected_Error_Pixels);
         Max_Disp := OpenCV.Float64_Value'Max (Max_Disp, R.Diagnostics (I).Total_Displacement_Pixels);
         Sum_Sq := Sum_Sq + R.Diagnostics (I).Total_Displacement_Pixels**2;
      end loop;
      Ada.Text_IO.Put_Line ("  " & Label & " ACCEPTED");
      Ada.Text_IO.Put_Line ("  Input correspondence count:" & Natural'Image (P1'Length));
      Ada.Text_IO.Put_Line ("  Corrected correspondence count:" & Natural'Image (R.Count));
      Ada.Text_IO.Put_Line ("  Selected thresholds: s2/s1 >=" &
        OpenCV.Float64_Value'Image (Example_Policy.Minimum_Second_Singular_Ratio) & ", s3/s1 <=" &
        OpenCV.Float64_Value'Image (Example_Policy.Maximum_Third_Singular_Ratio) & ", line ratio >=" &
        OpenCV.Float64_Value'Image (Example_Policy.Minimum_Epipolar_Line_Ratio));
      Ada.Text_IO.Put_Line ("  Maximum input epipolar error:" & OpenCV.Float64_Value'Image (Max_In));
      Ada.Text_IO.Put_Line ("  Maximum corrected epipolar error:" & OpenCV.Float64_Value'Image (Max_Out));
      if R.Count > 0 then
         Ada.Text_IO.Put_Line ("  RMS correction displacement:" & OpenCV.Float64_Value'Image
           (Math.Sqrt (Sum_Sq / OpenCV.Float64_Value (R.Count))));
      end if;
      Ada.Text_IO.Put_Line ("  Maximum correction displacement:" & OpenCV.Float64_Value'Image (Max_Disp));
   end Report;

   --  EXAMPLE-ONLY measurement through Core's existing qualified SVD of the
   --  max-|coefficient|-normalized F (the same quantities the guard checks).
   procedure Print_Conditioning (F : Fundamental_Matrix) is
      Scale : OpenCV.Float64_Value := 0.0;
      M : OpenCV.Core.Mat := OpenCV.Core.Create (3, 3, (Depth => OpenCV.Core.Float64, Channels => 1));
   begin
      for V of F loop
         Scale := OpenCV.Float64_Value'Max (Scale, abs V);
      end loop;
      for Row in 0 .. 2 loop
         for Col in 0 .. 2 loop
            OpenCV.Core.Float64_Access.Set (M, Row, Col, F (Row, Col) / Scale);
         end loop;
      end loop;
      declare
         S : constant OpenCV.Core.Singular_Value_Decomposition_Result :=
           OpenCV.Core.Singular_Value_Decomposition (M);
         S1 : constant OpenCV.Float64_Value := OpenCV.Core.Float64_Access.Get (S.Singular_Values, 0, 0);
         S2 : constant OpenCV.Float64_Value := OpenCV.Core.Float64_Access.Get (S.Singular_Values, 1, 0);
         S3 : constant OpenCV.Float64_Value := OpenCV.Core.Float64_Access.Get (S.Singular_Values, 2, 0);
      begin
         Ada.Text_IO.Put_Line ("  Input F conditioning (example-only Core SVD): s2/s1=" &
           OpenCV.Float64_Value'Image (S2 / S1) & " s3/s1=" & OpenCV.Float64_Value'Image (S3 / S1));
      end;
   end Print_Conditioning;

   procedure Attempt (Label : String; F : Fundamental_Matrix; P1, P2 : Image_Point_Array) is
   begin
      Ada.Text_IO.Put_Line (Label);
      Print_Conditioning (F);
      declare
         R : constant Corrected_Epipolar_Matches := Correct_Epipolar_Matches (F, P1, P2, Example_Policy);
      begin
         Accepted_Batches := Accepted_Batches + 1;
         Report (Label, F, P1, P2, R);
      end;
   exception
      when E : OpenCV.OpenCV_Error =>
         Rejected_Batches := Rejected_Batches + 1;
         Ada.Text_IO.Put_Line ("  REJECTED by guard (" &
           Ada.Exceptions.Exception_Message (E) & ")");
   end Attempt;

   procedure Guarded_Correction (F : Fundamental_Matrix; P1, P2 : Image_Point_Array;
                                 Selected : Inlier_Index_Array) is
      S1, S2 : Image_Point_Array (1 .. Selected'Length);
   begin
      for K in Selected'Range loop
         --  Small controlled perturbation of the selected RANSAC inliers.
         S1 (K) := [P1 (Selected (K)) (0) + 0.3, P1 (Selected (K)) (1) - 0.2];
         S2 (K) := [P2 (Selected (K)) (0) - 0.1, P2 (Selected (K)) (1) + 0.4];
      end loop;
      Attempt ("Guarded native correction after RANSAC (estimated F, selected inliers)", F, S1, S2);
   end Guarded_Correction;

   --  Nonidentity two-view geometry built independently: E=[t]x R and
   --  F=K^-T E K^-1 (K: 800/820, 320/240), R=Ry(0.08)*Rx(0.03), t=(-0.75,0.08,0.12).
   procedure Nonidentity_Stereo is
      type M3 is array (0 .. 2, 0 .. 2) of OpenCV.Float64_Value;
      function Mul (A, B : M3) return M3 is
      begin
         return C : M3 := [others => [others => 0.0]] do
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  for K in 0 .. 2 loop
                     C (I, J) := C (I, J) + A (I, K) * B (K, J);
                  end loop;
               end loop;
            end loop;
         end return;
      end Mul;
      R : constant M3 := Mul
        ([[Math.Cos (0.08), 0.0, Math.Sin (0.08)], [0.0, 1.0, 0.0], [-Math.Sin (0.08), 0.0, Math.Cos (0.08)]],
         [[1.0, 0.0, 0.0], [0.0, Math.Cos (0.03), -Math.Sin (0.03)], [0.0, Math.Sin (0.03), Math.Cos (0.03)]]);
      T : constant array (0 .. 2) of OpenCV.Float64_Value := [-0.75, 0.08, 0.12];
      Tx : constant M3 := [[0.0, -T (2), T (1)], [T (2), 0.0, -T (0)], [-T (1), T (0), 0.0]];
      K_Inv : constant M3 := [[1.0 / 800.0, 0.0, -0.4], [0.0, 1.0 / 820.0, -240.0 / 820.0], [0.0, 0.0, 1.0]];
      K_Inv_T : constant M3 := [[K_Inv (0, 0), 0.0, 0.0], [0.0, K_Inv (1, 1), 0.0], [K_Inv (0, 2), K_Inv (1, 2), 1.0]];
      FM : constant M3 := Mul (K_Inv_T, Mul (Mul (Tx, R), K_Inv));
      F : constant Fundamental_Matrix := [for I in 0 .. 2 => [for J in 0 .. 2 => FM (I, J)]];
      P1, P2 : Image_Point_Array (1 .. 12);
   begin
      for J in 0 .. 11 loop
         declare
            X : constant array (0 .. 2) of OpenCV.Float64_Value :=
              [OpenCV.Float64_Value ((J * 7) mod 17 - 8) * 0.24, OpenCV.Float64_Value ((J * 11) mod 19 - 9) * 0.19,
               4.0 + OpenCV.Float64_Value ((J * 13) mod 23) * 0.17];
            Y : array (0 .. 2) of OpenCV.Float64_Value := [T (0), T (1), T (2)];
         begin
            for I in 0 .. 2 loop
               for K in 0 .. 2 loop
                  Y (I) := Y (I) + R (I, K) * X (K);
               end loop;
            end loop;
            --  Direct pinhole projection plus a controlled 1-pixel-scale perturbation.
            P1 (J + 1) := [800.0 * X (0) / X (2) + 320.0 + 0.7, 820.0 * X (1) / X (2) + 240.0 - 0.4];
            P2 (J + 1) := [800.0 * Y (0) / Y (2) + 320.0 - 0.3, 820.0 * Y (1) / Y (2) + 240.0 + 0.9];
         end;
      end loop;
      Attempt ("Guarded native correction, nonidentity rotated stereo (independently constructed F)", F, P1, P2);
   end Nonidentity_Stereo;

   procedure Rectified_And_Rejected is
      Rect : constant Fundamental_Matrix := [[0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]];
      Rank_One : constant Fundamental_Matrix := [[1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]];
   begin
      Nonidentity_Stereo;
      Attempt ("Guarded native correction, rectified analytic case (expect (100,22)/(90,22))", Rect,
               [[100.0, 20.0], [300.0, 40.0]], [[90.0, 24.0], [250.0, 30.0]]);
      Ada.Text_IO.Put_Line ("Intentionally inadmissible rank-one F (an EXPECTED guarded outcome, " &
                            "not automatically a programming error):");
      Attempt ("Rank-one F", Rank_One, [[100.0, 20.0]], [[90.0, 24.0]]);
      Ada.Text_IO.Put_Line ("Accepted batches:" & Natural'Image (Accepted_Batches) &
                            "; rejected batches:" & Natural'Image (Rejected_Batches));
      Ada.Text_IO.Put_Line ("Passing the guard does NOT certify the globally optimal correction, " &
                            "3-D accuracy, pose, metric scale or navigation quality.");
      if Accepted_Batches /= 3 or else Rejected_Batches /= 1 then
         raise Program_Error with "unexpected guarded correction outcome";
      end if;
   end Rectified_And_Rejected;
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
         Guarded_Correction (F, First, Second, Inliers (Estimate));
      end;
   end;
   Rectified_And_Rejected;
end Fundamental_Synthetic;
