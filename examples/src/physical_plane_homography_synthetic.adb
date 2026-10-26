with Ada.Numerics;
with Ada.Text_IO;
with OpenCV.Calib3D;

procedure Physical_Plane_Homography_Synthetic is
   use Ada.Text_IO;
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   KA : constant Camera_Intrinsics := (900.0, 870.0, 320.0, 240.0);
   KB : constant Camera_Intrinsics := (760.0, 810.0, 300.0, 210.0);
   A : constant World_To_Camera_Pose := ([Ada.Numerics.Pi, 0.0, 0.0], [0.0, 0.0, 10.0]);
   B : constant World_To_Camera_Pose := ([Ada.Numerics.Pi, 0.0, 0.0], [-2.0, 0.0, 10.0]);
   Ground : constant World_Plane := ([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
   Algebraic_Policy : constant Plane_Homography_Consistency_Options :=
     (3000, 1.0E-5, 0.999, 18, 2.0E-6, 2.0E-6, 2.0E-6);
   Direction : constant Plane_Correspondence_Verification_Options := ((0.0, 0.0, 100.0, 1.0E-10), 1.0E-8);
   Physical_Policy : constant Reciprocal_Plane_Verification_Options := (Direction, Direction, 1.0E-8);
   F1, F2 : Image_Point_Array (1 .. 24);
   H1, H2 : Image_Point_Array (1 .. 8);
   procedure Check (Condition : Boolean; Message : String) is
   begin
      if not Condition then raise Program_Error with Message; end if;
   end Check;
   --  Independent scalar observations of a downward-looking Z=0 world plane:
   --  Xc=(X-Cx,-Y,10). Separate grids; no homography/projection binding generates
   --  the observations. Lengths here are synthetic Cartesian world units.
   procedure Generate (First, Second : out Image_Point_Array; Offset : OpenCV.Float64_Value) is
   begin
      for I in 1 .. First'Length loop
         declare
            X : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((I - 1) mod 6) * 0.23 - 0.6 + Offset;
            Y : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((I - 1) / 6) * 0.25 - 0.4 + Offset;
         begin
            First (I) := [KA.Center_X + KA.Focal_X * X / 10.0, KA.Center_Y - KA.Focal_Y * Y / 10.0];
            Second (I) := [KB.Center_X + KB.Focal_X * (X - 2.0) / 10.0, KB.Center_Y - KB.Focal_Y * Y / 10.0];
         end;
      end loop;
   end Generate;
   function Request (Policy : Reciprocal_Plane_Verification_Options) return Physical_Plane_Homography_Result is
     (Verify_Physical_Plane_Homography_Consistency
       (F1, F2, H1, H2, KA, KB, No_Distortion, No_Distortion, A, B, Ground, Algebraic_Policy, Policy));
   procedure Report (Label : String; Result : Physical_Plane_Homography_Result) is
      Count : Natural := 0;
   begin
      Put_Line (Label & ": " & Physical_Plane_Homography_Disposition'Image (Disposition (Result)));
      Put_Line ("  Algebraic: " & Plane_Homography_Consistency_Disposition'Image (Disposition (Algebraic_Consistency (Result))));
      for D of Holdouts (Result) loop
         declare P : constant Reciprocal_Plane_Result := Physical_Verification (D);
         begin
            Put_Line ("  Held-out" & Positive'Image (Original_Position (D)) & ": " &
                      Reciprocal_Plane_Disposition'Image (Disposition (P)));
            if World_Separation_Evaluated (P) then
               Put_Line ("    World separation (world units):" & OpenCV.Float64_Value'Image (World_Separation (P)));
            end if;
            if Accepted (D) then Count := Count + 1; end if;
         end;
      end loop;
      Put_Line ("  Final accepted:" & Natural'Image (Count) & "/" & Natural'Image (Holdout_Count (Result)));
   end Report;
begin
   Put_Line ("SYNTHETIC compatibility evidence only; no DTED, ground truth, or navigation-accuracy claim.");
   Generate (F1, F2, 0.0); Generate (H1, H2, 0.073);
   declare
      Clean : constant Physical_Plane_Homography_Result := Request (Physical_Policy);
      Stricter : Reciprocal_Plane_Verification_Options := Physical_Policy;
   begin
      Report ("Known plane, independent cameras and withheld observations", Clean);
      Check (Accepted (Clean), "clean synthetic case must pass both policies");
      Stricter.Backward.Intersection.Maximum_Forward_Range := 1.0;
      declare Rejected : constant Physical_Plane_Homography_Result := Request (Stricter);
      begin
         Report ("Only backward physical range changed to 1 world unit", Rejected);
         Check (Accepted (Algebraic_Consistency (Rejected)) and then not Accepted (Rejected),
                "algebraic agreement must not override physical rejection");
         Check (Algebraic_Consistency (Clean) = Algebraic_Consistency (Rejected),
                "physical policy must not change predicted or empirical homography diagnostics");
         for D of Holdouts (Rejected) loop
            Check (Accepted (Forward_Verification (Physical_Verification (D))) and then
                   not Accepted (Reverse_Verification (Physical_Verification (D))), "independent directional policy");
         end loop;
      end;
   end;
end Physical_Plane_Homography_Synthetic;