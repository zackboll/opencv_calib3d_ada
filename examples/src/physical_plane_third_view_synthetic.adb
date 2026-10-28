with Ada.Numerics;
with Ada.Text_IO;
with OpenCV.Calib3D;

procedure Physical_Plane_Third_View_Synthetic is
   use Ada.Text_IO;
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   KA : constant Camera_Intrinsics := (900.0, 870.0, 320.0, 240.0);
   KB : constant Camera_Intrinsics := (760.0, 810.0, 300.0, 210.0);
   KC : constant Camera_Intrinsics := (730.0, 910.0, 301.0, 227.0);
   A : constant World_To_Camera_Pose := ([Ada.Numerics.Pi, 0.0, 0.0], [0.0, 0.0, 10.0]);
   B : constant World_To_Camera_Pose := ([Ada.Numerics.Pi, 0.0, 0.0], [-2.0, 0.0, 10.0]);
   C : constant World_To_Camera_Pose := ([0.0, 0.0, 0.0], [0.4, -0.6, 8.0]);
   Ground : constant World_Plane := ([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
   Algebraic_Policy : constant Plane_Homography_Consistency_Options :=
     (3000, 1.0E-5, 0.999, 18, 2.0E-6, 2.0E-6, 2.0E-6);
   Direction : constant Plane_Correspondence_Verification_Options := ((0.0, 0.0, 100.0, 1.0E-10), 1.0E-8);
   Physical_Policy : constant Reciprocal_Plane_Verification_Options := (Direction, Direction, 1.0E-8);
   F1 : Image_Point_Array (7 .. 30);
   F2 : Image_Point_Array (41 .. 64);
   H1 : Image_Point_Array (11 .. 18);
   H2 : Image_Point_Array (31 .. 38);
   Third : Image_Point_Array (51 .. 58);
   Later : Image_Point_Array (71 .. 78);
   procedure Check (Condition : Boolean; Message : String) is
   begin
      if not Condition then raise Program_Error with Message; end if;
   end Check;
   --  Independent scalar pinhole oracles; no production projection generates
   --  expected observations. Downward cameras: Xc=(X-Cx,-Y,10).
   procedure Generate (First, Second : out Image_Point_Array; Offset : OpenCV.Float64_Value) is
   begin
      for I in 1 .. First'Length loop
         declare
            X : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((I - 1) mod 6) * 0.23 - 0.6 + Offset;
            Y : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((I - 1) / 6) * 0.25 - 0.4 + Offset;
         begin
            First (First'First + (I - 1)) := [KA.Center_X + KA.Focal_X * X / 10.0, KA.Center_Y - KA.Focal_Y * Y / 10.0];
            Second (Second'First + (I - 1)) := [KB.Center_X + KB.Focal_X * (X - 2.0) / 10.0, KB.Center_Y - KB.Focal_Y * Y / 10.0];
         end;
      end loop;
   end Generate;
   function Evidence (Policy : Reciprocal_Plane_Verification_Options) return Physical_Plane_Homography_Result is
     (Verify_Physical_Plane_Homography_Consistency
       (F1, F2, H1, H2, KA, KB, No_Distortion, No_Distortion, A, B, Ground, Algebraic_Policy, Policy));
   function Verify (Prior : Physical_Plane_Homography_Result) return Physical_Plane_Third_View_Result is
     (Verify_Physical_Plane_Third_View (Prior, Third, KC, No_Distortion, C, (Maximum_Third_Pixel_Error => 0.5)));
begin
   Put_Line ("SYNTHETIC third-view compatibility only: no DTED, ground truth or navigation-accuracy claim.");
   Put_Line ("X_camera=R*X_world+t; t is not camera position. All lengths are Cartesian world units.");
   Generate (F1, F2, 0.0); Generate (H1, H2, 0.073);
   for I in 1 .. H1'Length loop
      declare
         X : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((I - 1) mod 6) * 0.23 - 0.6 + 0.073;
         Y : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((I - 1) / 6) * 0.25 - 0.4 + 0.073;
      begin
         Third (Third'First + (I - 1)) := [KC.Center_X + KC.Focal_X * (X + 0.4) / 8.0,
                                          KC.Center_Y + KC.Focal_Y * (Y - 0.6) / 8.0];
         --  Independently supplied later pixels are a handoff only, not verified.
         Later (Later'First + (I - 1)) := [100.0 + OpenCV.Float64_Value (I), 200.0];
      end;
   end loop;
   declare
      Prior : constant Physical_Plane_Homography_Result := Evidence (Physical_Policy);
      Clean : constant Physical_Plane_Third_View_Result := Verify (Prior);
      Stricter : Reciprocal_Plane_Verification_Options := Physical_Policy;
   begin
      Check (Accepted (Prior) and then Accepted (Clean), "all three independent camera policies must accept");
      for D of Holdouts (Prior) loop
         Check (Accepted (Physical_Verification (D)), "reciprocal physical evidence required");
      end loop;
      Put_Line ("Clean third camera: 8/8 accepted, inclusive maximum pixel error 0.5.");
      Third (Third'First + 2) (0) := Third (Third'First + 2) (0) + 2.0;
      declare
         Partial : constant Physical_Plane_Third_View_Result := Verify (Prior);
         Handoff : constant Accepted_Landmark_Set := Physical_Plane_Third_View_Landmark_Observations (Partial, Later);
      begin
         Check (not Accepted (Partial) and then Handoff.Count = 7, "isolated third observation rejection retains seven survivors");
         Check (Original_Positions (Handoff) = [1, 2, 4, 5, 6, 7, 8], "original identities survive compaction");
         for D of Holdouts (Partial) loop
            Put_Line ("Original held-out" & Positive'Image (Original_Position (D)) & ": " &
                      Physical_Plane_Third_View_Disposition'Image (Disposition (D)));
         end loop;
         Put ("Accepted later-image handoff original positions:");
         for Position of Original_Positions (Handoff) loop Put (Positive'Image (Position)); end loop;
         New_Line;
      end;
      Stricter.Backward.Intersection.Maximum_Forward_Range := 1.0;
      declare
         Rejected : constant Physical_Plane_Homography_Result := Evidence (Stricter);
         Verified : constant Physical_Plane_Third_View_Result := Verify (Rejected);
         Empty : constant Accepted_Landmark_Set := Physical_Plane_Third_View_Landmark_Observations (Verified, Later);
      begin
         Check (not Accepted (Rejected) and then Empty.Count = 0, "third pixels cannot repair rejected prior aggregate");
         Put_Line ("Rejected original aggregate: zero accepted landmark handoff.");
      end;
   end;
   Put_Line ("Planar landmarks do not establish depth diversity, PnP uniqueness or robust general six-DOF relocalization.");
   Put_Line ("The later camera is neither estimated nor verified. No PnP call hides planar degeneracy.");
end Physical_Plane_Third_View_Synthetic;