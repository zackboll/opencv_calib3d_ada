with Ada.Numerics;
with Ada.Numerics.Generic_Elementary_Functions;
with Ada.Text_IO;
with OpenCV.Calib3D;

procedure Ray_Plane_Unprojection_Synthetic is
   use Ada.Text_IO;
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   package Math is new Ada.Numerics.Generic_Elementary_Functions (OpenCV.Float64_Value);
   K : constant Camera_Intrinsics := (1000.0, 1000.0, 320.0, 240.0);
   Pose : constant World_To_Camera_Pose :=
     ([OpenCV.Float64_Value (Ada.Numerics.Pi), 0.0, 0.0], [0.0, 0.0, 10.0]);
   Ground : constant World_Plane := ([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
   Policy : constant World_Plane_Intersection_Options := (0.0, 0.0, 1000.0, 1.0E-10);
   Known : constant Object_Point_Array :=
     [[0.0, 0.0, 0.0], [1.0, -2.0, 0.0], [-3.0, 1.0, 0.0]];
   --  Analytic Rx(pi): Xc=Xw, Yc=-Yw, Zc=10-Zw. No production projection.
   Pixels : Image_Point_Array (1 .. Known'Length);
   procedure Check (Condition : Boolean; Message : String) is
   begin
      if not Condition then
         raise Program_Error with Message;
      end if;
   end Check;
   procedure Vector (Label : String; Value : Object_Point) is
   begin
      Put (Label & " (");
      for Component of Value loop
         Put (OpenCV.Float64_Value'Image (Component) & " ");
      end loop;
      Put_Line (")");
   end Vector;
   procedure Report (Value : World_Plane_Intersection) is
   begin
      Put_Line ("disposition: " & World_Plane_Intersection_Disposition'Image (Disposition (Value)));
      if Has_Signed_Ray_Distance (Value) then
         Put_Line ("signed range (forward only if positive): " &
                   OpenCV.Float64_Value'Image (Signed_Ray_Distance (Value)));
      end if;
      if Incidence_Evaluated (Value) then
         Put_Line ("absolute incidence cosine: " &
                   OpenCV.Float64_Value'Image (Absolute_Incidence_Cosine (Value)));
      end if;
      if Plane_Residual_Evaluated (Value) then
         Put_Line ("represented plane residual: " &
                   OpenCV.Float64_Value'Image (Absolute_Plane_Residual (Value)));
      end if;
      if Accepted (Value) then
         Vector ("accepted Cartesian world point", Accepted_World_Position (Value));
      end if;
   end Report;
begin
   Put_Line ("SYNTHETIC caller-defined Cartesian geometry, NOT sensor-derived terrain positioning.");
   Put_Line ("World coordinates and ranges use caller length units; translation is NOT camera position.");
   for I in Known'Range loop
      Pixels (I) := [K.Center_X + K.Focal_X * Known (I) (0) / 10.0,
                     K.Center_Y - K.Focal_Y * Known (I) (1) / 10.0];
   end loop;
   declare
      Rays : constant World_Ray_Array := World_Bearing_Rays (Pixels, K, No_Distortion, Pose);
      Values : constant World_Plane_Intersection_Array := Unproject_Pixels_Onto_World_Plane
        (Pixels, K, No_Distortion, Pose, Ground, Policy);
   begin
      for I in Pixels'Range loop
         Put_Line ("pixel: " & OpenCV.Float64_Value'Image (Pixels (I) (0)) & "," &
                   OpenCV.Float64_Value'Image (Pixels (I) (1)));
         Vector ("world ray origin", Rays (I).Origin);
         Vector ("world ray direction", Object_Point (Rays (I).Direction));
         Report (Values (I));
         Check (Accepted (Values (I)), "known ground point rejected");
         for Axis in 0 .. 2 loop
            Check (abs (Accepted_World_Position (Values (I)) (Axis) - Known (I) (Axis)) < 1.0E-10,
                   "independent ground oracle violated");
         end loop;
         Check (abs (Signed_Ray_Distance (Values (I)) -
                     Math.Sqrt (100.0 + Known (I) (0)**2 + Known (I) (1)**2)) < 1.0E-10,
                "independent range oracle violated");
      end loop;
   end;
   declare
      Rays : constant World_Ray_Array :=
        [([0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
         ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
         ([0.0, 0.0, 1.0], [1.0, 0.0, -1.0E-6]),
         ([0.0, 0.0, 1.0], [0.0, 0.0, 1.0]),
         ([0.0, 0.0, 1.0E10], [1.0, 0.0, -1.0E-300])];
      Values : constant World_Plane_Intersection_Array :=
        Intersect_World_Rays_With_Plane (Rays, Ground,
          (0.01, 0.0, OpenCV.Float64_Value'Last, 1.0E-10));
      type States is array (Positive range <>) of World_Plane_Intersection_Disposition;
      Expected : constant States :=
        [Parallel_Ray, Coplanar_Ray, Grazing_Incidence_Rejected,
         Intersection_Behind_Ray, Numerically_Unavailable];
   begin
      Put_Line ("Direct geometric cases (no original pixel):");
      for I in Values'Range loop
         Report (Values (I));
         Check (Disposition (Values (I)) = Expected (I), "classification assertion violated");
      end loop;
   end;
   declare
      --  Plane X+2Y+3Z=25; ray C=(1,2,3), V=(2,-1,3).
      --  lambda=(25-14)/9=11/9, not an expected production intersection.
      Values : constant World_Plane_Intersection_Array := Intersect_World_Rays_With_Plane
        ([([1.0, 2.0, 3.0], [2.0, -1.0, 3.0])],
         ([4.0, 0.0, 7.0], [1.0, 2.0, 3.0]), Policy);
      Expected : constant Object_Point := [31.0 / 9.0, 7.0 / 9.0, 20.0 / 3.0];
   begin
      Put_Line ("Tilted plane (no original pixel):");
      Report (Values (1));
      Check (Accepted (Values (1)), "tilted intersection rejected");
      for Axis in 0 .. 2 loop
         Check (abs (Accepted_World_Position (Values (1)) (Axis) - Expected (Axis)) < 1.0E-10,
                "tilted independent scalar oracle violated");
      end loop;
   end;
   Put_Line ("PASS: synthetic world-plane assertions");
end Ray_Plane_Unprojection_Synthetic;