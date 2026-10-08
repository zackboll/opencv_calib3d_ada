with Ada.Text_IO;
with OpenCV;
with OpenCV.Calib3D;

procedure Triangulation_Synthetic is
   use Ada.Text_IO;
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   Pose : constant Relative_Camera_Pose :=
     ([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [-1.0, 0.0, 0.0]);
   Known : constant Object_Point_Array := [[2.0, 1.0, 5.0], [-1.0, 0.5, 4.0], [2.0, 1.0, -5.0]];
   First, Second : Normalized_Image_Point_Array (Known'Range);
   Center : constant Camera_Direction := Second_Camera_Center_Direction_In_First (Pose);
begin
   Put_Line ("Coordinates are expressed in unit-baseline first-camera units.");
   Put_Line ("Metric baseline magnitude is unknown.");
   Put_Line ("These are not georeferenced navigation coordinates.");
   for Row in 0 .. 2 loop
      Put ("Relative rotation row:");
      for Col in 0 .. 2 loop
         Put (OpenCV.Float64_Value'Image (Pose.Rotation_First_To_Second (Row, Col)));
      end loop;
      New_Line;
   end loop;
   for Axis in 0 .. 2 loop
      Put_Line ("Unit translation-term / second-center direction: " &
        OpenCV.Float64_Value'Image (Pose.Translation_Direction (Axis)) & " / " &
        OpenCV.Float64_Value'Image (Center (Axis)));
   end loop;
   for I in Known'Range loop
      First (I) := [Known (I) (0) / Known (I) (2), Known (I) (1) / Known (I) (2)];
      Second (I) := [(Known (I) (0) - 1.0) / Known (I) (2), Known (I) (1) / Known (I) (2)];
   end loop;
   declare
      Points : constant Triangulated_Point_Array := Triangulate_Normalized (First, Second, Pose);
   begin
      for I in Points'Range loop
         Put_Line ("Correspondence" & Positive'Image (I) & " " & Triangulation_Status'Image (Points (I).Status));
         for Axis in 0 .. 1 loop
            Put_Line ("Normalized first/second:" & OpenCV.Float64_Value'Image (First (I) (Axis)) &
              OpenCV.Float64_Value'Image (Second (I) (Axis)));
         end loop;
         if Points (I).Status = Usable then
            for Axis in 0 .. 2 loop
               Put_Line ("First-frame reconstructed / known / difference:" &
                 OpenCV.Float64_Value'Image (Points (I).Position_In_First_Camera (Axis)) &
                 OpenCV.Float64_Value'Image (Known (I) (Axis)) &
                 OpenCV.Float64_Value'Image (Points (I).Position_In_First_Camera (Axis) - Known (I) (Axis)));
            end loop;
            Put_Line ("First/second depths:" & OpenCV.Float64_Value'Image (Points (I).First_Depth) &
              OpenCV.Float64_Value'Image (Points (I).Second_Depth));
            Put_Line ("Dimensionless normalized errors:" & OpenCV.Float64_Value'Image (Points (I).First_Normalized_Error) &
              OpenCV.Float64_Value'Image (Points (I).Second_Normalized_Error));
         end if;
      end loop;
   end;
end Triangulation_Synthetic;