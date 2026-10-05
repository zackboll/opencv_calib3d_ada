with Ada.Unchecked_Deallocation;
with Ada.Numerics.Generic_Elementary_Functions;
with Interfaces;
with Interfaces.C;
with OpenCV.Core;
with OpenCV.Core.Float64_Vec2_Access;
with OpenCV.Core.Float64_Vec3_Access;
with OpenCV.Core.Module_Interop;
with OpenCV.Calib3D.Internal.C_API;
with System;

package body OpenCV.Calib3D is
   package C renames OpenCV.Calib3D.Internal.C_API;
   package Bridge renames OpenCV.Core.Module_Interop;
   package Vec2_Access renames OpenCV.Core.Float64_Vec2_Access;
   package Vec3_Access renames OpenCV.Core.Float64_Vec3_Access;
   package Math is new Ada.Numerics.Generic_Elementary_Functions
     (OpenCV.Float64_Value);
   use type OpenCV.Float64_Value;
   use type OpenCV.Core.Depth_Type;
   use type OpenCV.Core.Channel_Count;
   use type Interfaces.Integer_32;
   use type Interfaces.Unsigned_8;
   use type Interfaces.Unsigned_64;
   use type Interfaces.C.C_float;
   use type System.Address;

   procedure Free is new Ada.Unchecked_Deallocation
     (Object => Inlier_Index_Array, Name => Inlier_Buffer);

   overriding procedure Finalize (Self : in out Pose_Estimate) is
   begin
      if Self.Data /= null then
         Free (Self.Data);
      end if;
      Self.Has_Pose := False;
      Self.Value := (Rotation => [others => 0.0], Translation => [others => 0.0]);
   end Finalize;

   function Is_Finite (Value : OpenCV.Float64_Value) return Boolean is
     (Value = Value and then
      Value >= -OpenCV.Float64_Value'Last and then
      Value <= OpenCV.Float64_Value'Last);

   procedure Validate (Value : Camera_Intrinsics) is
   begin
      if not (Is_Finite (Value.Focal_X) and then Is_Finite (Value.Focal_Y)
              and then Is_Finite (Value.Center_X) and then Is_Finite (Value.Center_Y)
              and then Value.Focal_X > 0.0 and then Value.Focal_Y > 0.0)
      then
         raise OpenCV.OpenCV_Error with "Camera intrinsics must be finite with positive focal lengths";
      end if;
   end Validate;

   procedure Validate (Value : Distortion_Coefficients) is
   begin
      if not (Is_Finite (Value.K1) and then Is_Finite (Value.K2)
              and then Is_Finite (Value.P1) and then Is_Finite (Value.P2)
              and then Is_Finite (Value.K3))
      then
         raise OpenCV.OpenCV_Error with "Distortion coefficients must be finite";
      end if;
   end Validate;

   procedure Validate (Value : World_To_Camera_Pose) is
   begin
      for Component of Value.Rotation loop
         if not Is_Finite (Component) then
            raise OpenCV.OpenCV_Error with "Rotation vector must be finite";
         end if;
      end loop;
      for Component of Value.Translation loop
         if not Is_Finite (Component) then
            raise OpenCV.OpenCV_Error with "Translation vector must be finite";
         end if;
      end loop;
   end Validate;

   --  Check in a type wider than Positive: Positive is already INT32-bounded
   --  on common GNAT targets, but the C ABI bound is independent of that.
   function Native_Iterations_Fit (Value : Interfaces.Unsigned_64) return Boolean is
     (Value > 0 and then Value <= Interfaces.Unsigned_64 (Interfaces.Integer_32'Last));

   procedure Validate (Value : RANSAC_Options) is
   begin
      if not Native_Iterations_Fit (Interfaces.Unsigned_64 (Value.Maximum_Iterations))
        or else not Is_Finite (Value.Reprojection_Error_Pixels)
        or else Value.Reprojection_Error_Pixels <= 0.0
        or else Value.Reprojection_Error_Pixels > OpenCV.Float64_Value (Interfaces.C.C_float'Last)
         or else Interfaces.C.C_float (Value.Reprojection_Error_Pixels) <= 0.0
        or else not Is_Finite (Value.Confidence)
        or else not (Value.Confidence > 0.0 and then Value.Confidence < 1.0)
      then
         raise OpenCV.OpenCV_Error with "Invalid solvePnPRansac options";
      end if;
   end Validate;

   procedure Validate (Points : Object_Point_Array) is
   begin
      for Point of Points loop
         for Component of Point loop
            if not Is_Finite (Component) then
               raise OpenCV.OpenCV_Error with "Object points must be finite";
            end if;
         end loop;
      end loop;
   end Validate;

   procedure Validate (Points : Image_Point_Array) is
   begin
      for Point of Points loop
         for Component of Point loop
            if not Is_Finite (Component) then
               raise OpenCV.OpenCV_Error with "Image points must be finite";
            end if;
         end loop;
      end loop;
   end Validate;

   function To_C (Value : Camera_Intrinsics) return C.C_Camera_Intrinsics is
     (Interfaces.C.double (Value.Focal_X), Interfaces.C.double (Value.Focal_Y),
      Interfaces.C.double (Value.Center_X), Interfaces.C.double (Value.Center_Y));

   function To_C (Value : Distortion_Coefficients) return C.C_Distortion5 is
     (Interfaces.C.double (Value.K1), Interfaces.C.double (Value.K2),
      Interfaces.C.double (Value.P1), Interfaces.C.double (Value.P2),
      Interfaces.C.double (Value.K3));

   function To_C (Value : World_To_Camera_Pose) return C.C_Pose is
     (Interfaces.C.double (Value.Rotation (0)), Interfaces.C.double (Value.Rotation (1)),
      Interfaces.C.double (Value.Rotation (2)), Interfaces.C.double (Value.Translation (0)),
      Interfaces.C.double (Value.Translation (1)), Interfaces.C.double (Value.Translation (2)));

   function From_C (Value : C.C_Pose) return World_To_Camera_Pose is
     ([OpenCV.Float64_Value (Value.RX), OpenCV.Float64_Value (Value.RY),
       OpenCV.Float64_Value (Value.RZ)],
      [OpenCV.Float64_Value (Value.TX), OpenCV.Float64_Value (Value.TY),
       OpenCV.Float64_Value (Value.TZ)]);

   function Object_Matrix (Points : Object_Point_Array) return OpenCV.Core.Mat is
   begin
      if Points'Length = 0 then
         declare
            Empty : OpenCV.Core.Mat;
         begin
            return Empty;
         end;
      end if;
      return Result : OpenCV.Core.Mat := OpenCV.Core.Create
        (Points'Length, 1, (Depth => OpenCV.Core.Float64, Channels => 3)) do
         for I in Points'Range loop
            Vec3_Access.Set (Result, I - Points'First, 0, Points (I));
         end loop;
      end return;
   end Object_Matrix;

   function Image_Matrix (Points : Image_Point_Array) return OpenCV.Core.Mat is
   begin
      if Points'Length = 0 then
         declare
            Empty : OpenCV.Core.Mat;
         begin
            return Empty;
         end;
      end if;
      return Result : OpenCV.Core.Mat := OpenCV.Core.Create
        (Points'Length, 1, (Depth => OpenCV.Core.Float64, Channels => 2)) do
         for I in Points'Range loop
            Vec2_Access.Set (Result, I - Points'First, 0, Points (I));
         end loop;
      end return;
   end Image_Matrix;

   function Found (Estimate : Pose_Estimate) return Boolean is (Estimate.Has_Pose);

   function Pose (Estimate : Pose_Estimate) return World_To_Camera_Pose is
   begin
      if not Estimate.Has_Pose then
         raise OpenCV.OpenCV_Error with "Pose requested from an unsuccessful estimate";
      end if;
      return Estimate.Value;
   end Pose;

   function Inlier_Count (Estimate : Pose_Estimate) return Natural is
     (if Estimate.Data = null then 0 else Estimate.Data.all'Length);

   function Inlier (Estimate : Pose_Estimate; Index : Positive) return Positive is
   begin
      if Estimate.Data = null or else Index not in Estimate.Data.all'Range then
         raise OpenCV.OpenCV_Error with "Inlier index out of range";
      end if;
      return Estimate.Data (Index);
   end Inlier;

   function Inliers (Estimate : Pose_Estimate) return Inlier_Index_Array is
   begin
      if Estimate.Data = null then
         return [1 .. 0 => <>];
      end if;
      return Estimate.Data.all;
   end Inliers;

   function Camera_Center (Pose : World_To_Camera_Pose) return Object_Point is
      Native_Pose : aliased C.C_Pose;
      Center      : aliased C.C_Point3 := (others => 0.0);
   begin
      Validate (Pose);
      Native_Pose := To_C (Pose);
      C.Check (C.Camera_Center (Native_Pose'Access, Center'Access), "Calib3D.Camera_Center");
      return [0 => OpenCV.Float64_Value (Center.X),
              1 => OpenCV.Float64_Value (Center.Y),
              2 => OpenCV.Float64_Value (Center.Z)];
   end Camera_Center;

   function Rotation_Matrix_Of (Pose : World_To_Camera_Pose) return Rotation_Matrix is
      Native_Pose : aliased C.C_Pose;
      Matrix : aliased C.C_Rotation_Matrix := (others => 0.0);
   begin
      Validate (Pose);
      Native_Pose := To_C (Pose);
      C.Check (C.Rotation_Matrix_Of (Native_Pose'Access, Matrix'Access),
               "Calib3D.Rotation_Matrix_Of");
      return Result : constant Rotation_Matrix :=
        [[OpenCV.Float64_Value (Matrix.M00), OpenCV.Float64_Value (Matrix.M01),
          OpenCV.Float64_Value (Matrix.M02)],
         [OpenCV.Float64_Value (Matrix.M10), OpenCV.Float64_Value (Matrix.M11),
          OpenCV.Float64_Value (Matrix.M12)],
         [OpenCV.Float64_Value (Matrix.M20), OpenCV.Float64_Value (Matrix.M21),
          OpenCV.Float64_Value (Matrix.M22)]]
      do
         for Value of Result loop
            if not Is_Finite (Value) then
               raise OpenCV.OpenCV_Error with "Rodrigues produced a nonfinite matrix";
            end if;
         end loop;
      end return;
   end Rotation_Matrix_Of;

   procedure Validate_Vector (Value : Object_Point) is
   begin
      for Component of Value loop
         if not Is_Finite (Component) then
            raise OpenCV.OpenCV_Error with "Point/direction must be finite";
         end if;
      end loop;
   end Validate_Vector;

   function Rotate
     (Matrix : Rotation_Matrix; Value : Object_Point; Transpose : Boolean := False)
      return Object_Point
   is
   begin
      Validate_Vector (Value);
      return Result : Object_Point := [others => 0.0] do
         for Row in 0 .. 2 loop
            for Col in 0 .. 2 loop
               declare
                  Factor : constant OpenCV.Float64_Value :=
                    (if Transpose then Matrix (Col, Row) else Matrix (Row, Col));
                  Product : constant OpenCV.Float64_Value := Factor * Value (Col);
               begin
                  if not Is_Finite (Product) then
                     raise OpenCV.OpenCV_Error with "Nonfinite rotation intermediate";
                  end if;
                  Result (Row) := Result (Row) + Product;
                  if not Is_Finite (Result (Row)) then
                     raise OpenCV.OpenCV_Error with "Nonfinite rotation result";
                  end if;
               end;
            end loop;
         end loop;
      end return;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Rotation exceeds Float64 range";
   end Rotate;

   function World_To_Camera_Point
     (Pose : World_To_Camera_Pose; Point : Object_Point) return Object_Point
   is
      Matrix : constant Rotation_Matrix := Rotation_Matrix_Of (Pose);
      Result : Object_Point := Rotate (Matrix, Point);
   begin
      for Axis in 0 .. 2 loop
         Result (Axis) := Result (Axis) + Pose.Translation (Axis);
      end loop;
      Validate_Vector (Result);
      return Result;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Point transform exceeds Float64 range";
   end World_To_Camera_Point;

   function Camera_To_World_Point
     (Pose : World_To_Camera_Pose; Point : Object_Point) return Object_Point
   is
      Matrix : constant Rotation_Matrix := Rotation_Matrix_Of (Pose);
      Shifted : Object_Point;
   begin
      Validate_Vector (Point);
      for Axis in 0 .. 2 loop
         Shifted (Axis) := Point (Axis) - Pose.Translation (Axis);
      end loop;
      Validate_Vector (Shifted);
      return Rotate (Matrix, Shifted, Transpose => True);
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Point transform exceeds Float64 range";
   end Camera_To_World_Point;

   function World_To_Camera_Direction
     (Pose : World_To_Camera_Pose; Direction : World_Direction) return Camera_Direction is
     (Camera_Direction (Rotate (Rotation_Matrix_Of (Pose), Object_Point (Direction))));

   function Camera_To_World_Direction
     (Pose : World_To_Camera_Pose; Direction : Camera_Direction) return World_Direction is
     (World_Direction (Rotate (Rotation_Matrix_Of (Pose), Object_Point (Direction), True)));

   function Unit_Vector (Value : Object_Point) return Object_Point is
      Scale : OpenCV.Float64_Value := 0.0;
      Scaled : Object_Point;
      Squared : OpenCV.Float64_Value := 0.0;
      Norm : OpenCV.Float64_Value;
   begin
      Validate_Vector (Value);
      for Component of Value loop
         Scale := OpenCV.Float64_Value'Max (Scale, abs Component);
      end loop;
      if Scale = 0.0 then
         raise OpenCV.OpenCV_Error with "Zero direction cannot be normalized";
      end if;
      for Axis in 0 .. 2 loop
         Scaled (Axis) := Value (Axis) / Scale;
         Squared := Squared + Scaled (Axis) * Scaled (Axis);
      end loop;
      --  Normalize in scaled space; do not form Scale*Norm, which could
      --  overflow for a finite vector whose mathematical norm exceeds Last.
      Norm := Math.Sqrt (Squared);
      if not Is_Finite (Norm) or else Norm <= 0.0 then
         raise OpenCV.OpenCV_Error with "Invalid scaled direction norm";
      end if;
      for Axis in 0 .. 2 loop
         Scaled (Axis) := Scaled (Axis) / Norm;
      end loop;
      Validate_Vector (Scaled);
      return Scaled;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Direction normalization exceeds Float64 range";
   end Unit_Vector;

   function Undistort_To_Normalized
     (Points : Image_Point_Array; Intrinsics : Camera_Intrinsics;
      Distortion : Distortion_Coefficients := No_Distortion)
      return Normalized_Image_Point_Array
   is
      Native_Intrinsics : aliased C.C_Camera_Intrinsics;
      Native_Distortion : aliased C.C_Distortion5;
      Images, Output : OpenCV.Core.Mat;
      Code : C.Status := C.Success;
      procedure Input_Callback (Input : Bridge.Input_Mat_Handle) is
         procedure Output_Callback (Destination : Bridge.Output_Mat_Handle) is
         begin
            Code := C.Undistort_Normalized
              (Input, Native_Intrinsics'Access, Native_Distortion'Access, Destination);
         end Output_Callback;
      begin
         Bridge.With_Output_Handle (Output, Output_Callback'Access);
      end Input_Callback;
   begin
      Validate (Intrinsics);
      Validate (Distortion);
      Validate (Points);
      if Points'Length = 0 then
         return [1 .. 0 => <>];
      end if;
      Images := Image_Matrix (Points);
      Output := OpenCV.Core.Create
        (Points'Length, 1, (Depth => OpenCV.Core.Float64, Channels => 2));
      Native_Intrinsics := To_C (Intrinsics);
      Native_Distortion := To_C (Distortion);
      Bridge.With_Input_Handle (Images, Input_Callback'Access);
      C.Check (Code, "Calib3D.Undistort_To_Normalized");
      if Output.Rows /= Points'Length or else Output.Columns /= 1
        or else Output.Depth /= OpenCV.Core.Float64 or else Output.Channels /= 2
      then
         raise OpenCV.OpenCV_Error with "Invalid native undistortPoints output";
      end if;
      return Result : Normalized_Image_Point_Array (1 .. Points'Length) do
         for I in Result'Range loop
            Result (I) := Normalized_Image_Point (Vec2_Access.Get (Output, I - 1, 0));
            for Component of Result (I) loop
               if not Is_Finite (Component) then
                  raise OpenCV.OpenCV_Error with "Nonfinite normalized coordinate";
               end if;
            end loop;
         end loop;
      end return;
   end Undistort_To_Normalized;

   function Camera_Bearing_Rays
     (Points : Image_Point_Array; Intrinsics : Camera_Intrinsics;
      Distortion : Distortion_Coefficients := No_Distortion)
      return Camera_Direction_Array
   is
      Normalized : constant Normalized_Image_Point_Array :=
        Undistort_To_Normalized (Points, Intrinsics, Distortion);
   begin
      return Result : Camera_Direction_Array (1 .. Points'Length) do
         for I in Result'Range loop
            Result (I) := Camera_Direction
              (Unit_Vector ([Normalized (I) (0), Normalized (I) (1), 1.0]));
            if Result (I) (2) <= 0.0 then
               raise OpenCV.OpenCV_Error with "Camera bearing must have positive Z";
            end if;
         end loop;
      end return;
   end Camera_Bearing_Rays;

   function World_Bearing_Rays
     (Points : Image_Point_Array; Intrinsics : Camera_Intrinsics;
      Distortion : Distortion_Coefficients; Pose : World_To_Camera_Pose)
      return World_Ray_Array
   is
   begin
      Validate (Pose);
      declare
         Camera : constant Camera_Direction_Array :=
           Camera_Bearing_Rays (Points, Intrinsics, Distortion);
      begin
         if Points'Length = 0 then
            return [1 .. 0 => <>];
         end if;
         declare
            Matrix : constant Rotation_Matrix := Rotation_Matrix_Of (Pose);
            Center : constant Object_Point := Camera_Center (Pose);
         begin
            Validate_Vector (Center);
            return Result : World_Ray_Array (1 .. Points'Length) do
               for I in Result'Range loop
                  Result (I) := (Origin => Center, Direction => World_Direction
                    (Unit_Vector (Rotate (Matrix, Object_Point (Camera (I)), True))));
               end loop;
            end return;
         end;
      end;
   end World_Bearing_Rays;

   function Project_Points
     (Points     : Object_Point_Array;
      Intrinsics : Camera_Intrinsics;
      Distortion : Distortion_Coefficients;
      Pose       : World_To_Camera_Pose) return Image_Point_Array
   is
      Native_Intrinsics : aliased C.C_Camera_Intrinsics;
      Native_Distortion : aliased C.C_Distortion5;
      Native_Pose       : aliased C.C_Pose;
      Objects           : OpenCV.Core.Mat;
      Output            : OpenCV.Core.Mat;
      Code              : C.Status := C.Success;
      procedure Input_Callback (Input : Bridge.Input_Mat_Handle) is
         procedure Output_Callback (Destination : Bridge.Output_Mat_Handle) is
         begin
            Code := C.Project_Points (Input, Native_Intrinsics'Access,
                                      Native_Distortion'Access, Native_Pose'Access,
                                      Destination);
         end Output_Callback;
      begin
         Bridge.With_Output_Handle (Output, Output_Callback'Access);
      end Input_Callback;
   begin
      Validate (Intrinsics);
      Validate (Distortion);
      Validate (Pose);
      Validate (Points);
      if Points'Length = 0 then
         return [1 .. 0 => <>];
      end if;
      Objects := Object_Matrix (Points);
      Output := OpenCV.Core.Create
        (Points'Length, 1, (Depth => OpenCV.Core.Float64, Channels => 2));
      Native_Intrinsics := To_C (Intrinsics);
      Native_Distortion := To_C (Distortion);
      Native_Pose := To_C (Pose);
      Bridge.With_Input_Handle (Objects, Input_Callback'Access);
      C.Check (Code, "Calib3D.Project_Points");
      if Output.Rows /= Points'Length or else Output.Columns /= 1
        or else Output.Depth /= OpenCV.Core.Float64 or else Output.Channels /= 2
      then
         raise OpenCV.OpenCV_Error with "Invalid native projectPoints output";
      end if;
      return Result : Image_Point_Array (1 .. Points'Length) do
         for I in Result'Range loop
            Result (I) := Vec2_Access.Get (Output, I - 1, 0);
            for Component of Result (I) loop
               if not Is_Finite (Component) then
                  raise OpenCV.OpenCV_Error with "projectPoints produced a nonfinite coordinate";
               end if;
            end loop;
         end loop;
      end return;
   end Project_Points;

   function Reprojection_Errors
     (Object_Points : Object_Point_Array;
      Image_Points  : Image_Point_Array;
      Intrinsics    : Camera_Intrinsics;
      Distortion    : Distortion_Coefficients;
      Pose          : World_To_Camera_Pose) return Reprojection_Error_Array
   is
   begin
      if Object_Points'Length /= Image_Points'Length then
         raise OpenCV.OpenCV_Error with "Object/image correspondence counts differ";
      end if;
      Validate (Image_Points);
      declare
         Projected : constant Image_Point_Array :=
           Project_Points (Object_Points, Intrinsics, Distortion, Pose);
      begin
         return Errors : Reprojection_Error_Array (1 .. Object_Points'Length) do
            for I in Errors'Range loop
               declare
                  DX : constant OpenCV.Float64_Value :=
                    Projected (I) (0) - Image_Points (Image_Points'First + (I - 1)) (0);
                  DY : constant OpenCV.Float64_Value :=
                    Projected (I) (1) - Image_Points (Image_Points'First + (I - 1)) (1);
                  Scale : OpenCV.Float64_Value;
               begin
                  if not (Is_Finite (DX) and then Is_Finite (DY)) then
                     raise OpenCV.OpenCV_Error with "Nonfinite reprojection subtraction";
                  end if;
                  Scale := OpenCV.Float64_Value'Max (abs DX, abs DY);
                  Errors (I) :=
                    (if Scale = 0.0 then 0.0 else
                     Scale * Math.Sqrt ((DX / Scale) ** 2 + (DY / Scale) ** 2));
                  if not Is_Finite (Errors (I)) then
                     raise OpenCV.OpenCV_Error with "Nonfinite reprojection error";
                  end if;
               end;
            end loop;
         end return;
      end;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Reprojection error exceeds Float64 range";
   end Reprojection_Errors;

   function Summarize_Reprojection
     (Errors : Reprojection_Error_Array) return Reprojection_Summary
   is
      Scale : OpenCV.Float64_Value := 0.0;
      SSQ   : OpenCV.Float64_Value := 1.0;
      Result : Reprojection_Summary := (Count => Errors'Length, others => <>);
   begin
      for Error of Errors loop
         if not Is_Finite (Error) or else Error < 0.0 then
            raise OpenCV.OpenCV_Error with "Pixel errors must be finite and nonnegative";
         end if;
         if Error > 0.0 then
            if Scale < Error then
               SSQ := 1.0 + SSQ * (Scale / Error) ** 2;
               Scale := Error;
            else
               SSQ := SSQ + (Error / Scale) ** 2;
            end if;
         end if;
      end loop;
      Result.Maximum_Error_Pixels := Scale;
      if Errors'Length > 0 then
         --  Divide before multiplying by Scale. RMS cannot exceed the maximum;
         --  clamp rounding of SSQ/Count to one at the Float64 range boundary.
         Result.RMS_Error_Pixels := Scale * Math.Sqrt
           (OpenCV.Float64_Value'Min (1.0, SSQ / OpenCV.Float64_Value (Errors'Length)));
      end if;
      if not Is_Finite (Result.RMS_Error_Pixels) then
         raise OpenCV.OpenCV_Error with "Nonfinite reprojection RMS";
      end if;
      return Result;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Reprojection summary exceeds Float64 range";
   end Summarize_Reprojection;

   procedure Refine_Pose_Iterative
     (Object_Points : Object_Point_Array;
      Image_Points  : Image_Point_Array;
      Intrinsics    : Camera_Intrinsics;
      Distortion    : Distortion_Coefficients := No_Distortion;
      Pose          : in out World_To_Camera_Pose;
      Refined       : out Boolean)
   is
      Native_Intrinsics : aliased C.C_Camera_Intrinsics;
      Native_Distortion : aliased C.C_Distortion5;
      Initial_Pose      : aliased C.C_Pose;
      Native_Pose       : aliased C.C_Pose := (others => 0.0);
      Native_Refined    : aliased Interfaces.Unsigned_8 := 0;
      Objects, Images   : OpenCV.Core.Mat;
      Code              : C.Status := C.Success;
      procedure Object_Callback (Object_Handle : Bridge.Input_Mat_Handle) is
         procedure Image_Callback (Image_Handle : Bridge.Input_Mat_Handle) is
         begin
            Code := C.Refine_Pose_Iterative
              (Object_Handle, Image_Handle, Native_Intrinsics'Access,
               Native_Distortion'Access, Initial_Pose'Access,
               Native_Refined'Access, Native_Pose'Access);
         end Image_Callback;
      begin
         Bridge.With_Input_Handle (Images, Image_Callback'Access);
      end Object_Callback;
   begin
      Refined := False;
      Validate (Intrinsics);
      Validate (Distortion);
      Validate (Pose);
      Validate (Object_Points);
      Validate (Image_Points);
      if Object_Points'Length /= Image_Points'Length or else Object_Points'Length < 4 then
         raise OpenCV.OpenCV_Error with "Iterative refinement requires equal counts >=4";
      end if;
      Objects := Object_Matrix (Object_Points);
      Images := Image_Matrix (Image_Points);
      Native_Intrinsics := To_C (Intrinsics);
      Native_Distortion := To_C (Distortion);
      Initial_Pose := To_C (Pose);
      Bridge.With_Input_Handle (Objects, Object_Callback'Access);
      C.Check (Code, "Calib3D.Refine_Pose_Iterative");
      if Native_Refined = 0 then
         return;
      elsif Native_Refined /= 1 then
         raise OpenCV.OpenCV_Error with "Invalid native refinement flag";
      end if;
      declare
         Value : constant World_To_Camera_Pose := From_C (Native_Pose);
      begin
         Validate (Value);
         Pose := Value;
         Refined := True;
      end;
   end Refine_Pose_Iterative;

   function Solve_PnP_RANSAC
     (Object_Points : Object_Point_Array;
      Image_Points  : Image_Point_Array;
      Intrinsics    : Camera_Intrinsics;
      Distortion    : Distortion_Coefficients := No_Distortion;
      Options       : RANSAC_Options := (others => <>)) return Pose_Estimate
   is
      type Result_Guard is new Ada.Finalization.Limited_Controlled with record
         Handle : aliased System.Address := System.Null_Address;
      end record;
      overriding procedure Finalize (Self : in out Result_Guard);
      overriding procedure Finalize (Self : in out Result_Guard) is
      begin
         C.Result_Destroy (Self.Handle);
         Self.Handle := System.Null_Address;
      end Finalize;

      Native_Intrinsics : aliased C.C_Camera_Intrinsics;
      Native_Distortion : aliased C.C_Distortion5;
      Objects           : OpenCV.Core.Mat;
      Images            : OpenCV.Core.Mat;
      Guard             : Result_Guard;
      Code              : C.Status := C.Success;
      Native_Found      : aliased Interfaces.Unsigned_8 := 0;
      Native_Pose       : aliased C.C_Pose := (others => 0.0);
      Native_Count      : aliased Interfaces.Integer_32 := 0;
      procedure Object_Callback (Object_Handle : Bridge.Input_Mat_Handle) is
         procedure Image_Callback (Image_Handle : Bridge.Input_Mat_Handle) is
         begin
            Code := C.Solve_PnP_RANSAC
              (Object_Handle, Image_Handle, Native_Intrinsics'Access,
               Native_Distortion'Access,
               Interfaces.Integer_32 (Options.Maximum_Iterations),
               Interfaces.C.double (Options.Reprojection_Error_Pixels),
               Interfaces.C.double (Options.Confidence), Guard.Handle'Access);
         end Image_Callback;
      begin
         Bridge.With_Input_Handle (Images, Image_Callback'Access);
      end Object_Callback;
   begin
      Validate (Intrinsics);
      Validate (Distortion);
      Validate (Options);
      Validate (Object_Points);
      Validate (Image_Points);
      if Object_Points'Length /= Image_Points'Length then
         raise OpenCV.OpenCV_Error with "Object/image correspondence counts differ";
      end if;
      if Object_Points'Length < 4 then
         raise OpenCV.OpenCV_Error with "solvePnPRansac requires at least four correspondences";
      end if;
      Objects := Object_Matrix (Object_Points);
      Images := Image_Matrix (Image_Points);
      Native_Intrinsics := To_C (Intrinsics);
      Native_Distortion := To_C (Distortion);
      Bridge.With_Input_Handle (Objects, Object_Callback'Access);
      C.Check (Code, "Calib3D.Solve_PnP_RANSAC");
      if Guard.Handle = System.Null_Address then
         raise OpenCV.OpenCV_Error with "Native solvePnPRansac did not publish a result";
      end if;
      C.Check (C.Result_Found (Guard.Handle, Native_Found'Access), "Calib3D.Pose_Found");
      if Native_Found = 0 then
         return Estimate : Pose_Estimate do
            null;
         end return;
      end if;
      C.Check (C.Result_Pose (Guard.Handle, Native_Pose'Access), "Calib3D.Pose_Value");
      C.Check (C.Result_Inlier_Count (Guard.Handle, Native_Count'Access), "Calib3D.Inlier_Count");
      if Native_Count < 4 or else Native_Count > Interfaces.Integer_32 (Object_Points'Length) then
         raise OpenCV.OpenCV_Error with "Invalid native PnP inlier count";
      end if;
      declare
         Value : constant World_To_Camera_Pose := From_C (Native_Pose);
      begin
         Validate (Value);
         return Estimate : Pose_Estimate do
            Estimate.Has_Pose := True;
            Estimate.Value := Value;
            Estimate.Data := new Inlier_Index_Array (1 .. Natural (Native_Count));
            for I in Estimate.Data.all'Range loop
               declare
                  Native_Index : aliased Interfaces.Integer_32 := 0;
               begin
                  C.Check (C.Result_Inlier (Guard.Handle, Interfaces.Integer_32 (I - 1),
                                            Native_Index'Access), "Calib3D.Inlier");
                  if Native_Index < 0 or else Native_Index >= Interfaces.Integer_32 (Object_Points'Length) then
                     raise OpenCV.OpenCV_Error with "Invalid native PnP inlier index";
                  end if;
                  Estimate.Data (I) := Positive (Native_Index + 1);
                  if I > Estimate.Data.all'First and then Estimate.Data (I) <= Estimate.Data (I - 1) then
                     raise OpenCV.OpenCV_Error with "Native PnP inliers are not strictly ascending";
                  end if;
               end;
            end loop;
         end return;
      end;
   end Solve_PnP_RANSAC;
end OpenCV.Calib3D;
