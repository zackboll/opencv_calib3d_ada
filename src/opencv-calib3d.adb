with Ada.Unchecked_Deallocation;
with Ada.Numerics.Generic_Elementary_Functions;
with Interfaces;
with Interfaces.C;
with OpenCV.Core;
with OpenCV.Core.Float64_Vec2_Access;
with OpenCV.Core.Float64_Vec3_Access;
with OpenCV.Core.Module_Interop;
with OpenCV.Calib3D.Internal.C_API;
with OpenCV.Calib3D.Internal.Point_Refinement;
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

   overriding procedure Finalize (Self : in out Homography_Estimate) is
   begin
      if Self.Data /= null then
         Free (Self.Data);
      end if;
      Self.Has_Model := False;
      Self.Value := [others => [others => 0.0]];
   end Finalize;

   function Found (Estimate : Homography_Estimate) return Boolean is (Estimate.Has_Model);
   function Homography (Estimate : Homography_Estimate) return Homography_Matrix is
   begin
      if not Estimate.Has_Model then
         raise OpenCV.OpenCV_Error with "Homography requested from an unsuccessful estimate";
      end if;
      return Estimate.Value;
   end Homography;
   function Inlier_Count (Estimate : Homography_Estimate) return Natural is
     (if Estimate.Data = null then 0 else Estimate.Data.all'Length);
   function Inlier (Estimate : Homography_Estimate; Index : Positive) return Positive is
   begin
      if Estimate.Data = null or else Index not in Estimate.Data.all'Range then
         raise OpenCV.OpenCV_Error with "Homography inlier index out of range";
      end if;
      return Estimate.Data (Index);
   end Inlier;
   function Inliers (Estimate : Homography_Estimate) return Inlier_Index_Array is
   begin
      if Estimate.Data = null then
         return [1 .. 0 => <>];
      end if;
      return Estimate.Data.all;
   end Inliers;

   overriding procedure Finalize (Self : in out Fundamental_Estimate) is
   begin
      if Self.Data /= null then
         Free (Self.Data);
      end if;
      Self.Has_Model := False;
      Self.Value := [others => [others => 0.0]];
   end Finalize;

   function Found (Estimate : Fundamental_Estimate) return Boolean is (Estimate.Has_Model);
   function Fundamental (Estimate : Fundamental_Estimate) return Fundamental_Matrix is
   begin
      if not Estimate.Has_Model then
         raise OpenCV.OpenCV_Error with "Fundamental requested from an unsuccessful estimate";
      end if;
      return Estimate.Value;
   end Fundamental;
   function Inlier_Count (Estimate : Fundamental_Estimate) return Natural is
     (if Estimate.Data = null then 0 else Estimate.Data.all'Length);
   function Inlier (Estimate : Fundamental_Estimate; Index : Positive) return Positive is
   begin
      if Estimate.Data = null or else Index not in Estimate.Data.all'Range then
         raise OpenCV.OpenCV_Error with "Fundamental inlier index out of range";
      end if;
      return Estimate.Data (Index);
   end Inlier;
   function Inliers (Estimate : Fundamental_Estimate) return Inlier_Index_Array is
   begin
      if Estimate.Data = null then
         return [1 .. 0 => <>];
      end if;
      return Estimate.Data.all;
   end Inliers;

   function Maximum_Epipolar_Error
     (Matrix : Fundamental_Matrix; First_Point, Second_Point : Image_Point)
      return Epipolar_Error_Result
   is
      subtype Scalar is OpenCV.Float64_Value;
      type Line is array (Natural range 0 .. 2) of Scalar;
      L1, L2 : Line;
      N1, N2, D1, D2 : Scalar;
      function Sum3 (A, X, B, Y, C : Scalar) return Scalar is
         AX : constant Scalar := A * X;
         BY : constant Scalar := B * Y;
         Sum : constant Scalar := AX + BY;
         Value : constant Scalar := Sum + C;
      begin
         if not Is_Finite (AX) or else not Is_Finite (BY) or else
           not Is_Finite (Sum) or else not Is_Finite (Value)
         then
            raise Constraint_Error;
         end if;
         return Value;
      end Sum3;
      function Hypot (A, B : Scalar) return Scalar is
         Scale : constant Scalar := Scalar'Max (abs A, abs B);
      begin
         if Scale = 0.0 then
            return 0.0;
         end if;
         return Scale * Math.Sqrt ((A / Scale)**2 + (B / Scale)**2);
      end Hypot;
   begin
      for Value of Matrix loop
         if not Is_Finite (Value) then
            raise OpenCV.OpenCV_Error with "Fundamental entries must be finite";
         end if;
      end loop;
      for Value of First_Point loop
         if not Is_Finite (Value) then
            raise OpenCV.OpenCV_Error with "First epipolar point must be finite";
         end if;
      end loop;
      for Value of Second_Point loop
         if not Is_Finite (Value) then
            raise OpenCV.OpenCV_Error with "Second epipolar point must be finite";
         end if;
      end loop;
      for I in 0 .. 2 loop
         L2 (I) := Sum3 (Matrix (I, 0), First_Point (0),
                        Matrix (I, 1), First_Point (1), Matrix (I, 2));
         L1 (I) := Sum3 (Matrix (0, I), Second_Point (0),
                        Matrix (1, I), Second_Point (1), Matrix (2, I));
      end loop;
      N1 := Hypot (L1 (0), L1 (1));
      N2 := Hypot (L2 (0), L2 (1));
      if not Is_Finite (N1) or else not Is_Finite (N2) or else
        N1 = 0.0 or else N2 = 0.0
      then
         return (Defined => False);
      end if;
      D1 := abs Sum3 (L1 (0), First_Point (0), L1 (1), First_Point (1), L1 (2)) / N1;
      D2 := abs Sum3 (L2 (0), Second_Point (0), L2 (1), Second_Point (1), L2 (2)) / N2;
      if not Is_Finite (D1) or else not Is_Finite (D2) then
         return (Defined => False);
      end if;
      return (Defined => True, Maximum_Error_Pixels => Scalar'Max (D1, D2));
   exception
      when Constraint_Error => return (Defined => False);
   end Maximum_Epipolar_Error;

   function Map_With_Homography
     (Matrix : Homography_Matrix; Point : Image_Point) return Homography_Point_Result
   is
      NX, NY, W : OpenCV.Float64_Value;
      Value : Image_Point;
   begin
      for Coefficient of Matrix loop
         if not Is_Finite (Coefficient) then
            raise OpenCV.OpenCV_Error with "Homography entries must be finite";
         end if;
      end loop;
      for Component of Point loop
         if not Is_Finite (Component) then
            raise OpenCV.OpenCV_Error with "Homography point must be finite";
         end if;
      end loop;
      NX := Matrix (0, 0) * Point (0) + Matrix (0, 1) * Point (1) + Matrix (0, 2);
      NY := Matrix (1, 0) * Point (0) + Matrix (1, 1) * Point (1) + Matrix (1, 2);
      W := Matrix (2, 0) * Point (0) + Matrix (2, 1) * Point (1) + Matrix (2, 2);
      if not Is_Finite (NX) or else not Is_Finite (NY) or else
        not Is_Finite (W) or else W = 0.0
      then
         return (Finite => False);
      end if;
      Value := [NX / W, NY / W];
      if not Is_Finite (Value (0)) or else not Is_Finite (Value (1)) then
         return (Finite => False);
      end if;
      return (Finite => True, Point => Value);
   exception
      when Constraint_Error => return (Finite => False);
   end Map_With_Homography;

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

   function Decompose_Calibrated_Homography
     (Matrix : Homography_Matrix; Intrinsics : Camera_Intrinsics)
      return Planar_Motion_Hypothesis_Array
   is
      H : aliased C.C_Homography :=
        (Interfaces.C.double (Matrix (0, 0)), Interfaces.C.double (Matrix (0, 1)),
         Interfaces.C.double (Matrix (0, 2)), Interfaces.C.double (Matrix (1, 0)),
         Interfaces.C.double (Matrix (1, 1)), Interfaces.C.double (Matrix (1, 2)),
         Interfaces.C.double (Matrix (2, 0)), Interfaces.C.double (Matrix (2, 1)),
         Interfaces.C.double (Matrix (2, 2)));
      K : aliased C.C_Camera_Intrinsics := To_C (Intrinsics);
      Native : aliased C.C_Planar_Decomposition;
   begin
      Validate (Intrinsics);
      C.Check (C.Decompose_Homography (H'Access, K'Access, Native'Access),
               "Decompose_Calibrated_Homography");
      if Native.Count < 0 or else Native.Count > 4 then
         raise OpenCV.OpenCV_Error with "Malformed planar hypothesis count";
      end if;
      return Result : Planar_Motion_Hypothesis_Array (1 .. Natural (Native.Count)) do
         for I in Result'Range loop
            declare
               V : constant C.C_Planar_Motion := Native.Candidates (I);
               Values : constant array (Positive range 1 .. 15) of Interfaces.C.double :=
                 [V.R00, V.R01, V.R02, V.R10, V.R11, V.R12, V.R20, V.R21, V.R22,
                  V.TX, V.TY, V.TZ, V.NX, V.NY, V.NZ];
            begin
               for X of Values loop
                  if not Is_Finite (OpenCV.Float64_Value (X)) then
                     raise OpenCV.OpenCV_Error with "Nonfinite planar hypothesis";
                  end if;
               end loop;
               for Row in 0 .. 2 loop
                  for Col in 0 .. 2 loop
                     Result (I).Rotation_First_To_Second (Row, Col) :=
                       OpenCV.Float64_Value (Values (Row * 3 + Col + 1));
                  end loop;
                  Result (I).Translation_Over_Plane_Distance (Row) :=
                    OpenCV.Float64_Value (Values (10 + Row));
                  Result (I).Plane_Normal_In_First (Row) :=
                    OpenCV.Float64_Value (Values (13 + Row));
               end loop;
               Result (I).Pure_Rotation :=
                 (for all J in 10 .. 15 => OpenCV.Float64_Value (Values (J)) = 0.0);
            end;
         end loop;
      end return;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Unrepresentable planar decomposition";
   end Decompose_Calibrated_Homography;

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

   function Estimate_Homography_RANSAC
     (Source_Points, Destination_Points : Image_Point_Array;
      Options : Homography_RANSAC_Options := (others => <>))
      return Homography_Estimate
   is
      type Result_Guard is new Ada.Finalization.Limited_Controlled with record
         Handle : aliased System.Address := System.Null_Address;
      end record;
      overriding procedure Finalize (Self : in out Result_Guard);
      overriding procedure Finalize (Self : in out Result_Guard) is
      begin
         C.Homography_Result_Destroy (Self.Handle);
         Self.Handle := System.Null_Address;
      end Finalize;
      Guard : Result_Guard;
      Source, Destination : OpenCV.Core.Mat;
      Native_Options : aliased C.C_Homography_Options;
      Native_Found : aliased Interfaces.Unsigned_8 := 0;
      Native_Count : aliased Interfaces.Integer_32 := 0;
      Matrix : aliased C.C_Homography := (others => 0.0);
      Code : C.Status := C.Success;
      procedure Source_Callback (Source_Handle : Bridge.Input_Mat_Handle) is
         procedure Destination_Callback (Destination_Handle : Bridge.Input_Mat_Handle) is
         begin
            Code := C.Find_Homography_RANSAC
              (Source_Handle, Destination_Handle, Native_Options'Access, Guard.Handle'Access);
         end Destination_Callback;
      begin
         Bridge.With_Input_Handle (Destination, Destination_Callback'Access);
      end Source_Callback;
   begin
      --  Share the qualified numeric policy, not PnP's estimator or minimum.
      Validate (RANSAC_Options'(Options.Maximum_Iterations,
                Options.Reprojection_Threshold_Pixels, Options.Confidence));
      Validate (Source_Points);
      Validate (Destination_Points);
      if Source_Points'Length /= Destination_Points'Length or else
        Source_Points'Length < 5 or else
        Source_Points'Length > Interfaces.Integer_32'Last
      then
         raise OpenCV.OpenCV_Error with "Robust homography requires equal counts >=5 fitting INT32";
      end if;
      Source := Image_Matrix (Source_Points);
      Destination := Image_Matrix (Destination_Points);
      Native_Options := (Interfaces.Integer_32 (Options.Maximum_Iterations),
                         Interfaces.C.double (Options.Reprojection_Threshold_Pixels),
                         Interfaces.C.double (Options.Confidence));
      Bridge.With_Input_Handle (Source, Source_Callback'Access);
      C.Check (Code, "Calib3D.Estimate_Homography_RANSAC");
      if Guard.Handle = System.Null_Address then
         raise OpenCV.OpenCV_Error with "Native homography did not publish a result";
      end if;
      C.Check (C.Homography_Result_Found (Guard.Handle, Native_Found'Access), "Homography.Found");
      if Native_Found = 0 then
         return Estimate : Homography_Estimate do
            null;
         end return;
      end if;
      C.Check (C.Homography_Result_Matrix (Guard.Handle, Matrix'Access), "Homography.Matrix");
      C.Check (C.Homography_Result_Inlier_Count (Guard.Handle, Native_Count'Access), "Homography.Count");
      if Native_Count < 4 or else Native_Count > Interfaces.Integer_32 (Source_Points'Length) then
         raise OpenCV.OpenCV_Error with "Invalid native homography inlier count";
      end if;
      return Estimate : Homography_Estimate do
         Estimate.Value :=
           [[OpenCV.Float64_Value (Matrix.H00), OpenCV.Float64_Value (Matrix.H01), OpenCV.Float64_Value (Matrix.H02)],
            [OpenCV.Float64_Value (Matrix.H10), OpenCV.Float64_Value (Matrix.H11), OpenCV.Float64_Value (Matrix.H12)],
            [OpenCV.Float64_Value (Matrix.H20), OpenCV.Float64_Value (Matrix.H21), OpenCV.Float64_Value (Matrix.H22)]];
         for Coefficient of Estimate.Value loop
            if not Is_Finite (Coefficient) then
               raise OpenCV.OpenCV_Error with "Native homography is nonfinite";
            end if;
         end loop;
         Estimate.Data := new Inlier_Index_Array (1 .. Natural (Native_Count));
         for I in Estimate.Data.all'Range loop
            declare
               Native_Index : aliased Interfaces.Integer_32 := 0;
            begin
               C.Check (C.Homography_Result_Inlier
                 (Guard.Handle, Interfaces.Integer_32 (I - 1), Native_Index'Access), "Homography.Inlier");
               if Native_Index < 0 or else Native_Index >= Interfaces.Integer_32 (Source_Points'Length) then
                  raise OpenCV.OpenCV_Error with "Invalid native homography correspondence index";
               end if;
               Estimate.Data (I) := Positive (Native_Index + 1);
               if I > 1 and then Estimate.Data (I) <= Estimate.Data (I - 1) then
                  raise OpenCV.OpenCV_Error with "Homography inliers are not strictly ascending";
               end if;
            end;
         end loop;
         Estimate.Has_Model := True;
      end return;
   end Estimate_Homography_RANSAC;

   function Estimate_Fundamental_RANSAC
     (First_Points, Second_Points : Image_Point_Array;
      Options : Fundamental_RANSAC_Options := (others => <>))
      return Fundamental_Estimate
   is
      type Result_Guard is new Ada.Finalization.Limited_Controlled with record
         Handle : aliased System.Address := System.Null_Address;
      end record;
      overriding procedure Finalize (Self : in out Result_Guard);
      overriding procedure Finalize (Self : in out Result_Guard) is
      begin
         C.Fundamental_Result_Destroy (Self.Handle);
         Self.Handle := System.Null_Address;
      end Finalize;
      Guard : Result_Guard;
      First, Second : OpenCV.Core.Mat;
      Native_Options : aliased C.C_Fundamental_Options;
      Native_Found : aliased Interfaces.Unsigned_8 := 0;
      Native_Count : aliased Interfaces.Integer_32 := 0;
      Matrix : aliased C.C_Fundamental := (others => 0.0);
      Code : C.Status := C.Success;
      procedure First_Callback (First_Handle : Bridge.Input_Mat_Handle) is
         procedure Second_Callback (Second_Handle : Bridge.Input_Mat_Handle) is
         begin
            Code := C.Find_Fundamental_RANSAC
              (First_Handle, Second_Handle, Native_Options'Access, Guard.Handle'Access);
         end Second_Callback;
      begin
         Bridge.With_Input_Handle (Second, Second_Callback'Access);
      end First_Callback;
   begin
      Native_Options := (Interfaces.C.double (Options.Epipolar_Threshold_Pixels),
                         Interfaces.C.double (Options.Confidence));
      C.Check (C.Validate_Fundamental_Options (Native_Options'Access), "Fundamental.Options");
      Validate (First_Points);
      Validate (Second_Points);
      if First_Points'Length /= Second_Points'Length or else
        First_Points'Length < 15 or else
        First_Points'Length > Interfaces.Integer_32'Last
      then
         raise OpenCV.OpenCV_Error with "Robust fundamental requires equal counts >=15 fitting INT32";
      end if;
      First := Image_Matrix (First_Points);
      Second := Image_Matrix (Second_Points);
      Bridge.With_Input_Handle (First, First_Callback'Access);
      C.Check (Code, "Calib3D.Estimate_Fundamental_RANSAC");
      if Guard.Handle = System.Null_Address then
         raise OpenCV.OpenCV_Error with "Native fundamental did not publish a result";
      end if;
      C.Check (C.Fundamental_Result_Found (Guard.Handle, Native_Found'Access), "Fundamental.Found");
      if Native_Found = 0 then
         return Estimate : Fundamental_Estimate do
            null;
         end return;
      end if;
      C.Check (C.Fundamental_Result_Matrix (Guard.Handle, Matrix'Access), "Fundamental.Matrix");
      C.Check (C.Fundamental_Result_Inlier_Count (Guard.Handle, Native_Count'Access), "Fundamental.Count");
      if Native_Count < 7 or else Native_Count > Interfaces.Integer_32 (First_Points'Length) then
         raise OpenCV.OpenCV_Error with "Invalid native fundamental inlier count";
      end if;
      return Estimate : Fundamental_Estimate do
         Estimate.Value :=
           [[OpenCV.Float64_Value (Matrix.F00), OpenCV.Float64_Value (Matrix.F01), OpenCV.Float64_Value (Matrix.F02)],
            [OpenCV.Float64_Value (Matrix.F10), OpenCV.Float64_Value (Matrix.F11), OpenCV.Float64_Value (Matrix.F12)],
            [OpenCV.Float64_Value (Matrix.F20), OpenCV.Float64_Value (Matrix.F21), OpenCV.Float64_Value (Matrix.F22)]];
         for Coefficient of Estimate.Value loop
            if not Is_Finite (Coefficient) then
               raise OpenCV.OpenCV_Error with "Native fundamental is nonfinite";
            end if;
         end loop;
         Estimate.Data := new Inlier_Index_Array (1 .. Natural (Native_Count));
         for I in Estimate.Data.all'Range loop
            declare
               Native_Index : aliased Interfaces.Integer_32 := 0;
            begin
               C.Check (C.Fundamental_Result_Inlier
                 (Guard.Handle, Interfaces.Integer_32 (I - 1), Native_Index'Access), "Fundamental.Inlier");
               if Native_Index < 0 or else Native_Index >= Interfaces.Integer_32 (First_Points'Length) then
                  raise OpenCV.OpenCV_Error with "Invalid native fundamental correspondence index";
               end if;
               Estimate.Data (I) := Positive (Native_Index + 1);
               if I > 1 and then Estimate.Data (I) <= Estimate.Data (I - 1) then
                  raise OpenCV.OpenCV_Error with "Fundamental inliers are not strictly ascending";
               end if;
            end;
         end loop;
         --  The public classification is calculated in Ada with the caller's
         --  original coordinates; the raw ABI independently offers the same
         --  metric. Do not make native Float32 mask rounding public policy.
         declare
            Indices : Inlier_Index_Array (1 .. First_Points'Length);
            Count : Natural := 0;
         begin
            for I in 1 .. First_Points'Length loop
               declare
                  Error : constant Epipolar_Error_Result := Maximum_Epipolar_Error
                    (Estimate.Value, First_Points (First_Points'First + I - 1),
                     Second_Points (Second_Points'First + I - 1));
               begin
                  if Error.Defined and then
                    Error.Maximum_Error_Pixels <= Options.Epipolar_Threshold_Pixels
                  then
                     Count := Count + 1;
                     Indices (Count) := I;
                  end if;
               end;
            end loop;
            Free (Estimate.Data);
            if Count >= 7 then
               Estimate.Data := new Inlier_Index_Array'(Indices (1 .. Count));
               Estimate.Has_Model := True;
            else
               Estimate.Value := [others => [others => 0.0]];
            end if;
         end;
      end return;
   end Estimate_Fundamental_RANSAC;

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
   overriding procedure Finalize (Self : in out Essential_Estimate) is
   begin
      Free (Self.Data);
      Free (Self.Pose_Data);
      Self.Has_Model := False;
      Self.Has_Pose := False;
      Self.Value := [others => [others => 0.0]];
      Self.Pose_Value := (Rotation_First_To_Second => [others => [others => 0.0]],
                          Translation_Direction => [others => 0.0]);
   end Finalize;
   function Found (Estimate : Essential_Estimate) return Boolean is (Estimate.Has_Model);
   function Essential (Estimate : Essential_Estimate) return Essential_Matrix is
   begin
      if not Estimate.Has_Model then
         raise OpenCV.OpenCV_Error with "Essential requested from a no-E estimate";
      end if;
      return Estimate.Value;
   end Essential;
   function Inlier_Count (Estimate : Essential_Estimate) return Natural is
     (if Estimate.Data = null then 0 else Estimate.Data.all'Length);
   function Inlier (Estimate : Essential_Estimate; Index : Positive) return Positive is
   begin
      if Estimate.Data = null or else Index not in Estimate.Data.all'Range then
         raise OpenCV.OpenCV_Error with "Essential inlier index out of range";
      end if;
      return Estimate.Data (Index);
   end Inlier;
   function Inliers (Estimate : Essential_Estimate) return Inlier_Index_Array is
   begin
      if Estimate.Data = null then
         return [1 .. 0 => <>];
      end if;
      return Estimate.Data.all;
   end Inliers;
   function Pose_Recovered (Estimate : Essential_Estimate) return Boolean is (Estimate.Has_Pose);
   function Recovered_Pose (Estimate : Essential_Estimate) return Relative_Camera_Pose is
   begin
      if not Estimate.Has_Pose then
         raise OpenCV.OpenCV_Error with "Relative pose requested from a no-pose estimate";
      end if;
      return Estimate.Pose_Value;
   end Recovered_Pose;
   function Pose_Inlier_Count (Estimate : Essential_Estimate) return Natural is
     (if Estimate.Pose_Data = null then 0 else Estimate.Pose_Data.all'Length);
   function Pose_Inlier (Estimate : Essential_Estimate; Index : Positive) return Positive is
   begin
      if Estimate.Pose_Data = null or else Index not in Estimate.Pose_Data.all'Range then
         raise OpenCV.OpenCV_Error with "Relative pose inlier index out of range";
      end if;
      return Estimate.Pose_Data (Index);
   end Pose_Inlier;
   function Pose_Inliers (Estimate : Essential_Estimate) return Inlier_Index_Array is
   begin
      if Estimate.Pose_Data = null then
         return [1 .. 0 => <>];
      end if;
      return Estimate.Pose_Data.all;
   end Pose_Inliers;

   function Normalized_Sampson_Error
     (Matrix : Essential_Matrix; First_Point, Second_Point : Normalized_Image_Point)
      return Normalized_Sampson_Error_Result
   is
      Native : aliased constant C.C_Essential :=
        (Interfaces.C.double (Matrix (0, 0)), Interfaces.C.double (Matrix (0, 1)),
         Interfaces.C.double (Matrix (0, 2)), Interfaces.C.double (Matrix (1, 0)),
         Interfaces.C.double (Matrix (1, 1)), Interfaces.C.double (Matrix (1, 2)),
         Interfaces.C.double (Matrix (2, 0)), Interfaces.C.double (Matrix (2, 1)),
         Interfaces.C.double (Matrix (2, 2)));
      Defined : aliased Interfaces.Unsigned_8 := 0;
      Error : aliased Interfaces.C.double := 0.0;
   begin
      C.Check (C.Normalized_Sampson_Error
        (Native'Access, Interfaces.C.double (First_Point (0)), Interfaces.C.double (First_Point (1)),
         Interfaces.C.double (Second_Point (0)), Interfaces.C.double (Second_Point (1)),
         Defined'Access, Error'Access), "Calib3D.Normalized_Sampson_Error");
      if Defined = 0 then
         return (Defined => False);
      end if;
      return (Defined => True, Error => OpenCV.Float64_Value (Error));
   exception
      when Constraint_Error => return (Defined => False);
   end Normalized_Sampson_Error;

   function Second_Camera_Center_Direction_In_First
     (Pose : Relative_Camera_Pose) return Camera_Direction
   is
      Center : Object_Point;
   begin
      for Value of Pose.Rotation_First_To_Second loop
         if not Is_Finite (Value) then
            raise OpenCV.OpenCV_Error with "Relative rotation must be finite";
         end if;
      end loop;
      Center := Rotate (Pose.Rotation_First_To_Second,
                        Object_Point (Pose.Translation_Direction), Transpose => True);
      for Axis in 0 .. 2 loop
         Center (Axis) := -Center (Axis);
      end loop;
      return Camera_Direction (Unit_Vector (Center));
   end Second_Camera_Center_Direction_In_First;

   function Estimate_Essential_RANSAC
     (First_Points, Second_Points : Normalized_Image_Point_Array;
      Options : Essential_RANSAC_Options := (others => <>)) return Essential_Estimate
   is
      type Result_Guard is new Ada.Finalization.Limited_Controlled with record
         Handle : aliased System.Address := System.Null_Address;
      end record;
      overriding procedure Finalize (Self : in out Result_Guard);
      overriding procedure Finalize (Self : in out Result_Guard) is
      begin
         C.Essential_Result_Destroy (Self.Handle);
         Self.Handle := System.Null_Address;
      end Finalize;
      Guard : Result_Guard;
      First, Second : OpenCV.Core.Mat;
      Native_Options : aliased constant C.C_Essential_Options :=
        (Interfaces.C.double (Options.Normalized_Epipolar_Threshold),
         Interfaces.C.double (Options.Confidence));
      Native_Found, Native_Pose_Found : aliased Interfaces.Unsigned_8 := 0;
      Native_Count, Native_Pose_Count : aliased Interfaces.Integer_32 := 0;
      Matrix : aliased C.C_Essential := (others => 0.0);
      Pose : aliased C.C_Relative_Pose := (others => 0.0);
      Code : C.Status := C.Success;
      function Snapshot (Points : Normalized_Image_Point_Array) return OpenCV.Core.Mat is
         Values : Image_Point_Array (Points'Range);
      begin
         for I in Points'Range loop
            Values (I) := Image_Point (Points (I));
         end loop;
         Validate (Values);
         return Image_Matrix (Values);
      end Snapshot;
      procedure First_Callback (First_Handle : Bridge.Input_Mat_Handle) is
         procedure Second_Callback (Second_Handle : Bridge.Input_Mat_Handle) is
         begin
            Code := C.Find_Essential_RANSAC
              (First_Handle, Second_Handle, Native_Options'Access, Guard.Handle'Access);
         end Second_Callback;
      begin
         Bridge.With_Input_Handle (Second, Second_Callback'Access);
      end First_Callback;
      procedure Read_Indices (Data : in out Inlier_Buffer; Count : Interfaces.Integer_32;
                              For_Pose : Boolean) is
      begin
         if Count < 5 or else Count > Interfaces.Integer_32 (First_Points'Length) then
            raise OpenCV.OpenCV_Error with "Invalid Essential/pose inlier count";
         end if;
         Data := new Inlier_Index_Array (1 .. Natural (Count));
         for I in Data.all'Range loop
            declare
               Index : aliased Interfaces.Integer_32 := 0;
            begin
               if For_Pose then
                  C.Check (C.Essential_Result_Pose_Inlier
                    (Guard.Handle, Interfaces.Integer_32 (I - 1), Index'Access), "Essential.Pose_Inlier");
               else
                  C.Check (C.Essential_Result_Inlier
                    (Guard.Handle, Interfaces.Integer_32 (I - 1), Index'Access), "Essential.Inlier");
               end if;
               if Index < 0 or else Index >= Interfaces.Integer_32 (First_Points'Length) then
                  raise OpenCV.OpenCV_Error with "Invalid Essential correspondence index";
               end if;
               Data (I) := Positive (Index + 1);
               if I > 1 and then Data (I) <= Data (I - 1) then
                  raise OpenCV.OpenCV_Error with "Essential indices not strictly ascending";
               end if;
            end;
         end loop;
      end Read_Indices;
   begin
      C.Check (C.Validate_Essential_Options (Native_Options'Access), "Essential.Options");
      if First_Points'Length /= Second_Points'Length or else First_Points'Length < 6 or else
        First_Points'Length > Interfaces.Integer_32'Last
      then
         raise OpenCV.OpenCV_Error with "Essential RANSAC requires equal counts >=6 fitting INT32";
      end if;
      First := Snapshot (First_Points);
      Second := Snapshot (Second_Points);
      Bridge.With_Input_Handle (First, First_Callback'Access);
      C.Check (Code, "Calib3D.Estimate_Essential_RANSAC");
      if Guard.Handle = System.Null_Address then
         raise OpenCV.OpenCV_Error with "Native Essential did not publish a result";
      end if;
      C.Check (C.Essential_Result_Found (Guard.Handle, Native_Found'Access), "Essential.Found");
      if Native_Found = 0 then
         return Estimate : Essential_Estimate do
            null;
         end return;
      end if;
      C.Check (C.Essential_Result_Matrix (Guard.Handle, Matrix'Access), "Essential.Matrix");
      C.Check (C.Essential_Result_Inlier_Count (Guard.Handle, Native_Count'Access), "Essential.Count");
      C.Check (C.Essential_Result_Pose_Found (Guard.Handle, Native_Pose_Found'Access), "Essential.Pose_Found");
      return Estimate : Essential_Estimate do
         Estimate.Value :=
           [[OpenCV.Float64_Value (Matrix.E00), OpenCV.Float64_Value (Matrix.E01), OpenCV.Float64_Value (Matrix.E02)],
            [OpenCV.Float64_Value (Matrix.E10), OpenCV.Float64_Value (Matrix.E11), OpenCV.Float64_Value (Matrix.E12)],
            [OpenCV.Float64_Value (Matrix.E20), OpenCV.Float64_Value (Matrix.E21), OpenCV.Float64_Value (Matrix.E22)]];
         Read_Indices (Estimate.Data, Native_Count, False);
         Estimate.Has_Model := True;
         if Native_Pose_Found /= 0 then
            C.Check (C.Essential_Result_Pose (Guard.Handle, Pose'Access), "Essential.Pose");
            C.Check (C.Essential_Result_Pose_Inlier_Count
              (Guard.Handle, Native_Pose_Count'Access), "Essential.Pose_Count");
            Estimate.Pose_Value :=
              (Rotation_First_To_Second =>
                [[OpenCV.Float64_Value (Pose.R00), OpenCV.Float64_Value (Pose.R01), OpenCV.Float64_Value (Pose.R02)],
                 [OpenCV.Float64_Value (Pose.R10), OpenCV.Float64_Value (Pose.R11), OpenCV.Float64_Value (Pose.R12)],
                 [OpenCV.Float64_Value (Pose.R20), OpenCV.Float64_Value (Pose.R21), OpenCV.Float64_Value (Pose.R22)]],
               Translation_Direction => Camera_Direction (Unit_Vector
                 ([OpenCV.Float64_Value (Pose.TX), OpenCV.Float64_Value (Pose.TY), OpenCV.Float64_Value (Pose.TZ)])));
            Read_Indices (Estimate.Pose_Data, Native_Pose_Count, True);
            for Index of Estimate.Pose_Data.all loop
               declare
                  Present : Boolean := False;
               begin
                  for Essential_Index of Estimate.Data.all loop
                     Present := Present or else Index = Essential_Index;
                  end loop;
                  if not Present then
                     raise OpenCV.OpenCV_Error with "Pose inlier not an Essential inlier";
                  end if;
               end;
            end loop;
            Estimate.Has_Pose := True;
         end if;
      end return;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Essential conversion exceeds representable range";
   end Estimate_Essential_RANSAC;
   procedure Validate_Relative_Rotation (Pose : Relative_Camera_Pose) is
      R : Rotation_Matrix renames Pose.Rotation_First_To_Second;
      Dot, Det : OpenCV.Float64_Value;
   begin
      for V of R loop
         if not Is_Finite (V) or else abs (V) > 1.000000001 then
            raise OpenCV.OpenCV_Error with "Invalid relative rotation";
         end if;
      end loop;
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            Dot := 0.0;
            for K in 0 .. 2 loop
               Dot := Dot + R (K, I) * R (K, J);
            end loop;
            if abs (Dot - (if I = J then 1.0 else 0.0)) > 1.0E-9 then
               raise OpenCV.OpenCV_Error with "Relative rotation is not orthogonal";
            end if;
         end loop;
      end loop;
      Det := R (0, 0) * (R (1, 1) * R (2, 2) - R (1, 2) * R (2, 1)) -
        R (0, 1) * (R (1, 0) * R (2, 2) - R (1, 2) * R (2, 0)) +
        R (0, 2) * (R (1, 0) * R (2, 1) - R (1, 1) * R (2, 0));
      if abs (Det - 1.0) > 1.0E-9 then
         raise OpenCV.OpenCV_Error with "Relative rotation determinant is not +1";
      end if;
   end Validate_Relative_Rotation;

   procedure Validate_Relative_Pose (Pose : Relative_Camera_Pose) is
      T : constant Object_Point := Unit_Vector (Object_Point (Pose.Translation_Direction));
   begin
      Validate_Vector (T);
      Validate_Relative_Rotation (Pose);
   end Validate_Relative_Pose;

   function Measure_Stereo_Parallax
     (First_Points, Second_Points : Normalized_Image_Point_Array;
      Pose : Relative_Camera_Pose) return Stereo_Parallax_Array
   is
   begin
      Validate_Relative_Pose (Pose);
      if First_Points'Length /= Second_Points'Length then
         raise OpenCV.OpenCV_Error with "Parallax requires equal point counts";
      end if;
      return Result : Stereo_Parallax_Array (1 .. First_Points'Length) do
         for I in Result'Range loop
            declare
               P : Normalized_Image_Point renames First_Points (First_Points'First + I - 1);
               Q : Normalized_Image_Point renames Second_Points (Second_Points'First + I - 1);
               A : constant Object_Point := Unit_Vector ([P (0), P (1), 1.0]);
               B : constant Object_Point := Unit_Vector (Rotate
                 (Pose.Rotation_First_To_Second, Unit_Vector ([Q (0), Q (1), 1.0]), True));
               Cross : constant Object_Point :=
                 [A (1) * B (2) - A (2) * B (1),
                  A (2) * B (0) - A (0) * B (2),
                  A (0) * B (1) - A (1) * B (0)];
               Dot : OpenCV.Float64_Value := 0.0;
               Scale : OpenCV.Float64_Value := 0.0;
               Squared : OpenCV.Float64_Value := 0.0;
               Norm : OpenCV.Float64_Value := 0.0;
            begin
               Validate_Vector (Cross);
               for Axis in 0 .. 2 loop
                  Dot := Dot + A (Axis) * B (Axis);
                  if not Is_Finite (Dot) then
                     raise OpenCV.OpenCV_Error with "Nonfinite bearing dot product";
                  end if;
                  Scale := OpenCV.Float64_Value'Max (Scale, abs Cross (Axis));
               end loop;
               --  Scale even the bounded cross product: squaring a tiny
               --  component directly could underflow and erase its angle.
               if Scale /= 0.0 then
                  for Component of Cross loop
                     Squared := Squared + (Component / Scale) ** 2;
                  end loop;
                  Norm := Scale * Math.Sqrt (Squared);
               end if;
               if not Is_Finite (Norm) then
                  raise OpenCV.OpenCV_Error with "Nonfinite bearing cross norm";
               end if;
               Result (I) := (Math.Arctan (Norm, Dot), Math.Arctan (Norm, abs Dot));
               if not Is_Finite (Result (I).Forward_Ray_Angle_Radians) or else
                 not Is_Finite (Result (I).Acute_Line_Angle_Radians)
               then
                  raise OpenCV.OpenCV_Error with "Nonfinite parallax angle";
               end if;
            end;
         end loop;
      end return;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Parallax exceeds Float64 range";
   end Measure_Stereo_Parallax;

   function Assess_Triangulation
     (First_Points, Second_Points : Normalized_Image_Point_Array;
      Pose : Relative_Camera_Pose;
      Points : Triangulated_Point_Array;
      Options : Triangulation_Quality_Options := (others => <>))
      return Triangulation_Quality_Array
   is
      Angles : constant Stereo_Parallax_Array :=
        Measure_Stereo_Parallax (First_Points, Second_Points, Pose);
   begin
      if Points'Length /= Angles'Length then
         raise OpenCV.OpenCV_Error with "Assessment requires equal point counts";
      end if;
      if not Is_Finite (Options.Minimum_Acute_Parallax_Radians) or else
        Options.Minimum_Acute_Parallax_Radians < 0.0 or else
        Options.Minimum_Acute_Parallax_Radians > Math.Arctan (1.0, 0.0) or else
        not Is_Finite (Options.Maximum_Normalized_Reprojection_Error) or else
        Options.Maximum_Normalized_Reprojection_Error < 0.0
      then
         raise OpenCV.OpenCV_Error with "Invalid triangulation quality options";
      end if;
      return Result : Triangulation_Quality_Array (Angles'Range) do
         for I in Result'Range loop
            declare
               P : Triangulated_Point renames Points (Points'First + I - 1);
               Residual_OK : Boolean := False;
               Angle_OK : constant Boolean := Angles (I).Acute_Line_Angle_Radians >=
                 Options.Minimum_Acute_Parallax_Radians;
            begin
               if P.Status = Usable then
                  Validate_Vector (P.Position_In_First_Camera);
                  if not Is_Finite (P.First_Depth) or else P.First_Depth <= 0.0 or else
                    not Is_Finite (P.Second_Depth) or else P.Second_Depth <= 0.0 or else
                    not Is_Finite (P.First_Normalized_Error) or else P.First_Normalized_Error < 0.0 or else
                    not Is_Finite (P.Second_Normalized_Error) or else P.Second_Normalized_Error < 0.0
                  then
                     raise OpenCV.OpenCV_Error with "Invalid manually constructed usable point";
                  end if;
                  Residual_OK := P.First_Normalized_Error <= Options.Maximum_Normalized_Reprojection_Error
                    and then P.Second_Normalized_Error <= Options.Maximum_Normalized_Reprojection_Error;
               end if;
               Result (I) := (P.Status, Angles (I), Angle_OK, Residual_OK,
                 P.Status = Usable and then Angle_OK and then Residual_OK);
            end;
         end loop;
      end return;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Assessment exceeds representable range";
   end Assess_Triangulation;

   function Refine_Triangulated_Points
     (First_Points, Second_Points : Normalized_Image_Point_Array;
      Pose : Relative_Camera_Pose;
      Initial : Triangulated_Point_Array;
      Options : Triangulation_Refinement_Options := (others => <>))
      return Point_Refinement_Result_Array
   is
      package Kernel renames OpenCV.Calib3D.Internal.Point_Refinement;
      Angles : constant Stereo_Parallax_Array :=
        Measure_Stereo_Parallax (First_Points, Second_Points, Pose);
      Fixed : constant Relative_Camera_Pose :=
        (Pose.Rotation_First_To_Second,
         Camera_Direction (Unit_Vector (Object_Point (Pose.Translation_Direction))));
   begin
      if Initial'Length /= First_Points'Length then
         raise OpenCV.OpenCV_Error with "Refinement requires equal point counts";
      end if;
      if Options.Maximum_Iterations > 100 or else
        not Is_Finite (Options.Minimum_Acute_Parallax_Radians) or else
        Options.Minimum_Acute_Parallax_Radians < 0.0 or else
        Options.Minimum_Acute_Parallax_Radians > Math.Arctan (1.0, 0.0)
      then
         raise OpenCV.OpenCV_Error with "Invalid refinement options";
      end if;
      return Result : Point_Refinement_Result_Array (1 .. Initial'Length) do
         for I in Result'Range loop
            declare
               P : Triangulated_Point renames Initial (Initial'First + I - 1);
               Depth_Available : Boolean := True;
               Q : Object_Point;
            begin
               Result (I) := (Outcome => Skipped_Unusable, Point => P, others => <>);
               if P.Status = Usable then
                  Validate_Vector (P.Position_In_First_Camera);
                  if not Is_Finite (P.First_Depth) or else P.First_Depth <= 0.0 or else
                    not Is_Finite (P.Second_Depth) or else P.Second_Depth <= 0.0 or else
                    not Is_Finite (P.First_Normalized_Error) or else P.First_Normalized_Error < 0.0 or else
                    not Is_Finite (P.Second_Normalized_Error) or else P.Second_Normalized_Error < 0.0
                  then
                     raise OpenCV.OpenCV_Error with "Invalid manually constructed usable point";
                  end if;
                  if P.Position_In_First_Camera (2) <= 0.0 then
                     raise OpenCV.OpenCV_Error with "Usable point has nonpositive actual first depth";
                  end if;
                  begin
                     Q := Kernel.Second_Position (P.Position_In_First_Camera, Fixed);
                     if Q (2) <= 0.0 then
                        raise OpenCV.OpenCV_Error with "Usable point has nonpositive actual second depth";
                     end if;
                  exception
                     when Constraint_Error => Depth_Available := False;
                  end;
                  if not Depth_Available then
                     Result (I).Outcome := Numerically_Unavailable;
                  elsif Options.Minimum_Acute_Parallax_Radians > 0.0 and then
                    Angles (I).Acute_Line_Angle_Radians < Options.Minimum_Acute_Parallax_Radians
                  then
                     Result (I).Outcome := Skipped_Low_Parallax;
                  else
                     Result (I) := Kernel.Refine (P,
                       First_Points (First_Points'First + I - 1),
                       Second_Points (Second_Points'First + I - 1), Fixed,
                       Options.Maximum_Iterations);
                  end if;
               end if;
            end;
         end loop;
      end return;
   end Refine_Triangulated_Points;

   function Triangulate_Normalized
     (First_Points, Second_Points : Normalized_Image_Point_Array;
      Pose : Relative_Camera_Pose) return Triangulated_Point_Array
   is
      type Result_Guard is new Ada.Finalization.Limited_Controlled with record
         Handle : aliased System.Address := System.Null_Address;
      end record;
      overriding procedure Finalize (Self : in out Result_Guard);
      overriding procedure Finalize (Self : in out Result_Guard) is
      begin
         C.Triangulation_Result_Destroy (Self.Handle);
      end Finalize;
      Guard : Result_Guard;
      First, Second : OpenCV.Core.Mat;
      R : Rotation_Matrix renames Pose.Rotation_First_To_Second;
      T : constant Object_Point := Unit_Vector (Object_Point (Pose.Translation_Direction));
      Native_Pose : aliased constant C.C_Relative_Pose :=
        (Interfaces.C.double (R (0, 0)), Interfaces.C.double (R (0, 1)), Interfaces.C.double (R (0, 2)),
         Interfaces.C.double (R (1, 0)), Interfaces.C.double (R (1, 1)), Interfaces.C.double (R (1, 2)),
         Interfaces.C.double (R (2, 0)), Interfaces.C.double (R (2, 1)), Interfaces.C.double (R (2, 2)),
         Interfaces.C.double (T (0)), Interfaces.C.double (T (1)), Interfaces.C.double (T (2)));
      Count : aliased Interfaces.Integer_32 := 0;
      Point : aliased C.C_Triangulated_Point;
      Code : C.Status := C.Success;
      function Snapshot (Points : Normalized_Image_Point_Array) return OpenCV.Core.Mat is
         Values : Image_Point_Array (Points'Range);
      begin
         for I in Points'Range loop
            Values (I) := Image_Point (Points (I));
         end loop;
         Validate (Values);
         return Image_Matrix (Values);
      end Snapshot;
      procedure First_Callback (First_Handle : Bridge.Input_Mat_Handle) is
         procedure Second_Callback (Second_Handle : Bridge.Input_Mat_Handle) is
         begin
            Code := C.Triangulate_Normalized
              (First_Handle, Second_Handle, Native_Pose'Access, Guard.Handle'Access);
         end Second_Callback;
      begin
         Bridge.With_Input_Handle (Second, Second_Callback'Access);
      end First_Callback;
   begin
      if First_Points'Length /= Second_Points'Length or else
        First_Points'Length > Interfaces.Integer_32'Last
      then
         raise OpenCV.OpenCV_Error with "Triangulation requires equal counts fitting INT32";
      end if;
      Validate_Relative_Rotation (Pose);
      if First_Points'Length = 0 then
         return [1 .. 0 => <>];
      end if;
      First := Snapshot (First_Points);
      Second := Snapshot (Second_Points);
      Bridge.With_Input_Handle (First, First_Callback'Access);
      C.Check (Code, "Triangulate_Normalized");
      C.Check (C.Triangulation_Result_Count (Guard.Handle, Count'Access), "Triangulation.Count");
      if Count /= Interfaces.Integer_32 (First_Points'Length) then
         raise OpenCV.OpenCV_Error with "Invalid triangulation result count";
      end if;
      return Result : Triangulated_Point_Array (1 .. Natural (Count)) do
         for I in Result'Range loop
            C.Check (C.Triangulation_Result_Point
              (Guard.Handle, Interfaces.Integer_32 (I - 1), Point'Access), "Triangulation.Point");
            case Point.Point_Status is
               when 0 =>
                  Result (I) := (Status => Usable,
                    Position_In_First_Camera => [OpenCV.Float64_Value (Point.X),
                      OpenCV.Float64_Value (Point.Y), OpenCV.Float64_Value (Point.Z)],
                    First_Depth => OpenCV.Float64_Value (Point.Depth_First),
                    Second_Depth => OpenCV.Float64_Value (Point.Depth_Second),
                    First_Normalized_Error => OpenCV.Float64_Value (Point.Error_First),
                    Second_Normalized_Error => OpenCV.Float64_Value (Point.Error_Second));
               when 1 => Result (I) := (Status => At_Infinity);
               when 2 => Result (I) := (Status => Unrepresentable_Point);
               when 3 => Result (I) := (Status => Non_Positive_Depth);
               when 4 => Result (I) := (Status => Undefined_Reprojection);
               when others => raise OpenCV.OpenCV_Error with "Unknown triangulation status";
            end case;
         end loop;
      end return;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Triangulation conversion exceeds representable range";
   end Triangulate_Normalized;
end OpenCV.Calib3D;
