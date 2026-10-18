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
with OpenCV.Calib3D.Internal.Refinement_Policy;
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

   procedure Validate_Planar_Hypothesis
     (Value : Planar_Motion_Hypothesis; Count : Natural)
   is
      R : constant Rotation_Matrix := Value.Rotation_First_To_Second;
      T : constant Translation_Vector := Value.Translation_Over_Plane_Distance;
      N : constant Camera_Direction := Value.Plane_Normal_In_First;
      Tolerance : constant OpenCV.Float64_Value :=
        (if Value.Pure_Rotation then 0.001 else 1.0E-9);
      Norm_Squared : OpenCV.Float64_Value := 0.0;
      Determinant : OpenCV.Float64_Value;
   begin
      for X of R loop
         if not Is_Finite (X) or else abs X > 2.0 then
            raise OpenCV.OpenCV_Error with "Invalid planar rotation";
         end if;
      end loop;
      for I in 0 .. 2 loop
         if not Is_Finite (T (I)) or else not Is_Finite (N (I)) or else abs N (I) > 2.0 then
            raise OpenCV.OpenCV_Error with "Invalid planar translation/normal";
         end if;
         Norm_Squared := Norm_Squared + N (I) * N (I);
         for J in 0 .. 2 loop
            declare
               Dot : OpenCV.Float64_Value := 0.0;
            begin
               for K in 0 .. 2 loop
                  Dot := Dot + R (K, I) * R (K, J);
               end loop;
               if abs (Dot - (if I = J then 1.0 else 0.0)) > Tolerance then
                  raise OpenCV.OpenCV_Error with "Invalid planar rotation orthogonality";
               end if;
            end;
         end loop;
      end loop;
      Determinant := R (0, 0) * (R (1, 1) * R (2, 2) - R (1, 2) * R (2, 1))
        - R (0, 1) * (R (1, 0) * R (2, 2) - R (1, 2) * R (2, 0))
        + R (0, 2) * (R (1, 0) * R (2, 1) - R (1, 1) * R (2, 0));
      if abs (Determinant - 1.0) > Tolerance then
         raise OpenCV.OpenCV_Error with "Invalid planar rotation determinant";
      end if;
      if Value.Pure_Rotation then
         if Count /= 1 or else (for some X of T => X /= 0.0) or else
           (for some X of N => X /= 0.0)
         then
            raise OpenCV.OpenCV_Error with "Contradictory pure-rotation hypothesis";
         end if;
      elsif (for all X of T => X = 0.0) or else abs (Math.Sqrt (Norm_Squared) - 1.0) > 1.0E-9 then
         raise OpenCV.OpenCV_Error with "Invalid general planar hypothesis";
      end if;
   end Validate_Planar_Hypothesis;

   function Assess_Planar_Visibility
     (Hypotheses : Planar_Motion_Hypothesis_Array;
      First_Points, Second_Points : Normalized_Image_Point_Array;
      Selected_Indices : Inlier_Index_Array)
      return Planar_Hypothesis_Visibility_Array
   is
      Native : aliased C.C_Planar_Decomposition :=
        (Count => 0, Candidates => [others => (others => 0.0)]);
      Output : aliased C.C_Planar_Visibility;
      First, Second : OpenCV.Core.Mat;
      Code : C.Status := C.Success;
      Previous : Natural := 0;
      Position : Natural := 0;
      procedure Validate_Points (Points : Normalized_Image_Point_Array) is
      begin
         for P of Points loop
            for X of P loop
               if not Is_Finite (X) then
                  raise OpenCV.OpenCV_Error with "Nonfinite visibility observation";
               end if;
            end loop;
         end loop;
      end Validate_Points;
      procedure First_Callback (A : Bridge.Input_Mat_Handle) is
         procedure Second_Callback (B : Bridge.Input_Mat_Handle) is
         begin
            Code := C.Filter_Planar_Visibility (Native'Access, A, B, Output'Access);
         end Second_Callback;
      begin
         Bridge.With_Input_Handle (Second, Second_Callback'Access);
      end First_Callback;
   begin
      if Hypotheses'Length > 4 or else First_Points'Length /= Second_Points'Length or else
        First_Points'Length > Natural (Interfaces.Integer_32'Last)
      then
         raise OpenCV.OpenCV_Error with "Invalid visibility collection lengths";
      end if;
      Validate_Points (First_Points);
      Validate_Points (Second_Points);
      for Index of Selected_Indices loop
         if Index > First_Points'Length or else Index <= Previous then
            raise OpenCV.OpenCV_Error with "Invalid visibility selected position/order";
         end if;
         Previous := Index;
      end loop;
      for V of Hypotheses loop
         Validate_Planar_Hypothesis (V, Hypotheses'Length);
         Position := Position + 1;
         declare
            R : constant Rotation_Matrix := V.Rotation_First_To_Second;
            T : constant Translation_Vector := V.Translation_Over_Plane_Distance;
            N : constant Camera_Direction := V.Plane_Normal_In_First;
         begin
            Native.Candidates (Position) :=
              (Interfaces.C.double (R (0, 0)), Interfaces.C.double (R (0, 1)), Interfaces.C.double (R (0, 2)),
               Interfaces.C.double (R (1, 0)), Interfaces.C.double (R (1, 1)), Interfaces.C.double (R (1, 2)),
               Interfaces.C.double (R (2, 0)), Interfaces.C.double (R (2, 1)), Interfaces.C.double (R (2, 2)),
               Interfaces.C.double (T (0)), Interfaces.C.double (T (1)), Interfaces.C.double (T (2)),
               Interfaces.C.double (N (0)), Interfaces.C.double (N (1)), Interfaces.C.double (N (2)));
         end;
      end loop;
      Native.Count := Interfaces.Integer_32 (Hypotheses'Length);
      return Results : Planar_Hypothesis_Visibility_Array (1 .. Hypotheses'Length) do
         if Hypotheses'Length = 1 and then Hypotheses (Hypotheses'First).Pure_Rotation then
            Results (1).Disposition := Not_Applicable_Pure_Rotation;
         elsif Selected_Indices'Length = 0 then
            Results := [others => (Disposition => No_Selected_References)];
         elsif Hypotheses'Length /= 0 then
            First := OpenCV.Core.Create (Selected_Indices'Length, 1, (OpenCV.Core.Float64, 2));
            Second := OpenCV.Core.Create (Selected_Indices'Length, 1, (OpenCV.Core.Float64, 2));
            Position := 0;
            for Index of Selected_Indices loop
               Vec2_Access.Set (First, Position, 0, Image_Point (First_Points (First_Points'First + (Index - 1))));
               Vec2_Access.Set (Second, Position, 0, Image_Point (Second_Points (Second_Points'First + (Index - 1))));
               Position := Position + 1;
            end loop;
            Bridge.With_Input_Handle (First, First_Callback'Access);
            C.Check (Code, "Assess_Planar_Visibility");
            if Output.Count /= Native.Count then
               raise OpenCV.OpenCV_Error with "Malformed visibility result count";
            end if;
            for I in Results'Range loop
               if Output.Accepted (I) > 1 then
                  raise OpenCV.OpenCV_Error with "Malformed visibility flag";
               end if;
               Results (I).Disposition :=
                 (if Output.Accepted (I) = 1 then Passes_Visibility else Rejected_By_Visibility);
            end loop;
         end if;
      end return;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Unrepresentable visibility input";
   end Assess_Planar_Visibility;

   function Assess_Planar_Visibility
     (Hypotheses : Planar_Motion_Hypothesis_Array;
      First_Points, Second_Points : Normalized_Image_Point_Array)
      return Planar_Hypothesis_Visibility_Array
   is
   begin
      if First_Points'Length > Natural (Interfaces.Integer_32'Last) then
         raise OpenCV.OpenCV_Error with "Visibility point count exceeds INT32";
      end if;
      declare
         Selected : Inlier_Index_Array (1 .. First_Points'Length);
      begin
         for I in Selected'Range loop
            Selected (I) := I;
         end loop;
         return Assess_Planar_Visibility (Hypotheses, First_Points, Second_Points, Selected);
      end;
   end Assess_Planar_Visibility;

   function Visibility_Passing_Hypothesis_Indices
     (Results : Planar_Hypothesis_Visibility_Array) return Inlier_Index_Array
   is
      Indices : Inlier_Index_Array (1 .. Results'Length);
      Count : Natural := 0;
   begin
      for I in Results'Range loop
         if Results (I).Disposition = Passes_Visibility then
            Count := Count + 1;
            Indices (Count) := I - Results'First + 1;
         end if;
      end loop;
      return Indices (1 .. Count);
   end Visibility_Passing_Hypothesis_Indices;

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

   function Compose_Poses
     (First : World_To_Camera_Pose; Second : World_To_Camera_Pose)
      return World_To_Camera_Pose
   is
      Native_First  : aliased C.C_Pose;
      Native_Second : aliased C.C_Pose;
      Native_Result : aliased C.C_Pose := (others => 0.0);
   begin
      Validate (First);
      Validate (Second);
      Native_First := To_C (First);
      Native_Second := To_C (Second);
      C.Check (C.Compose_RT (Native_First'Access, Native_Second'Access, Native_Result'Access),
               "Calib3D.Compose_Poses");
      return Result : constant World_To_Camera_Pose := From_C (Native_Result) do
         Validate (Result);
      end return;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Pose composition exceeds Float64 range";
   end Compose_Poses;

   function Checked_Difference
     (Left, Right : OpenCV.Float64_Value) return OpenCV.Float64_Value
   is
      Result : OpenCV.Float64_Value;
   begin
      Result := Left - Right;
      if not Is_Finite (Result) then
         raise OpenCV.OpenCV_Error with "Pose difference exceeds Float64 range";
      end if;
      return Result;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Pose difference exceeds Float64 range";
   end Checked_Difference;

   function Checked_Sum
     (Left, Right : OpenCV.Float64_Value) return OpenCV.Float64_Value
   is
      Result : OpenCV.Float64_Value;
   begin
      Result := Left + Right;
      if not Is_Finite (Result) then
         raise OpenCV.OpenCV_Error with "Pose sum exceeds Float64 range";
      end if;
      return Result;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Pose sum exceeds Float64 range";
   end Checked_Sum;

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

   --  Scaled Euclidean norm. A zero vector returns exactly 0.0. Components
   --  are scaled before squaring, and Scale * Sqrt is rejected when the
   --  mathematical norm itself cannot be represented.
   function Scaled_Vector_Norm (Value : Object_Point) return OpenCV.Float64_Value is
      Scale : OpenCV.Float64_Value := 0.0;
      Sum   : OpenCV.Float64_Value := 0.0;
      Norm  : OpenCV.Float64_Value;
   begin
      Validate_Vector (Value);
      for Component of Value loop
         Scale := OpenCV.Float64_Value'Max (Scale, abs Component);
      end loop;
      if Scale = 0.0 then
         return 0.0;
      end if;
      for Component of Value loop
         declare
            Scaled : constant OpenCV.Float64_Value := Component / Scale;
         begin
            Sum := Checked_Sum (Sum, Scaled * Scaled);
         end;
      end loop;
      Norm := Math.Sqrt (Sum);
      if not Is_Finite (Norm) or else Norm < 0.0 then
         raise OpenCV.OpenCV_Error with "Baseline norm is not representable";
      end if;
      if Norm = 0.0 then
         return 0.0;
      end if;
      declare
         Result : constant OpenCV.Float64_Value := Scale * Norm;
      begin
         if not Is_Finite (Result) then
            raise OpenCV.OpenCV_Error with "Baseline magnitude exceeds Float64 range";
         end if;
         return Result;
      end;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Baseline magnitude exceeds Float64 range";
   end Scaled_Vector_Norm;

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

   --  Arithmetic-only overload: callers can keep native Rodrigues failures
   --  outside a per-point numerical-unavailability handler.
   function Camera_To_World_Point
     (Pose : World_To_Camera_Pose; Point : Object_Point;
      Matrix : Rotation_Matrix) return Object_Point
   is
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

   function Camera_To_World_Point
     (Pose : World_To_Camera_Pose; Point : Object_Point) return Object_Point
   is
      Matrix : constant Rotation_Matrix := Rotation_Matrix_Of (Pose);
   begin
      return Camera_To_World_Point (Pose, Point, Matrix);
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

   function Invert_Pose (Pose : World_To_Camera_Pose) return World_To_Camera_Pose is
      Matrix   : Rotation_Matrix;
      Center   : Object_Point;
      Negated  : World_To_Camera_Pose;
      Composed : World_To_Camera_Pose;
   begin
      Validate (Pose);
      --  Native camera center is the inverse translation -R^T * t.
      Center := Camera_Center (Pose);
      Validate_Vector (Center);
      Matrix := Rotation_Matrix_Of (Pose);
      --  A negated Rodrigues vector represents the inverse rotation only when
      --  native Rodrigues of that vector reproduces R^T.
      Negated :=
        (Rotation => [-Pose.Rotation (0), -Pose.Rotation (1), -Pose.Rotation (2)],
         Translation => Center);
      Validate (Negated);
      declare
         Negated_Matrix : constant Rotation_Matrix := Rotation_Matrix_Of (Negated);
      begin
         for Row in 0 .. 2 loop
            for Col in 0 .. 2 loop
               if not Is_Finite (Negated_Matrix (Row, Col))
                 or else abs (Negated_Matrix (Row, Col) - Matrix (Col, Row)) > 1.0E-12
               then
                  raise OpenCV.OpenCV_Error with
                    "Negated Rodrigues vector does not reproduce the inverse rotation";
               end if;
            end loop;
         end loop;
      end;
      --  Publish through native composeRT rather than an Ada-built vector.
      Composed := Compose_Poses
        ((Rotation => Negated.Rotation, Translation => [others => 0.0]),
         (Rotation => [others => 0.0], Translation => Center));
      for Axis in 0 .. 2 loop
         --  Composition with zero translation adds Center without scaling.
         if Composed.Translation (Axis) /= Center (Axis) then
            raise OpenCV.OpenCV_Error with "Inverse translation was not preserved";
         end if;
      end loop;
      Validate (Composed);
      declare
         Published_Matrix : constant Rotation_Matrix := Rotation_Matrix_Of (Composed);
      begin
         for Row in 0 .. 2 loop
            for Col in 0 .. 2 loop
               --  Rotation entries are bounded by one, so an absolute
               --  dimensionless tolerance is appropriate here.
               if abs (Published_Matrix (Row, Col) - Matrix (Col, Row)) > 1.0E-12 then
                  raise OpenCV.OpenCV_Error with "Published inverse rotation disagrees with transpose";
               end if;
            end loop;
         end loop;
      end;
      return Composed;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Pose inversion exceeds Float64 range";
   end Invert_Pose;

   function Camera_To_Camera_Pose
     (First, Second : World_To_Camera_Pose) return World_To_Camera_Pose
   is
   begin
      Validate (First);
      Validate (Second);
      --  Apply the inverse of First, then Second.
      return Compose_Poses (Invert_Pose (First), Second);
   end Camera_To_Camera_Pose;

   function Measure_Camera_Motion
     (First, Second : World_To_Camera_Pose) return Camera_Motion_Result
   is
      Relative : World_To_Camera_Pose;
      C1, C2, Displacement, In_First : Object_Point;
      Baseline : OpenCV.Float64_Value;
   begin
      Validate (First);
      Validate (Second);
      Relative := Camera_To_Camera_Pose (First, Second);
      C1 := Camera_Center (First);
      C2 := Camera_Center (Second);
      Validate_Vector (C1);
      Validate_Vector (C2);
      for Axis in 0 .. 2 loop
         Displacement (Axis) := Checked_Difference (C2 (Axis), C1 (Axis));
      end loop;
      Validate_Vector (Displacement);
      --  Second center expressed in the first camera frame.
      In_First := Rotate (Rotation_Matrix_Of (First), Displacement);
      Baseline := Scaled_Vector_Norm (Displacement);
      if Displacement (0) = 0.0 and then Displacement (1) = 0.0
        and then Displacement (2) = 0.0
      then
         return (Baseline_Direction_Defined => False,
                 Relative_Pose => Relative,
                 First_Center_World => C1,
                 Second_Center_World => C2,
                 Second_Center_In_First => In_First,
                 Displacement_World => Displacement,
                 Baseline_Distance => Baseline);
      end if;
      return (Baseline_Direction_Defined => True,
              Relative_Pose => Relative,
              First_Center_World => C1,
              Second_Center_World => C2,
              Second_Center_In_First => In_First,
              Displacement_World => Displacement,
              Baseline_Distance => Baseline,
              Baseline_Direction_World => Unit_Vector (Displacement));
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Camera motion exceeds Float64 range";
   end Measure_Camera_Motion;

   function Essential_From_Camera_Poses
     (First, Second : World_To_Camera_Pose) return Known_Pose_Essential_Result
   is
      Motion : constant Camera_Motion_Result := Measure_Camera_Motion (First, Second);
   begin
      if Motion.Baseline_Distance = 0.0 then
         return (Defined => False, Relative_Pose => Motion.Relative_Pose,
                 Baseline_Distance => 0.0);
      end if;
      declare
         --  Unit_Vector scales BEFORE squaring; it rejects an exact zero term.
         T : constant Object_Point := Unit_Vector (Motion.Relative_Pose.Translation);
         R : constant Rotation_Matrix := Rotation_Matrix_Of (Motion.Relative_Pose);
         E : Essential_Matrix;
      begin
         for Col in 0 .. 2 loop
            E (0, Col) := Checked_Difference (T (1) * R (2, Col), T (2) * R (1, Col));
            E (1, Col) := Checked_Difference (T (2) * R (0, Col), T (0) * R (2, Col));
            E (2, Col) := Checked_Difference (T (0) * R (1, Col), T (1) * R (0, Col));
         end loop;
         return (Defined => True, Relative_Pose => Motion.Relative_Pose,
                 Baseline_Distance => Motion.Baseline_Distance,
                 Rotation_First_To_Second => R,
                 Translation_Direction => Camera_Direction (T), Matrix => E);
      end;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Known-pose Essential arithmetic unavailable";
   end Essential_From_Camera_Poses;

   function Disposition (Result : Known_Pose_Epipolar_Result)
      return Epipolar_Verification_Disposition is (Result.State);
   function Accepted (Result : Known_Pose_Epipolar_Result) return Boolean is
     (Result.State = Accepted);
   function Geometry_Evaluated (Result : Known_Pose_Epipolar_Result) return Boolean is
     (Result.Has_Geometry);
   function Geometry (Result : Known_Pose_Epipolar_Result) return Known_Pose_Essential_Result is
   begin
      if not Result.Has_Geometry then
         raise OpenCV.OpenCV_Error with "Known-pose geometry not evaluated";
      end if;
      return Result.Geometry_Value;
   end Geometry;
   function Correspondence_Errors (Result : Known_Pose_Epipolar_Result)
      return Epipolar_Correspondence_Result_Array is (Result.Errors);
   function Entire_Set_Evaluable (Result : Known_Pose_Epipolar_Result) return Boolean is
     (Result.Has_Summary);
   function Summary (Result : Known_Pose_Epipolar_Result) return Epipolar_Verification_Summary is
   begin
      if not Result.Has_Summary then
         raise OpenCV.OpenCV_Error with "Whole epipolar set not evaluable";
      end if;
      return Result.Summary_Value;
   end Summary;

   function Verify_Known_Pose_Epipolar
     (First_Pixels, Second_Pixels : Image_Point_Array;
      First_Intrinsics, Second_Intrinsics : Camera_Intrinsics;
      First_Distortion, Second_Distortion : Distortion_Coefficients;
      First_Pose, Second_Pose : World_To_Camera_Pose;
      Options : Epipolar_Verification_Options) return Known_Pose_Epipolar_Result
   is
   begin
      Validate (First_Pixels); Validate (Second_Pixels);
      Validate (First_Intrinsics); Validate (Second_Intrinsics);
      Validate (First_Distortion); Validate (Second_Distortion);
      Validate (First_Pose); Validate (Second_Pose);
      if First_Pixels'Length /= Second_Pixels'Length or else
        First_Pixels'Length > Natural (Interfaces.Integer_32'Last)
      then
         raise OpenCV.OpenCV_Error with "Invalid epipolar correspondence counts";
      end if;
      if not Is_Finite (Options.Maximum_RMS_Normalized_Sampson) or else
        Options.Maximum_RMS_Normalized_Sampson < 0.0 or else
        not Is_Finite (Options.Maximum_Point_Normalized_Sampson) or else
        Options.Maximum_Point_Normalized_Sampson < 0.0
      then
         raise OpenCV.OpenCV_Error with "Invalid normalized Sampson policy";
      end if;
      return Result : Known_Pose_Epipolar_Result (First_Pixels'Length) do
         if First_Pixels'Length >= Options.Minimum_Correspondences then
            Result.Geometry_Value := Essential_From_Camera_Poses (First_Pose, Second_Pose);
            Result.Has_Geometry := True;
            if not Result.Geometry_Value.Defined then
               Result.State := No_Physical_Baseline;
            else
               declare
                  First : constant Normalized_Image_Point_Array := Undistort_To_Normalized
                    (First_Pixels, First_Intrinsics, First_Distortion);
                  Second : constant Normalized_Image_Point_Array := Undistort_To_Normalized
                    (Second_Pixels, Second_Intrinsics, Second_Distortion);
                  Scale : OpenCV.Float64_Value := 0.0;
                  SSQ : OpenCV.Float64_Value := 1.0;
                  All_Defined, All_Pass : Boolean := True;
               begin
                  for I in 1 .. Result.Count loop
                     declare
                        V : constant Normalized_Sampson_Error_Result :=
                          Normalized_Sampson_Error (Result.Geometry_Value.Matrix, First (I), Second (I));
                     begin
                        if not V.Defined then
                           All_Defined := False;
                        else
                           if not Is_Finite (V.Error) or else V.Error < 0.0 then
                              raise OpenCV.OpenCV_Error with "Invalid native Sampson metric";
                           end if;
                           Result.Errors (I) := (Defined => True, Error => V.Error,
                             Passes_Point_Limit => V.Error <= Options.Maximum_Point_Normalized_Sampson);
                           All_Pass := All_Pass and then Result.Errors (I).Passes_Point_Limit;
                           if V.Error > 0.0 then
                              if Scale < V.Error then
                                 SSQ := 1.0 + SSQ * (Scale / V.Error) ** 2;
                                 Scale := V.Error;
                              else
                                 SSQ := SSQ + (V.Error / Scale) ** 2;
                              end if;
                           end if;
                        end if;
                     end;
                  end loop;
                  if not All_Defined then
                     Result.State := Undefined_Correspondence_Errors;
                  else
                     Result.Summary_Value :=
                       (RMS_Normalized_Sampson => Scale * Math.Sqrt
                          (OpenCV.Float64_Value'Min (1.0, SSQ / OpenCV.Float64_Value (Result.Count))),
                        Maximum_Normalized_Sampson => Scale);
                     if not Is_Finite (Result.Summary_Value.RMS_Normalized_Sampson) then
                        raise OpenCV.OpenCV_Error with "Epipolar RMS unavailable";
                     end if;
                     Result.Has_Summary := True;
                     Result.State :=
                       (if All_Pass and then
                          Result.Summary_Value.RMS_Normalized_Sampson <= Options.Maximum_RMS_Normalized_Sampson
                          and then Scale <= Options.Maximum_Point_Normalized_Sampson
                        then Accepted else Rejected_By_Policy);
                  end if;
               end;
            end if;
         end if;
      end return;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Known-pose epipolar arithmetic unavailable";
   end Verify_Known_Pose_Epipolar;

   function Verify_Known_Pose_Epipolar
     (First_Pixels, Second_Pixels : Image_Point_Array;
      First_Intrinsics, Second_Intrinsics : Camera_Intrinsics;
      First_Distortion, Second_Distortion : Distortion_Coefficients;
      First, Second : Camera_Relocalization_Result;
      Options : Epipolar_Verification_Options) return Known_Pose_Epipolar_Result
   is
   begin
      if not Accepted (First) or else not Accepted (Second) then
         raise OpenCV.OpenCV_Error with "Pair verification requires two accepted relocalizations";
      end if;
      return Verify_Known_Pose_Epipolar
        (First_Pixels, Second_Pixels, First_Intrinsics, Second_Intrinsics,
         First_Distortion, Second_Distortion, Accepted_Pose (First), Accepted_Pose (Second), Options);
   end Verify_Known_Pose_Epipolar;

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

   procedure Validate
     (Plane : World_Plane; Options : World_Plane_Intersection_Options)
   is
      Normal : constant Object_Point := Unit_Vector (Object_Point (Plane.Normal));
      pragma Unreferenced (Normal);
   begin
      Validate_Vector (Plane.Anchor);
      if not Is_Finite (Options.Minimum_Absolute_Incidence_Cosine)
        or else Options.Minimum_Absolute_Incidence_Cosine not in 0.0 .. 1.0
        or else not Is_Finite (Options.Minimum_Forward_Range)
        or else Options.Minimum_Forward_Range < 0.0
        or else not Is_Finite (Options.Maximum_Forward_Range)
        or else Options.Maximum_Forward_Range <= 0.0
        or else Options.Maximum_Forward_Range < Options.Minimum_Forward_Range
        or else not Is_Finite (Options.Maximum_Absolute_Plane_Residual)
        or else Options.Maximum_Absolute_Plane_Residual < 0.0
      then
         raise OpenCV.OpenCV_Error with "Invalid world-plane intersection policy";
      end if;
   end Validate;

   function Intersect_World_Rays_With_Plane
     (Rays : World_Ray_Array; Plane : World_Plane;
      Options : World_Plane_Intersection_Options)
      return World_Plane_Intersection_Array
   is
      Normal : constant Object_Point := Unit_Vector (Object_Point (Plane.Normal));
      Directions : Object_Point_Array (1 .. Rays'Length);
      --  Only these local, arithmetic-only helpers raise this exception.
      --  No native call, input validation, or accessor is inside its handler.
      Arithmetic_Unavailable : exception;
      function Finite_Value (Value : OpenCV.Float64_Value) return OpenCV.Float64_Value is
      begin
         if not Is_Finite (Value) then
            raise Arithmetic_Unavailable;
         end if;
         return Value;
      end Finite_Value;
      function Difference (Left, Right : OpenCV.Float64_Value) return OpenCV.Float64_Value is
      begin
         return Finite_Value (Left - Right);
      exception
         when Constraint_Error => raise Arithmetic_Unavailable;
      end Difference;
      function Sum (Left, Right : OpenCV.Float64_Value) return OpenCV.Float64_Value is
      begin
         return Finite_Value (Left + Right);
      exception
         when Constraint_Error => raise Arithmetic_Unavailable;
      end Sum;
      function Product (Left, Right : OpenCV.Float64_Value) return OpenCV.Float64_Value is
         Value : OpenCV.Float64_Value;
      begin
         Value := Finite_Value (Left * Right);
         if Left /= 0.0 and then Right /= 0.0 and then Value = 0.0 then
            raise Arithmetic_Unavailable;
         end if;
         return Value;
      exception
         when Constraint_Error => raise Arithmetic_Unavailable;
      end Product;
      function Quotient (Left, Right : OpenCV.Float64_Value) return OpenCV.Float64_Value is
         Value : OpenCV.Float64_Value;
      begin
         Value := Finite_Value (Left / Right);
         --  A nonzero distance must not silently become an at-origin result.
         if Left /= 0.0 and then Value = 0.0 then
            raise Arithmetic_Unavailable;
         end if;
         return Value;
      exception
         when Constraint_Error => raise Arithmetic_Unavailable;
      end Quotient;
      function Dot (Left, Right : Object_Point) return OpenCV.Float64_Value is
         Value : OpenCV.Float64_Value := 0.0;
      begin
         for Axis in 0 .. 2 loop
            Value := Sum (Value, Product (Left (Axis), Right (Axis)));
         end loop;
         return Value;
      end Dot;
   begin
      Validate (Plane, Options);
      --  Validate the whole batch before evaluating any intersection.
      for I in Directions'Range loop
         Validate_Vector (Rays (Rays'First + (I - 1)).Origin);
         Directions (I) := Unit_Vector (Object_Point (Rays (Rays'First + (I - 1)).Direction));
      end loop;
      return Results : World_Plane_Intersection_Array (1 .. Rays'Length) do
         for I in Results'Range loop
            declare
               Ray : World_Ray renames Rays (Rays'First + (I - 1));
               Result : World_Plane_Intersection renames Results (I);
               Offset, Position : Object_Point;
               Denominator, Numerator, Distance : OpenCV.Float64_Value;
            begin
               Denominator := Dot (Normal, Directions (I));
               Result.Incidence := abs Denominator;
               Result.Has_Incidence := True;
               for Axis in 0 .. 2 loop
                  Offset (Axis) := Difference (Plane.Anchor (Axis), Ray.Origin (Axis));
               end loop;
               Numerator := Dot (Normal, Offset);
               if Denominator = 0.0 then
                  Result.State := (if Numerator = 0.0 then Coplanar_Ray else Parallel_Ray);
               else
                  Distance := Quotient (Numerator, Denominator);
                  Result.Distance := Distance;
                  Result.Has_Distance := True;
                  if Distance = 0.0 then
                     Result.State := Intersection_At_Origin;
                  elsif Distance < 0.0 then
                     Result.State := Intersection_Behind_Ray;
                  elsif Result.Incidence < Options.Minimum_Absolute_Incidence_Cosine then
                     Result.State := Grazing_Incidence_Rejected;
                  elsif Distance < Options.Minimum_Forward_Range then
                     Result.State := Below_Minimum_Forward_Range;
                  elsif Distance > Options.Maximum_Forward_Range then
                     Result.State := Above_Maximum_Forward_Range;
                  else
                     for Axis in 0 .. 2 loop
                        Position (Axis) := Sum
                          (Ray.Origin (Axis), Product (Distance, Directions (I) (Axis)));
                        Offset (Axis) := Difference (Position (Axis), Plane.Anchor (Axis));
                     end loop;
                     Result.Residual := abs Dot (Normal, Offset);
                     Result.Has_Residual := True;
                     if Result.Residual > Options.Maximum_Absolute_Plane_Residual then
                        Result.State := Plane_Residual_Rejected;
                     else
                        Result.Position := Position;
                        Result.State := Forward_Intersection_Accepted;
                     end if;
                  end if;
               end if;
            exception
               when Arithmetic_Unavailable => Results (I).State := Numerically_Unavailable;
            end;
         end loop;
      end return;
   end Intersect_World_Rays_With_Plane;

   function Unproject_Pixels_Onto_World_Plane
     (Pixels : Image_Point_Array; Intrinsics : Camera_Intrinsics;
      Distortion : Distortion_Coefficients; Pose : World_To_Camera_Pose;
      Plane : World_Plane; Options : World_Plane_Intersection_Options)
      return World_Plane_Intersection_Array
   is
   begin
      Validate (Plane, Options);
      return Intersect_World_Rays_With_Plane
        (World_Bearing_Rays (Pixels, Intrinsics, Distortion, Pose), Plane, Options);
   end Unproject_Pixels_Onto_World_Plane;

   function Disposition (Result : World_Plane_Intersection)
      return World_Plane_Intersection_Disposition is (Result.State);
   function Incidence_Evaluated (Result : World_Plane_Intersection) return Boolean is
     (Result.Has_Incidence);
   function Has_Signed_Ray_Distance (Result : World_Plane_Intersection) return Boolean is
     (Result.Has_Distance);
   function Plane_Residual_Evaluated (Result : World_Plane_Intersection) return Boolean is
     (Result.Has_Residual);
   function Accepted (Result : World_Plane_Intersection) return Boolean is
     (Result.State = Forward_Intersection_Accepted);
   function Absolute_Incidence_Cosine (Result : World_Plane_Intersection)
      return OpenCV.Float64_Value is
   begin
      if not Result.Has_Incidence then
         raise OpenCV.OpenCV_Error with "World-plane incidence unavailable";
      end if;
      return Result.Incidence;
   end Absolute_Incidence_Cosine;
   function Signed_Ray_Distance (Result : World_Plane_Intersection)
      return OpenCV.Float64_Value is
   begin
      if not Result.Has_Distance then
         raise OpenCV.OpenCV_Error with "World-plane distance unavailable";
      end if;
      return Result.Distance;
   end Signed_Ray_Distance;
   function Absolute_Plane_Residual (Result : World_Plane_Intersection)
      return OpenCV.Float64_Value is
   begin
      if not Result.Has_Residual then
         raise OpenCV.OpenCV_Error with "World-plane residual unavailable";
      end if;
      return Result.Residual;
   end Absolute_Plane_Residual;
   function Accepted_World_Position (Result : World_Plane_Intersection)
      return Object_Point is
   begin
      if not Accepted (Result) then
         raise OpenCV.OpenCV_Error with "World-plane intersection not accepted";
      end if;
      return Result.Position;
   end Accepted_World_Position;

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

    function Solve_AP3P_Hypotheses
      (Object_Points : Object_Point_Array;
       Image_Points  : Image_Point_Array;
       Intrinsics    : Camera_Intrinsics;
       Distortion    : Distortion_Coefficients := No_Distortion)
       return AP3P_Pose_Hypothesis_Array
    is
       Native_Intrinsics : aliased C.C_Camera_Intrinsics;
       Native_Distortion : aliased C.C_Distortion5;
       Native            : aliased C.C_AP3P_Hypotheses;
       Objects           : OpenCV.Core.Mat;
       Images            : OpenCV.Core.Mat;
       Code              : C.Status := C.Success;
       procedure Object_Callback (Object_Handle : Bridge.Input_Mat_Handle) is
          procedure Image_Callback (Image_Handle : Bridge.Input_Mat_Handle) is
          begin
             Code := C.Solve_AP3P_Hypotheses
               (Object_Handle, Image_Handle, Native_Intrinsics'Access,
                Native_Distortion'Access, Native'Access);
          end Image_Callback;
       begin
          Bridge.With_Input_Handle (Images, Image_Callback'Access);
       end Object_Callback;
    begin
       Validate (Intrinsics);
       Validate (Distortion);
       Validate (Object_Points);
       Validate (Image_Points);
       if Object_Points'Length /= 3 or else Image_Points'Length /= 3 then
          raise OpenCV.OpenCV_Error with "Solve_AP3P_Hypotheses requires exactly three correspondences";
       end if;
       Objects := Object_Matrix (Object_Points);
       Images := Image_Matrix (Image_Points);
       Native_Intrinsics := To_C (Intrinsics);
       Native_Distortion := To_C (Distortion);
       Bridge.With_Input_Handle (Objects, Object_Callback'Access);
       C.Check (Code, "Calib3D.Solve_AP3P_Hypotheses");
       if Native.Count < 0 or else Native.Count > 4 then
          raise OpenCV.OpenCV_Error with "Malformed AP3P hypothesis count";
       end if;
       return Result : AP3P_Pose_Hypothesis_Array (1 .. Natural (Native.Count)) do
          for I in Result'Range loop
             Result (I).Pose := From_C (Native.Candidates (I));
             Validate (Result (I).Pose);
          end loop;
       end return;
    exception
       when Constraint_Error =>
          raise OpenCV.OpenCV_Error with "Unrepresentable AP3P result";
    end Solve_AP3P_Hypotheses;

    function Pixel_Error
      (Predicted, Observed : Image_Point) return OpenCV.Float64_Value
    is
       DX : OpenCV.Float64_Value;
       DY : OpenCV.Float64_Value;
       Scale : OpenCV.Float64_Value;
       Error : OpenCV.Float64_Value;
       NX, NY, Sum : OpenCV.Float64_Value;
       Residual_Overflow : Boolean := False;
    begin
       begin
          DX := Predicted (0) - Observed (0);
          DY := Predicted (1) - Observed (1);
       exception
          when Constraint_Error =>
             Residual_Overflow := True;
       end;
       if Residual_Overflow or else not (Is_Finite (DX) and then Is_Finite (DY)) then
          raise Constraint_Error;
       end if;
       Scale := OpenCV.Float64_Value'Max (abs DX, abs DY);
       if Scale = 0.0 then
          Error := 0.0;
       else
          NX := DX / Scale;
          NY := DY / Scale;
          Sum := NX * NX + NY * NY;
          Error := Scale * Math.Sqrt (Sum);
       end if;
       if not Is_Finite (Error) then
          raise Constraint_Error;
       end if;
       return Error;
    end Pixel_Error;

    function Evaluate_AP3P_References
      (Hypotheses       : AP3P_Pose_Hypothesis_Array;
       Reference_World  : Object_Point_Array;
       Reference_Pixels : Image_Point_Array;
       Intrinsics       : Camera_Intrinsics;
       Distortion       : Distortion_Coefficients := No_Distortion)
       return AP3P_Reference_Result_Matrix
    is
       function Camera_Coordinate
         (Matrix : Rotation_Matrix; Point : Object_Point;
          Translation : Translation_Vector; Row : Natural)
          return OpenCV.Float64_Value
       is
          Sum : OpenCV.Float64_Value := 0.0;
       begin
          for Column in 0 .. 2 loop
             declare
                Product : OpenCV.Float64_Value;
             begin
                Product := Matrix (Row, Column) * Point (Column);
                if not Is_Finite (Product) then
                   raise Constraint_Error;
                end if;
                Sum := Sum + Product;
                if not Is_Finite (Sum) then
                   raise Constraint_Error;
                end if;
             end;
          end loop;
          Sum := Sum + Translation (Row);
          if not Is_Finite (Sum) then
             raise Constraint_Error;
          end if;
          return Sum;
       end Camera_Coordinate;

       Unavailable : constant AP3P_Reference_Result :=
         (Disposition => Numerically_Unavailable,
          Camera_Depth => 0.0, Error_Pixels => 0.0);
    begin
       if Hypotheses'Length > 4 then
          raise OpenCV.OpenCV_Error
            with "AP3P reference diagnostics accept at most four hypotheses";
       end if;
       if Reference_World'Length /= Reference_Pixels'Length then
          raise OpenCV.OpenCV_Error with "AP3P reference world/pixel counts differ";
       end if;
       Validate (Intrinsics);
       Validate (Distortion);
       for Hypothesis of Hypotheses loop
          Validate (Hypothesis.Pose);
       end loop;
       Validate (Reference_World);
       Validate (Reference_Pixels);
       return Results : AP3P_Reference_Result_Matrix
         (1 .. Hypotheses'Length, 1 .. Reference_World'Length) :=
         [others => [others => Unavailable]]
       do
          if Hypotheses'Length = 0 or else Reference_World'Length = 0 then
             return;
          end if;
          for Hypothesis_Index in Hypotheses'Range loop
             declare
                Pose : constant World_To_Camera_Pose :=
                  Hypotheses (Hypothesis_Index).Pose;
                Matrix : constant Rotation_Matrix := Rotation_Matrix_Of (Pose);
                Row : constant Positive := Hypothesis_Index - Hypotheses'First + 1;
                Eligible_World : Object_Point_Array (1 .. Reference_World'Length);
                Original_Positions : Inlier_Index_Array (Eligible_World'Range);
                Positive_Depths : Reprojection_Error_Array (Eligible_World'Range);
                Eligible_Count : Natural := 0;
             begin
                for Reference_Index in Reference_World'Range loop
                   declare
                      Column : constant Positive :=
                        Reference_Index - Reference_World'First + 1;
                      Point : constant Object_Point :=
                        Reference_World (Reference_Index);
                      Depth : OpenCV.Float64_Value := 0.0;
                      Camera_X, Camera_Y : OpenCV.Float64_Value := 0.0;
                      Transform_Failed : Boolean := False;
                   begin
                      begin
                         Camera_X := Camera_Coordinate (Matrix, Point, Pose.Translation, 0);
                         Camera_Y := Camera_Coordinate (Matrix, Point, Pose.Translation, 1);
                         Depth := Camera_Coordinate (Matrix, Point, Pose.Translation, 2);
                         if not (Is_Finite (Camera_X) and then Is_Finite (Camera_Y)) then
                            raise Constraint_Error;
                         end if;
                      exception
                         when Constraint_Error =>
                            Transform_Failed := True;
                      end;
                      if Transform_Failed then
                         Results (Row, Column) := Unavailable;
                      elsif Depth > 0.0 then
                         Eligible_Count := Eligible_Count + 1;
                         Eligible_World (Eligible_Count) := Point;
                         Original_Positions (Eligible_Count) := Column;
                         Positive_Depths (Eligible_Count) := Depth;
                      else
                         Results (Row, Column) :=
                           (Disposition => Non_Positive_Depth,
                            Camera_Depth => Depth, Error_Pixels => 0.0);
                      end if;
                   end;
                end loop;
                if Eligible_Count > 0 then
                   declare
                      --  Exactly one native projection batch per nonempty
                      --  hypothesis. Keep it outside all per-point handlers:
                      --  native failure aborts the entire owned result.
                      Predicted : constant Image_Point_Array := Project_Points
                        (Eligible_World (1 .. Eligible_Count), Intrinsics, Distortion, Pose);
                   begin
                      for Compact_Position in 1 .. Eligible_Count loop
                         declare
                            Column : constant Positive := Original_Positions (Compact_Position);
                            Error : OpenCV.Float64_Value;
                         begin
                            Error := Pixel_Error
                              (Predicted (Predicted'First + (Compact_Position - 1)),
                               Reference_Pixels (Reference_Pixels'First + (Column - 1)));
                            Results (Row, Column) :=
                              (Disposition => Projected_Positive_Depth,
                               Camera_Depth => Positive_Depths (Compact_Position),
                               Error_Pixels => Error);
                         exception
                            when Constraint_Error =>
                               Results (Row, Column) := Unavailable;
                         end;
                      end loop;
                   end;
                end if;
             end;
          end loop;
       end return;
    end Evaluate_AP3P_References;

    function Summarize_AP3P_References
      (Results : AP3P_Reference_Result_Matrix)
       return AP3P_Hypothesis_Reference_Summary_Array
    is
       procedure Validate_Result (Item : AP3P_Reference_Result) is
       begin
          case Item.Disposition is
             when Projected_Positive_Depth =>
                if not Is_Finite (Item.Camera_Depth) or else Item.Camera_Depth <= 0.0
                  or else not Is_Finite (Item.Error_Pixels) or else Item.Error_Pixels < 0.0
                then
                   raise OpenCV.OpenCV_Error with "Malformed projected AP3P reference";
                end if;
             when Non_Positive_Depth =>
                if not Is_Finite (Item.Camera_Depth) or else Item.Camera_Depth > 0.0
                  or else Item.Error_Pixels /= 0.0
                then
                   raise OpenCV.OpenCV_Error with "Malformed non-positive AP3P reference";
                end if;
             when Numerically_Unavailable =>
                if Item.Camera_Depth /= 0.0 or else Item.Error_Pixels /= 0.0 then
                   raise OpenCV.OpenCV_Error with "Malformed unavailable AP3P reference";
                end if;
          end case;
       end Validate_Result;
    begin
       if Results'Length (1) > 4 or else
         Results'Length (2) > Natural (Interfaces.Integer_32'Last)
       then
          raise OpenCV.OpenCV_Error with "AP3P reference summary exceeds supported bounds";
       end if;
       return Summaries : AP3P_Hypothesis_Reference_Summary_Array (1 .. Results'Length (1)) do
          for Row in Results'Range (1) loop
             declare
                Projected : Natural := 0;
                Behind : Natural := 0;
                Unavailable_Count : Natural := 0;
                Output : constant Positive := Row - Results'First (1) + 1;
             begin
                for Column in Results'Range (2) loop
                   Validate_Result (Results (Row, Column));
                   case Results (Row, Column).Disposition is
                      when Projected_Positive_Depth => Projected := Projected + 1;
                      when Non_Positive_Depth => Behind := Behind + 1;
                      when Numerically_Unavailable => Unavailable_Count := Unavailable_Count + 1;
                   end case;
                end loop;
                declare
                   Errors : Reprojection_Error_Array (1 .. Projected);
                   Next : Natural := 0;
                begin
                   for Column in Results'Range (2) loop
                      if Results (Row, Column).Disposition = Projected_Positive_Depth then
                         Next := Next + 1;
                         Errors (Next) := Results (Row, Column).Error_Pixels;
                      end if;
                   end loop;
                   Summaries (Output) :=
                     (Total_References => Results'Length (2),
                      Projected_References => Projected,
                      Non_Positive_Depth_References => Behind,
                      Numerically_Unavailable_References => Unavailable_Count,
                      Entire_Set_Evaluable =>
                        Results'Length (2) > 0 and then Projected = Results'Length (2),
                      Reprojection => Summarize_Reprojection (Errors));
                end;
             end;
          end loop;
       end return;
    exception
       when Constraint_Error =>
          raise OpenCV.OpenCV_Error with "AP3P reference summary is unrepresentable";
    end Summarize_AP3P_References;

   procedure Validate (Value : AP3P_Selection_Options) is
   begin
      if not (Is_Finite (Value.Maximum_RMS_Error_Pixels)
              and then Is_Finite (Value.Maximum_Point_Error_Pixels)
              and then Is_Finite (Value.Minimum_RMS_Separation_Pixels)
              and then Value.Maximum_RMS_Error_Pixels >= 0.0
              and then Value.Maximum_Point_Error_Pixels >= 0.0
              and then Value.Minimum_RMS_Separation_Pixels >= 0.0)
      then
         raise OpenCV.OpenCV_Error
           with "AP3P selection options must be finite and nonnegative";
      end if;
   end Validate;

   function Same_World_Point (Left, Right : Object_Point) return Boolean is
     (Left (0) = Right (0) and then Left (1) = Right (1) and then Left (2) = Right (2));

   function Selected (Result : AP3P_Selection_Result) return Boolean is
     (Result.Disposition = Unique_Selected_Hypothesis
      and then Result.Candidate_Index in 1 .. Result.Count);

   function Selected_Hypothesis_Index (Result : AP3P_Selection_Result) return Positive is
   begin
      if not Selected (Result) then
         raise OpenCV.OpenCV_Error
           with "AP3P selection has no unique hypothesis";
      end if;
      return Result.Candidate_Index;
   end Selected_Hypothesis_Index;

   function Selected_Pose
     (Result : AP3P_Selection_Result;
      Hypotheses : AP3P_Pose_Hypothesis_Array;
      Index : Positive) return World_To_Camera_Pose
   is
      Position : constant Positive := Selected_Hypothesis_Index (Result);
   begin
      if Hypotheses'Length /= Result.Count
        or else Index not in Hypotheses'Range
        or else Index - Hypotheses'First + 1 /= Position
      then
         raise OpenCV.OpenCV_Error
           with "AP3P selected pose index does not name the selected hypothesis";
      end if;
      Validate (Hypotheses (Index).Pose);
      return Hypotheses (Index).Pose;
   end Selected_Pose;

   function Disposition (Result : AP3P_Pose_Selection) return AP3P_Selection_Disposition is
     (Result.Disposition);

   function Candidate_Index (Result : AP3P_Pose_Selection) return Natural is
     (Result.Candidate_Index);

   function Separation_Pixels (Result : AP3P_Pose_Selection) return OpenCV.Float64_Value is
     (Result.Separation_Pixels);

   function Candidates (Result : AP3P_Pose_Selection) return AP3P_Candidate_Qualification_Array is
     (Result.Candidates);

   function Selected (Result : AP3P_Pose_Selection) return Boolean is
     (Result.Disposition = Unique_Selected_Hypothesis
      and then Result.Has_Pose
      and then Result.Candidate_Index in 1 .. Result.Count);

   function Selected_Pose (Result : AP3P_Pose_Selection) return World_To_Camera_Pose is
   begin
      if not Selected (Result) then
         raise OpenCV.OpenCV_Error
           with "AP3P selection has no owned unique pose";
      end if;
      Validate (Result.Pose);
      return Result.Pose;
   end Selected_Pose;

   function Select_AP3P_Hypothesis
     (Results : AP3P_Reference_Result_Matrix;
      Options : AP3P_Selection_Options) return AP3P_Selection_Result
   is
      Summaries : constant AP3P_Hypothesis_Reference_Summary_Array :=
        Summarize_AP3P_References (Results);
      Eligible_Count : Natural := 0;
      Best_Index : Natural := 0;
      Second_Index : Natural := 0;
   begin
      Validate (Options);
      if Summaries'Length /= Results'Length (1) then
         raise OpenCV.OpenCV_Error with "AP3P selection summary length mismatch";
      end if;
      return Result : AP3P_Selection_Result (Results'Length (1)) do
         if Results'Length (1) = 0 then
            Result.Disposition := No_Hypotheses;
            return;
         elsif Results'Length (2) = 0 then
            Result.Disposition := No_Independent_References;
            return;
         end if;

         for Row in Summaries'Range loop
            declare
               Summary : constant AP3P_Hypothesis_Reference_Summary := Summaries (Row);
               Slot : constant Positive := Row - Summaries'First + 1;
               Within : Boolean;
            begin
               if Summary.Total_References /= Results'Length (2)
                 or else not Is_Finite (Summary.Reprojection.RMS_Error_Pixels)
                 or else not Is_Finite (Summary.Reprojection.Maximum_Error_Pixels)
                 or else Summary.Reprojection.RMS_Error_Pixels < 0.0
                 or else Summary.Reprojection.Maximum_Error_Pixels < 0.0
               then
                  raise OpenCV.OpenCV_Error with "Malformed AP3P reference summary";
               end if;
               Within :=
                 Summary.Reprojection.RMS_Error_Pixels <= Options.Maximum_RMS_Error_Pixels
                 and then Summary.Reprojection.Maximum_Error_Pixels
                   <= Options.Maximum_Point_Error_Pixels;
               if Summary.Entire_Set_Evaluable and then Within then
                  Result.Candidates (Slot) :=
                    (Eligible => True,
                     Rank => 0,
                     RMS_Error_Pixels => Summary.Reprojection.RMS_Error_Pixels,
                     Maximum_Error_Pixels => Summary.Reprojection.Maximum_Error_Pixels);
                  Eligible_Count := Eligible_Count + 1;
               end if;
            end;
         end loop;

         if Eligible_Count = 0 then
            Result.Disposition := No_Eligible_Hypotheses;
            return;
         end if;

         declare
            Remaining : array (1 .. Result.Count) of Boolean := [others => False];
         begin
            for Slot in Result.Candidates'Range loop
               Remaining (Slot) := Result.Candidates (Slot).Eligible;
            end loop;
            for Place in 1 .. Eligible_Count loop
               declare
                  Chosen : Natural := 0;
               begin
                  for Slot in Remaining'Range loop
                     if Remaining (Slot) and then
                       (Chosen = 0
                        or else Result.Candidates (Slot).RMS_Error_Pixels
                          < Result.Candidates (Chosen).RMS_Error_Pixels
                        or else (Result.Candidates (Slot).RMS_Error_Pixels
                                   = Result.Candidates (Chosen).RMS_Error_Pixels
                                 and then Slot < Chosen))
                     then
                        Chosen := Slot;
                     end if;
                  end loop;
                  if Chosen = 0 then
                     raise OpenCV.OpenCV_Error with "AP3P selection ranking failed";
                  end if;
                  Result.Candidates (Chosen).Rank := Place;
                  Remaining (Chosen) := False;
                  if Place = 1 then
                     Best_Index := Chosen;
                  elsif Place = 2 then
                     Second_Index := Chosen;
                  end if;
               end;
            end loop;
         end;

         if Eligible_Count = 1 then
            Result.Disposition := Unique_Selected_Hypothesis;
            Result.Candidate_Index := Best_Index;
            return;
         end if;

         declare
            Best_RMS : constant OpenCV.Float64_Value :=
              Result.Candidates (Best_Index).RMS_Error_Pixels;
            Second_RMS : constant OpenCV.Float64_Value :=
              Result.Candidates (Second_Index).RMS_Error_Pixels;
            Gap : OpenCV.Float64_Value;
         begin
            Gap := Second_RMS - Best_RMS;
            if not Is_Finite (Gap) then
               raise OpenCV.OpenCV_Error
                 with "AP3P RMS separation is not representable";
            end if;
            Result.Separation_Pixels := Gap;
            if Gap > Options.Minimum_RMS_Separation_Pixels then
               Result.Disposition := Unique_Selected_Hypothesis;
               Result.Candidate_Index := Best_Index;
            else
               Result.Disposition := Ambiguous_Qualified_Hypotheses;
               Result.Candidate_Index := 0;
            end if;
         end;
      end return;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "AP3P selection is unrepresentable";
   end Select_AP3P_Hypothesis;

   function Select_AP3P_Pose
     (Seed_World       : Object_Point_Array;
      Seed_Pixels      : Image_Point_Array;
      Reference_World  : Object_Point_Array;
      Reference_Pixels : Image_Point_Array;
      Intrinsics       : Camera_Intrinsics;
      Distortion       : Distortion_Coefficients := No_Distortion;
      Options          : AP3P_Selection_Options) return AP3P_Pose_Selection
   is
      function Without_Pose (Value : AP3P_Selection_Result) return AP3P_Pose_Selection is
        (Count => Value.Count,
         Disposition => Value.Disposition,
         Candidate_Index => Value.Candidate_Index,
         Separation_Pixels => Value.Separation_Pixels,
         Candidates => Value.Candidates,
         Has_Pose => False,
         Pose => (Rotation => [others => 0.0], Translation => [others => 0.0]));
   begin
      Validate (Options);
      Validate (Intrinsics);
      Validate (Distortion);
      Validate (Seed_World);
      Validate (Seed_Pixels);
      Validate (Reference_World);
      Validate (Reference_Pixels);
      if Seed_World'Length /= 3 or else Seed_Pixels'Length /= 3 then
         raise OpenCV.OpenCV_Error
           with "Select_AP3P_Pose requires exactly three seed correspondences";
      end if;
      if Reference_World'Length /= Reference_Pixels'Length then
         raise OpenCV.OpenCV_Error with "AP3P reference world/pixel counts differ";
      end if;
      if Reference_World'Length = 0 then
         return (Count => 0,
                 Disposition => No_Independent_References,
                 Candidate_Index => 0,
                 Separation_Pixels => 0.0,
                 Candidates => [1 .. 0 => <>],
                 Has_Pose => False,
                 Pose => (Rotation => [others => 0.0], Translation => [others => 0.0]));
      end if;
      for Left in Reference_World'Range loop
         for Seed of Seed_World loop
            if Same_World_Point (Reference_World (Left), Seed) then
               raise OpenCV.OpenCV_Error
                 with "AP3P reference repeats a seed world point";
            end if;
         end loop;
         for Right in Reference_World'First .. Left - 1 loop
            if Same_World_Point (Reference_World (Left), Reference_World (Right)) then
               raise OpenCV.OpenCV_Error
                 with "AP3P reference world points are not independent";
            end if;
         end loop;
      end loop;

      declare
         Hypotheses : constant AP3P_Pose_Hypothesis_Array :=
           Solve_AP3P_Hypotheses (Seed_World, Seed_Pixels, Intrinsics, Distortion);
         Evaluated : constant AP3P_Reference_Result_Matrix :=
           Evaluate_AP3P_References
             (Hypotheses, Reference_World, Reference_Pixels, Intrinsics, Distortion);
         Decision : constant AP3P_Selection_Result :=
           Select_AP3P_Hypothesis (Evaluated, Options);
      begin
         if Decision.Disposition /= Unique_Selected_Hypothesis then
            return Without_Pose (Decision);
         end if;
         return
           (Count => Decision.Count,
            Disposition => Decision.Disposition,
            Candidate_Index => Decision.Candidate_Index,
            Separation_Pixels => Decision.Separation_Pixels,
            Candidates => Decision.Candidates,
            Has_Pose => True,
            Pose => Hypotheses (Hypotheses'First + Decision.Candidate_Index - 1).Pose);
      end;
   end Select_AP3P_Pose;

   procedure Validate (Value : AP3P_Refinement_Acceptance_Options) is
   begin
      if not (Is_Finite (Value.Maximum_Holdout_RMS_Pixels)
              and then Is_Finite (Value.Maximum_Holdout_Point_Error_Pixels)
              and then Is_Finite (Value.Maximum_Holdout_RMS_Worsening_Pixels)
              and then Value.Maximum_Holdout_RMS_Pixels >= 0.0
              and then Value.Maximum_Holdout_Point_Error_Pixels >= 0.0
              and then Value.Maximum_Holdout_RMS_Worsening_Pixels >= 0.0)
      then
         raise OpenCV.OpenCV_Error
           with "AP3P refinement acceptance options must be finite and nonnegative";
      end if;
   end Validate;

   function Selection_Disposition
     (Result : AP3P_Refined_Pose) return AP3P_Selection_Disposition is
     (Result.Selection_State);

   function Refinement_Disposition
     (Result : AP3P_Refined_Pose) return AP3P_Refinement_Disposition is
     (Result.Refinement_State);

   function Refinement_Attempted (Result : AP3P_Refined_Pose) return Boolean is
     (Result.Attempted);

   function Native_Refinement_Succeeded (Result : AP3P_Refined_Pose) return Boolean is
     (Result.Native_Succeeded);

   function Accepted (Result : AP3P_Refined_Pose) return Boolean is
     (Result.Refinement_State = OpenCV.Calib3D.Accepted
      and then Result.Has_Accepted
      and then Result.Native_Succeeded
      and then Result.Refined_Valid);

   function Initial_Pose (Result : AP3P_Refined_Pose) return World_To_Camera_Pose is
   begin
      if not Result.Has_Initial then
         raise OpenCV.OpenCV_Error
           with "AP3P refinement has no owned initial pose";
      end if;
      Validate (Result.Initial);
      return Result.Initial;
   end Initial_Pose;

   function Accepted_Pose (Result : AP3P_Refined_Pose) return World_To_Camera_Pose is
   begin
      if not Accepted (Result) then
         raise OpenCV.OpenCV_Error
           with "AP3P refinement was not accepted";
      end if;
      Validate (Result.Accepted_Value);
      return Result.Accepted_Value;
   end Accepted_Pose;

   function Initial_Holdout_Valid (Result : AP3P_Refined_Pose) return Boolean is
     (Result.Initial_Valid);

   function Refined_Holdout_Valid (Result : AP3P_Refined_Pose) return Boolean is
     (Result.Refined_Valid);

   function Initial_Holdout (Result : AP3P_Refined_Pose) return Reprojection_Summary is
   begin
      if not Result.Initial_Valid then
         raise OpenCV.OpenCV_Error
           with "AP3P initial held-out summary is not valid";
      end if;
      return Result.Initial_Summary;
   end Initial_Holdout;

   function Refined_Holdout (Result : AP3P_Refined_Pose) return Reprojection_Summary is
   begin
      if not Result.Refined_Valid then
         raise OpenCV.OpenCV_Error
           with "AP3P refined held-out summary is not valid";
      end if;
      return Result.Refined_Summary;
   end Refined_Holdout;

   function Select_Refine_AP3P_Pose
     (Seed_World        : Object_Point_Array;
      Seed_Pixels       : Image_Point_Array;
      Selection_World   : Object_Point_Array;
      Selection_Pixels  : Image_Point_Array;
      Holdout_World     : Object_Point_Array;
      Holdout_Pixels    : Image_Point_Array;
      Intrinsics        : Camera_Intrinsics;
      Distortion        : Distortion_Coefficients := No_Distortion;
      Selection         : AP3P_Selection_Options;
      Acceptance        : AP3P_Refinement_Acceptance_Options)
      return AP3P_Refined_Pose
   is
      function From_Selection
        (Value : AP3P_Pose_Selection;
         State : AP3P_Refinement_Disposition;
         Attempted : Boolean := False;
         Native_Succeeded : Boolean := False;
         Initial_Valid : Boolean := False;
         Refined_Valid : Boolean := False;
         Accepted_Value : World_To_Camera_Pose :=
           (Rotation => [others => 0.0], Translation => [others => 0.0]);
         Initial_Summary : Reprojection_Summary := (others => <>);
         Refined_Summary : Reprojection_Summary := (others => <>);
         Has_Accepted : Boolean := False) return AP3P_Refined_Pose is
        (Count => Value.Count,
         Selection_State => Value.Disposition,
         Refinement_State => State,
         Attempted => Attempted,
         Native_Succeeded => Native_Succeeded,
         Has_Initial => Value.Has_Pose,
         Has_Accepted => Has_Accepted,
         Initial_Valid => Initial_Valid,
         Refined_Valid => Refined_Valid,
         Initial => Value.Pose,
         Accepted_Value => Accepted_Value,
         Initial_Summary => Initial_Summary,
         Refined_Summary => Refined_Summary,
         Candidates => Value.Candidates);

      procedure Require_Disjoint
        (Left_Name : String;
         Left : Object_Point_Array;
         Right_Name : String;
         Right : Object_Point_Array) is
      begin
         for Item of Left loop
            for Other of Right loop
               if Same_World_Point (Item, Other) then
                  raise OpenCV.OpenCV_Error
                    with Left_Name & " repeats a " & Right_Name & " world point";
               end if;
            end loop;
         end loop;
      end Require_Disjoint;

      procedure Require_Unique (Name : String; Points : Object_Point_Array) is
      begin
         for Left in Points'Range loop
            for Right in Points'First .. Left - 1 loop
               if Same_World_Point (Points (Left), Points (Right)) then
                  raise OpenCV.OpenCV_Error
                    with Name & " world points are not independent";
               end if;
            end loop;
         end loop;
      end Require_Unique;

      function Holdout_Summary
        (Pose : World_To_Camera_Pose;
         Usable : out Boolean) return Reprojection_Summary
      is
         Rows : constant AP3P_Pose_Hypothesis_Array := [(Pose => Pose)];
         Evaluated : constant AP3P_Reference_Result_Matrix :=
           Evaluate_AP3P_References
             (Rows, Holdout_World, Holdout_Pixels, Intrinsics, Distortion);
         Summaries : constant AP3P_Hypothesis_Reference_Summary_Array :=
           Summarize_AP3P_References (Evaluated);
      begin
         Usable := Summaries (Summaries'First).Entire_Set_Evaluable;
         if Usable then
            return Summaries (Summaries'First).Reprojection;
         end if;
         return (others => <>);
      end Holdout_Summary;

      function Within_Allowance
        (Original_RMS, Refined_RMS, Allowance : OpenCV.Float64_Value) return Boolean is
        (OpenCV.Calib3D.Internal.Refinement_Policy.Holdout_RMS_Worsening_Within_Allowance
           (Original_RMS, Refined_RMS, Allowance));
   begin
      Validate (Selection);
      Validate (Acceptance);
      Validate (Intrinsics);
      Validate (Distortion);
      Validate (Seed_World);
      Validate (Seed_Pixels);
      Validate (Selection_World);
      Validate (Selection_Pixels);
      Validate (Holdout_World);
      Validate (Holdout_Pixels);
      if Seed_World'Length /= Seed_Pixels'Length
        or else Selection_World'Length /= Selection_Pixels'Length
        or else Holdout_World'Length /= Holdout_Pixels'Length
      then
         raise OpenCV.OpenCV_Error
           with "AP3P refinement world/pixel counts differ";
      end if;
      if Seed_World'Length /= 3 then
         raise OpenCV.OpenCV_Error
           with "Select_Refine_AP3P_Pose requires exactly three seed correspondences";
      end if;
      if Holdout_World'Length = 0 then
         raise OpenCV.OpenCV_Error
           with "AP3P refinement requires at least one held-out reference";
      end if;
      if Selection_World'Length > Natural (Interfaces.Integer_32'Last) - 3
        or else Holdout_World'Length > Natural (Interfaces.Integer_32'Last)
      then
         raise OpenCV.OpenCV_Error
           with "AP3P refinement correspondence count exceeds native range";
      end if;
      Require_Unique ("AP3P seed", Seed_World);
      Require_Disjoint ("AP3P selection reference", Selection_World, "seed", Seed_World);
      Require_Unique ("AP3P selection reference", Selection_World);
      Require_Disjoint ("AP3P held-out reference", Holdout_World, "seed", Seed_World);
      Require_Disjoint
        ("AP3P held-out reference", Holdout_World, "selection", Selection_World);
      Require_Unique ("AP3P held-out reference", Holdout_World);

      declare
         Chosen : constant AP3P_Pose_Selection :=
           Select_AP3P_Pose
             (Seed_World, Seed_Pixels, Selection_World, Selection_Pixels,
              Intrinsics, Distortion, Selection);
      begin
         if not Selected (Chosen) then
            return From_Selection (Chosen, Selection_Only);
         end if;

         declare
            Initial : constant World_To_Camera_Pose := Selected_Pose (Chosen);
            Fitting_World : Object_Point_Array (1 .. 3 + Selection_World'Length);
            Fitting_Pixels : Image_Point_Array (1 .. Fitting_World'Length);
            Initial_Usable : Boolean := False;
            Initial_Score : constant Reprojection_Summary :=
              Holdout_Summary (Initial, Initial_Usable);
         begin
            if not Initial_Usable then
               return From_Selection
                 (Chosen, Initial_Holdout_Unusable, Initial_Valid => False);
            end if;

            for Offset in 0 .. 2 loop
               Fitting_World (1 + Offset) :=
                 Seed_World (Seed_World'First + Offset);
               Fitting_Pixels (1 + Offset) :=
                 Seed_Pixels (Seed_Pixels'First + Offset);
            end loop;
            for Offset in 0 .. Selection_World'Length - 1 loop
               Fitting_World (4 + Offset) :=
                 Selection_World (Selection_World'First + Offset);
               Fitting_Pixels (4 + Offset) :=
                 Selection_Pixels (Selection_Pixels'First + Offset);
            end loop;

            declare
               Candidate : World_To_Camera_Pose := Initial;
               Native_OK : Boolean := False;
            begin
               Refine_Pose_Iterative
                 (Fitting_World, Fitting_Pixels, Intrinsics, Distortion,
                  Pose => Candidate, Refined => Native_OK);
               if not Native_OK then
                  if Candidate /= Initial then
                     raise OpenCV.OpenCV_Error
                       with "False native refinement changed the initial pose";
                  end if;
                  return From_Selection
                    (Chosen, Native_Refinement_False,
                     Attempted => True,
                     Initial_Valid => True,
                     Initial_Summary => Initial_Score);
               end if;

               declare
                  Refined_Usable : Boolean := False;
                  Refined_Score : constant Reprojection_Summary :=
                    Holdout_Summary (Candidate, Refined_Usable);
               begin
                  if not Refined_Usable then
                     return From_Selection
                       (Chosen, Refined_Holdout_Unusable,
                        Attempted => True,
                        Native_Succeeded => True,
                        Initial_Valid => True,
                        Initial_Summary => Initial_Score);
                  end if;
                  if Refined_Score.RMS_Error_Pixels
                       <= Acceptance.Maximum_Holdout_RMS_Pixels
                    and then Refined_Score.Maximum_Error_Pixels
                       <= Acceptance.Maximum_Holdout_Point_Error_Pixels
                    and then Within_Allowance
                      (Initial_Score.RMS_Error_Pixels,
                       Refined_Score.RMS_Error_Pixels,
                       Acceptance.Maximum_Holdout_RMS_Worsening_Pixels)
                  then
                     return From_Selection
                       (Chosen, OpenCV.Calib3D.Accepted,
                        Attempted => True,
                        Native_Succeeded => True,
                        Initial_Valid => True,
                        Refined_Valid => True,
                        Accepted_Value => Candidate,
                        Initial_Summary => Initial_Score,
                        Refined_Summary => Refined_Score,
                        Has_Accepted => True);
                  end if;
                  return From_Selection
                    (Chosen, Rejected_By_Policy,
                     Attempted => True,
                     Native_Succeeded => True,
                     Initial_Valid => True,
                     Refined_Valid => True,
                     Initial_Summary => Initial_Score,
                     Refined_Summary => Refined_Score);
               end;
            end;
         end;
      end;
   end Select_Refine_AP3P_Pose;

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
      exception
         when others =>
            --  Access parameters are copy-in/copy-out. On an exception the
            --  caller never receives this newly allocated pointer.
            Free (Data);
            raise;
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
      exception
         when others =>
            --  Release owned buffers before propagating construction failure.
            --  Finalize is idempotent if return-object finalization calls again.
            Finalize (Estimate);
            raise;
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

   function Scaled_Component (Value, Baseline : OpenCV.Float64_Value)
     return OpenCV.Float64_Value
   is
      Product : constant OpenCV.Float64_Value := Value * Baseline;
   begin
      if not Is_Finite (Product) or else
        (Value /= 0.0 and then Product = 0.0)
      then
         raise Constraint_Error with "Metric scaling overflow/underflow";
      end if;
      return Product;
   end Scaled_Component;

   function Metric_Pixel_Error (Prediction, Observation : Image_Point)
     return OpenCV.Float64_Value
   is
      DX : constant OpenCV.Float64_Value := Prediction (0) - Observation (0);
      DY : constant OpenCV.Float64_Value := Prediction (1) - Observation (1);
   begin
      --  Existing scaled norm checks both subtraction and final norm.
      return Scaled_Vector_Norm ([DX, DY, 0.0]);
   end Metric_Pixel_Error;

   function Evaluate_Metric_Point
     (Point : Triangulated_Point; Quality : Triangulation_Quality;
      Baseline : OpenCV.Float64_Value;
      First_Pose, Second_Pose : World_To_Camera_Pose;
      First_Rotation, Second_Rotation : Rotation_Matrix;
      First_Intrinsics, Second_Intrinsics : Camera_Intrinsics;
      First_Distortion, Second_Distortion : Distortion_Coefficients;
      First_Pixel, Second_Pixel : Image_Point;
      Options : Metric_Triangulation_Options;
      Recompute_Normalized : Boolean := False;
      First_Observation, Second_Observation : Normalized_Image_Point := [0.0, 0.0])
      return Metric_Triangulated_Point
   is
      Result : Metric_Triangulated_Point;
   begin
      Result.Source := (True, Point, Quality);
      Result.Disposition := Metric_Conversion_Unavailable;
      declare
         Local, World, Check_First, Check_Second : Object_Point;
         Z1, Z2 : OpenCV.Float64_Value;
         Converted : Boolean := False;
      begin
         --  Arithmetic only: native failures must never be
         --  swallowed by this per-point conversion handler.
         begin
            for Axis in 0 .. 2 loop
               Local (Axis) := Scaled_Component
                 (Point.Position_In_First_Camera (Axis), Baseline);
            end loop;
            Z1 := Scaled_Component (Point.First_Depth, Baseline);
            Z2 := Scaled_Component (Point.Second_Depth, Baseline);
            World := Camera_To_World_Point (First_Pose, Local, First_Rotation);
            Check_First := Rotate (First_Rotation, World);
            Check_Second := Rotate (Second_Rotation, World);
            for Axis in 0 .. 2 loop
               Check_First (Axis) := Checked_Sum
                 (Check_First (Axis), First_Pose.Translation (Axis));
               Check_Second (Axis) := Checked_Sum
                 (Check_Second (Axis), Second_Pose.Translation (Axis));
            end loop;
            Converted := Z1 > 0.0 and then Z2 > 0.0 and then
              Check_First (2) > 0.0 and then Check_Second (2) > 0.0;
         exception
            when Constraint_Error | OpenCV.OpenCV_Error => null;
         end;
         if Converted then
            Result.Geometry := (True, Local, World, Z1, Z2);
            if Recompute_Normalized then
               --  Diagnose the actual representable world point before native
               --  pixel projection. Large origins may round away local offsets.
               begin
                  declare
                     P : Triangulated_Point := Point;
                     Q : Triangulation_Quality := Quality;
                  begin
                     P.First_Normalized_Error := Metric_Pixel_Error
                       ([Check_First (0) / Check_First (2), Check_First (1) / Check_First (2)],
                        Image_Point (First_Observation));
                     P.Second_Normalized_Error := Metric_Pixel_Error
                       ([Check_Second (0) / Check_Second (2), Check_Second (1) / Check_Second (2)],
                        Image_Point (Second_Observation));
                     --  Parallax is the already freshly assessed acute angle
                     --  between observed bearing lines, independent of range.
                     Q.Passes_Reprojection_Limit :=
                       P.First_Normalized_Error <= Options.Maximum_Normalized_Reprojection_Error and then
                       P.Second_Normalized_Error <= Options.Maximum_Normalized_Reprojection_Error;
                     Q.Accepted := Q.Passes_Parallax_Limit and then Q.Passes_Reprojection_Limit;
                     Result.Source := (True, P, Q);
                  end;
               exception
                  when Constraint_Error | OpenCV.OpenCV_Error =>
                     Result.Source := (Available => False);
                     return Result;
               end;
            end if;
            declare
               --  Native projection is deliberately outside
               --  the arithmetic handler. Any failure aborts
               --  the whole batch, including earlier points.
               P1 : constant Image_Point_Array := Project_Points
                 ([1 => World], First_Intrinsics, First_Distortion, First_Pose);
               P2 : constant Image_Point_Array := Project_Points
                 ([1 => World], Second_Intrinsics, Second_Distortion, Second_Pose);
               E1, E2 : OpenCV.Float64_Value;
            begin
               begin
                  E1 := Metric_Pixel_Error (P1 (1), First_Pixel);
                  E2 := Metric_Pixel_Error (P2 (1), Second_Pixel);
                  Result.Pixels :=
                    (True, E1, E2,
                     E1 <= Options.Maximum_First_Pixel_Reprojection_Error,
                     E2 <= Options.Maximum_Second_Pixel_Reprojection_Error);
                  Result.Disposition :=
                    (if Result.Source.Quality.Accepted and then
                       Result.Pixels.Passes_First_Limit and then
                       Result.Pixels.Passes_Second_Limit
                     then Metric_Accepted else Metric_Policy_Rejected);
               exception
                  when Constraint_Error | OpenCV.OpenCV_Error => null;
               end;
            end;
         end if;
      end;
      return Result;
   end Evaluate_Metric_Point;

   procedure Validate (Options : Metric_Triangulation_Options) is
   begin
      if not Is_Finite (Options.Minimum_Acute_Parallax_Radians) or else
        Options.Minimum_Acute_Parallax_Radians < 0.0 or else
        Options.Minimum_Acute_Parallax_Radians > Math.Arctan (1.0, 0.0)
      then
         raise OpenCV.OpenCV_Error with "Invalid metric parallax threshold";
      end if;
      for Limit of Object_Point'[Options.Maximum_Normalized_Reprojection_Error,
                    Options.Maximum_First_Pixel_Reprojection_Error,
                    Options.Maximum_Second_Pixel_Reprojection_Error]
      loop
         if not Is_Finite (Limit) or else Limit < 0.0 then
            raise OpenCV.OpenCV_Error with "Invalid metric residual threshold";
         end if;
      end loop;
   end Validate;

   function Triangulate_World_Points
     (First_Pixels, Second_Pixels : Image_Point_Array;
      First_Intrinsics, Second_Intrinsics : Camera_Intrinsics;
      First_Distortion, Second_Distortion : Distortion_Coefficients;
      First_Pose, Second_Pose : World_To_Camera_Pose;
      Options : Metric_Triangulation_Options)
      return Metric_Triangulated_Point_Array
   is
   begin
      Validate (First_Pixels); Validate (Second_Pixels);
      Validate (First_Intrinsics); Validate (Second_Intrinsics);
      Validate (First_Distortion); Validate (Second_Distortion);
      Validate (First_Pose); Validate (Second_Pose);
      if First_Pixels'Length /= Second_Pixels'Length or else
        First_Pixels'Length > Interfaces.Integer_32'Last
      then
         raise OpenCV.OpenCV_Error with "Metric triangulation requires equal counts fitting INT32";
      end if;
      Validate (Options);
      if First_Pixels'Length = 0 then
         return [1 .. 0 => <>];
      end if;
      declare
         Motion : constant Camera_Motion_Result := Measure_Camera_Motion (First_Pose, Second_Pose);
      begin
         if Motion.Baseline_Distance = 0.0 then
            return [1 .. First_Pixels'Length => <>];
         end if;
         declare
            Relative : constant Relative_Camera_Pose :=
              (Rotation_Matrix_Of (Motion.Relative_Pose),
               Camera_Direction (Unit_Vector (Motion.Relative_Pose.Translation)));
            First_Rotation : constant Rotation_Matrix := Rotation_Matrix_Of (First_Pose);
            Second_Rotation : constant Rotation_Matrix := Rotation_Matrix_Of (Second_Pose);
            First : constant Normalized_Image_Point_Array := Undistort_To_Normalized
              (First_Pixels, First_Intrinsics, First_Distortion);
            Second : constant Normalized_Image_Point_Array := Undistort_To_Normalized
              (Second_Pixels, Second_Intrinsics, Second_Distortion);
            Source : constant Triangulated_Point_Array :=
              Triangulate_Normalized (First, Second, Relative);
            Quality : constant Triangulation_Quality_Array := Assess_Triangulation
              (First, Second, Relative, Source,
               (Options.Minimum_Acute_Parallax_Radians,
                Options.Maximum_Normalized_Reprojection_Error));
         begin
            return Result : Metric_Triangulated_Point_Array (Source'Range) do
               for I in Result'Range loop
                  Result (I).Source := (True, Source (I), Quality (I));
                  case Source (I).Status is
                     when At_Infinity => Result (I).Disposition := Source_At_Infinity;
                     when Unrepresentable_Point => Result (I).Disposition := Source_Unrepresentable_Point;
                     when Non_Positive_Depth => Result (I).Disposition := Source_Non_Positive_Depth;
                     when Undefined_Reprojection => Result (I).Disposition := Source_Undefined_Reprojection;
                     when Usable =>
                        Result (I) := Evaluate_Metric_Point
                          (Source (I), Quality (I), Motion.Baseline_Distance,
                           First_Pose, Second_Pose, First_Rotation, Second_Rotation,
                           First_Intrinsics, Second_Intrinsics, First_Distortion, Second_Distortion,
                           First_Pixels (First_Pixels'First + (I - 1)),
                           Second_Pixels (Second_Pixels'First + (I - 1)), Options);
                  end case;
               end loop;
            end return;
         end;
      end;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Metric triangulation exceeds representable range";
   end Triangulate_World_Points;

   function Disposition (Result : Metric_Point_Refinement_Result)
     return Metric_Refinement_Disposition is (Result.State);

   function Original_Reconstruction (Result : Metric_Point_Refinement_Result)
     return Metric_Triangulated_Point is (Result.Original);

   function Normalized_Refinement (Result : Metric_Point_Refinement_Result)
     return Optional_Point_Refinement is (Result.Optimization);

   function Refined_Diagnostics (Result : Metric_Point_Refinement_Result)
     return Metric_Triangulated_Point is (Result.Candidate);

   function Accepted (Result : Metric_Point_Refinement_Result) return Boolean is
     (Result.State = Refined_Metric_Accepted);

   function Accepted_World_Position (Result : Metric_Point_Refinement_Result)
     return Object_Point is
   begin
      if not Accepted (Result) then
         raise OpenCV.OpenCV_Error with "Metric refinement was not accepted";
      end if;
      return Result.Candidate.Geometry.World_Position;
   end Accepted_World_Position;

   procedure Validate
     (Refinement_Options : Triangulation_Refinement_Options;
      Acceptance : Metric_Refinement_Acceptance_Options)
   is
   begin
      for Limit of OpenCV.Core.Float64_Vec2.Vector'
        [Acceptance.Maximum_First_Pixel_Worsening, Acceptance.Maximum_Second_Pixel_Worsening]
      loop
         if not Is_Finite (Limit) or else Limit < 0.0 then
            raise OpenCV.OpenCV_Error with "Invalid metric refinement allowance";
         end if;
      end loop;
      --  Validate even for empty batches and unavailable physical baselines.
      if Refinement_Options.Maximum_Iterations > 100 or else
        not Is_Finite (Refinement_Options.Minimum_Acute_Parallax_Radians) or else
        Refinement_Options.Minimum_Acute_Parallax_Radians < 0.0 or else
        Refinement_Options.Minimum_Acute_Parallax_Radians > Math.Arctan (1.0, 0.0)
      then
         raise OpenCV.OpenCV_Error with "Invalid metric refinement options";
      end if;
   end Validate;

   function Refine_Metric_World_Points
     (First_Pixels, Second_Pixels : Image_Point_Array;
      First_Intrinsics, Second_Intrinsics : Camera_Intrinsics;
      First_Distortion, Second_Distortion : Distortion_Coefficients;
      First_Pose, Second_Pose : World_To_Camera_Pose;
      Metric_Options : Metric_Triangulation_Options;
      Refinement_Options : Triangulation_Refinement_Options;
      Acceptance : Metric_Refinement_Acceptance_Options)
      return Metric_Point_Refinement_Result_Array
   is
      function Worsening_OK (Initial, Refined, Allowance : OpenCV.Float64_Value)
        return Boolean is
      begin
         if Refined <= Initial then
            return True;
         end if;
         --  Finite nonnegative errors: direct subtraction, no rescaling.
         return Refined - Initial <= Allowance;
      end Worsening_OK;
   begin
      Validate (Refinement_Options, Acceptance);
      declare
         Initial : constant Metric_Triangulated_Point_Array := Triangulate_World_Points
           (First_Pixels, Second_Pixels, First_Intrinsics, Second_Intrinsics,
            First_Distortion, Second_Distortion, First_Pose, Second_Pose, Metric_Options);
         Seeds : Triangulated_Point_Array (Initial'Range) := [others => (Status => At_Infinity)];
         Eligible : Boolean := False;
      begin
         return Result : Metric_Point_Refinement_Result_Array (Initial'Range) do
            for I in Initial'Range loop
               Result (I).Original := Initial (I);
               if not Initial (I).Source.Available then
                  Result (I).State := Initial_No_Baseline;
               elsif Initial (I).Source.Point.Status /= Usable then
                  Result (I).State := Initial_Source_Unusable;
               elsif not Initial (I).Geometry.Available then
                  Result (I).State := Initial_Geometry_Unavailable;
               elsif not Initial (I).Pixels.Available then
                  Result (I).State := Initial_Pixels_Unavailable;
               else
                  Seeds (I) := Initial (I).Source.Point;
                  Eligible := True;
               end if;
            end loop;
            if Eligible then
               declare
                  Motion : constant Camera_Motion_Result := Measure_Camera_Motion (First_Pose, Second_Pose);
                  Relative : constant Relative_Camera_Pose :=
                    (Rotation_Matrix_Of (Motion.Relative_Pose),
                     Camera_Direction (Unit_Vector (Motion.Relative_Pose.Translation)));
                  R1 : constant Rotation_Matrix := Rotation_Matrix_Of (First_Pose);
                  R2 : constant Rotation_Matrix := Rotation_Matrix_Of (Second_Pose);
                  First : constant Normalized_Image_Point_Array := Undistort_To_Normalized
                    (First_Pixels, First_Intrinsics, First_Distortion);
                  Second : constant Normalized_Image_Point_Array := Undistort_To_Normalized
                    (Second_Pixels, Second_Intrinsics, Second_Distortion);
                  Refined : constant Point_Refinement_Result_Array := Refine_Triangulated_Points
                    (First, Second, Relative, Seeds, Refinement_Options);
                  Quality : constant Triangulation_Quality_Array := Assess_Triangulation
                    (First, Second, Relative, [for P of Refined => P.Point],
                     (Metric_Options.Minimum_Acute_Parallax_Radians,
                      Metric_Options.Maximum_Normalized_Reprojection_Error));
               begin
                  for I in Initial'Range loop
                     if Seeds (I).Status = Usable then
                        Result (I).Optimization := (True, Refined (I));
                        case Refined (I).Outcome is
                           when Skipped_Unusable => Result (I).State := Initial_Source_Unusable;
                           when Skipped_Low_Parallax => Result (I).State := Refinement_Low_Parallax;
                           when No_Improving_Step => Result (I).State := Refinement_No_Improving_Step;
                           when Numerically_Unavailable => Result (I).State := Refinement_Numerically_Unavailable;
                           when Improved =>
                              Result (I).Candidate := Evaluate_Metric_Point
                                (Refined (I).Point, Quality (I), Motion.Baseline_Distance,
                                 First_Pose, Second_Pose, R1, R2,
                                 First_Intrinsics, Second_Intrinsics, First_Distortion, Second_Distortion,
                                 First_Pixels (First_Pixels'First + (I - 1)),
                                 Second_Pixels (Second_Pixels'First + (I - 1)), Metric_Options,
                                 True, First (I), Second (I));
                              declare
                                 C : Metric_Triangulated_Point renames Result (I).Candidate;
                              begin
                                 if not C.Geometry.Available then
                                    Result (I).State := Refined_Conversion_Unavailable;
                                 elsif not C.Source.Available then
                                    Result (I).State := Refined_Conversion_Unavailable;
                                 elsif not C.Pixels.Available then
                                    Result (I).State := Refined_Pixels_Unavailable;
                                 elsif C.Source.Quality.Accepted and then
                                   C.Pixels.Passes_First_Limit and then C.Pixels.Passes_Second_Limit and then
                                   Worsening_OK (Initial (I).Pixels.First_Error_Pixels,
                                     C.Pixels.First_Error_Pixels, Acceptance.Maximum_First_Pixel_Worsening) and then
                                   Worsening_OK (Initial (I).Pixels.Second_Error_Pixels,
                                     C.Pixels.Second_Error_Pixels, Acceptance.Maximum_Second_Pixel_Worsening)
                                 then
                                    Result (I).State := Refined_Metric_Accepted;
                                 else
                                    Result (I).State := Refined_Policy_Rejected;
                                 end if;
                                 if Result (I).State = Refined_Metric_Accepted then
                                    C.Disposition := Metric_Accepted;
                                 elsif Result (I).State = Refined_Policy_Rejected then
                                    C.Disposition := Metric_Policy_Rejected;
                                 end if;
                              end;
                        end case;
                     end if;
                  end loop;
               end;
            end if;
         end return;
      end;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Metric refinement exceeds representable range";
   end Refine_Metric_World_Points;

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

   ---------------------------------------------------------------------------
   --  Guarded native epipolar correspondence correction (Task 015).
   --  Ada validates, measures and re-verifies; native cv::correctMatches is the
   --  only correction algorithm. Nothing here solves for corrected points.
   ---------------------------------------------------------------------------

   type Scalar_Array is array (Positive range <>) of OpenCV.Float64_Value;

   --  Scaled Euclidean norm: never squares an unscaled component.
   function Scaled_Norm (Values : Scalar_Array) return OpenCV.Float64_Value is
      Scale : OpenCV.Float64_Value := 0.0;
      Sum   : OpenCV.Float64_Value := 0.0;
   begin
      for V of Values loop
         if not Is_Finite (V) then
            raise Constraint_Error;
         end if;
         Scale := OpenCV.Float64_Value'Max (Scale, abs V);
      end loop;
      if Scale = 0.0 then
         return 0.0;
      end if;
      for V of Values loop
         Sum := Sum + (V / Scale)**2;
      end loop;
      return Scale * Math.Sqrt (Sum);
   end Scaled_Norm;

   --  d1 = dist(p1, F^T p2), d2 = dist(p2, F p1); identical arithmetic to
   --  Maximum_Epipolar_Error, but both distances are returned separately.
   procedure Epipolar_Distances
     (Matrix : Fundamental_Matrix; First_Point, Second_Point : Image_Point;
      Defined : out Boolean; D1, D2 : out OpenCV.Float64_Value)
   is
      subtype Scalar is OpenCV.Float64_Value;
      type Line is array (Natural range 0 .. 2) of Scalar;
      L1, L2 : Line;
      N1, N2 : Scalar;
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
   begin
      Defined := False;
      D1 := 0.0;
      D2 := 0.0;
      for I in 0 .. 2 loop
         L2 (I) := Sum3 (Matrix (I, 0), First_Point (0),
                         Matrix (I, 1), First_Point (1), Matrix (I, 2));
         L1 (I) := Sum3 (Matrix (0, I), Second_Point (0),
                         Matrix (1, I), Second_Point (1), Matrix (2, I));
      end loop;
      N1 := Scaled_Norm ([L1 (0), L1 (1)]);
      N2 := Scaled_Norm ([L2 (0), L2 (1)]);
      if not Is_Finite (N1) or else not Is_Finite (N2) or else N1 = 0.0 or else N2 = 0.0 then
         return;
      end if;
      D1 := abs Sum3 (L1 (0), First_Point (0), L1 (1), First_Point (1), L1 (2)) / N1;
      D2 := abs Sum3 (L2 (0), Second_Point (0), L2 (1), Second_Point (1), L2 (2)) / N2;
      Defined := Is_Finite (D1) and then Is_Finite (D2);
   exception
      when Constraint_Error =>
         Defined := False;
         D1 := 0.0;
         D2 := 0.0;
   end Epipolar_Distances;

   procedure Validate (Value : Epipolar_Correction_Options) is
   begin
      if not (Is_Finite (Value.Minimum_Second_Singular_Ratio)
              and then Is_Finite (Value.Maximum_Third_Singular_Ratio)
              and then Is_Finite (Value.Minimum_Epipolar_Line_Ratio)
              and then Is_Finite (Value.Maximum_Corrected_Error_Pixels)
              and then Is_Finite (Value.Absolute_Displacement_Excess_Pixels)
              and then Is_Finite (Value.Relative_Displacement_Excess))
        or else not (Value.Minimum_Second_Singular_Ratio > 0.0
                     and then Value.Minimum_Second_Singular_Ratio <= 1.0)
        or else not (Value.Maximum_Third_Singular_Ratio >= 0.0
                     and then Value.Maximum_Third_Singular_Ratio < 1.0)
        or else not (Value.Maximum_Third_Singular_Ratio < Value.Minimum_Second_Singular_Ratio)
        or else not (Value.Minimum_Epipolar_Line_Ratio >= 0.0
                     and then Value.Minimum_Epipolar_Line_Ratio <= 1.0)
        or else not (Value.Maximum_Corrected_Error_Pixels >= 0.0)
        or else not (Value.Absolute_Displacement_Excess_Pixels >= 0.0)
        or else not (Value.Relative_Displacement_Excess >= 0.0
                     and then Value.Relative_Displacement_Excess <= 1.0)
      then
         raise OpenCV.OpenCV_Error with "Invalid epipolar correction options";
      end if;
   end Validate;

   function Correct_Epipolar_Matches
     (Matrix        : Fundamental_Matrix;
      First_Points  : Image_Point_Array;
      Second_Points : Image_Point_Array;
      Options       : Epipolar_Correction_Options)
      return Corrected_Epipolar_Matches
   is
      Count : constant Natural := First_Points'Length;
      Normalized : Fundamental_Matrix;
      Scale : OpenCV.Float64_Value := 0.0;
      Native_Matrix : aliased C.C_Fundamental;
      Native_Options : aliased C.C_Correction_Options;
      First, Second, Output : OpenCV.Core.Mat;
      Code : C.Status := C.Success;
      Bounds : Scalar_Array (1 .. Natural'Max (Count, 1)) := [others => 0.0];
      Originals : Scalar_Array (1 .. Natural'Max (Count, 1)) := [others => 0.0];
      procedure First_Callback (First_Handle : Bridge.Input_Mat_Handle) is
         procedure Second_Callback (Second_Handle : Bridge.Input_Mat_Handle) is
            procedure Output_Callback (Destination : Bridge.Output_Mat_Handle) is
            begin
               Code := C.Correct_Matches (Native_Matrix'Access, First_Handle, Second_Handle,
                                          Native_Options'Access, Destination);
            end Output_Callback;
         begin
            Bridge.With_Output_Handle (Output, Output_Callback'Access);
         end Second_Callback;
      begin
         Bridge.With_Input_Handle (Second, Second_Callback'Access);
      end First_Callback;
      function Typed (Points : Image_Point_Array) return OpenCV.Core.Mat is
        (if Points'Length = 0
         then OpenCV.Core.Create (0, 1, (Depth => OpenCV.Core.Float64, Channels => 2))
         else Image_Matrix (Points));
   begin
      Validate (Options);
      for Value of Matrix loop
         if not Is_Finite (Value) then
            raise OpenCV.OpenCV_Error with "Fundamental entries must be finite";
         end if;
         Scale := OpenCV.Float64_Value'Max (Scale, abs Value);
      end loop;
      if Scale = 0.0 then
         raise OpenCV.OpenCV_Error with "Fundamental matrix is zero";
      end if;
      --  Homogeneous max-coefficient normalization, also used for diagnostics so
      --  extreme-scale F does not cause avoidable overflow. Reject a nonzero
      --  coefficient that would become zero/subnormal (a different matrix).
      for Row in 0 .. 2 loop
         for Col in 0 .. 2 loop
            Normalized (Row, Col) := Matrix (Row, Col) / Scale;
            if Matrix (Row, Col) /= 0.0 and then
              abs Normalized (Row, Col) < OpenCV.Float64_Value'Model_Small
            then
               raise OpenCV.OpenCV_Error with
                 "Fundamental normalization would underflow a nonzero coefficient";
            end if;
         end loop;
      end loop;
      if Second_Points'Length /= Count or else
        First_Points'Length > Interfaces.Integer_32'Last
      then
         raise OpenCV.OpenCV_Error with "Epipolar correction requires equal counts fitting INT32";
      end if;
      Validate (First_Points);
      Validate (Second_Points);
      for I in 1 .. Count loop
         declare
            Defined : Boolean;
            D1, D2 : OpenCV.Float64_Value;
            Error : constant Epipolar_Error_Result := Maximum_Epipolar_Error
              (Normalized, First_Points (First_Points'First + (I - 1)),
               Second_Points (Second_Points'First + (I - 1)));
         begin
            Epipolar_Distances (Normalized, First_Points (First_Points'First + (I - 1)),
                                Second_Points (Second_Points'First + (I - 1)), Defined, D1, D2);
            if not Defined or else not Error.Defined then
               raise OpenCV.OpenCV_Error with "Input epipolar line is undefined";
            end if;
            Bounds (I) := OpenCV.Float64_Value'Min (D1, D2);
            Originals (I) := Error.Maximum_Error_Pixels;
         end;
      end loop;
      Native_Matrix :=
        (Interfaces.C.double (Matrix (0, 0)), Interfaces.C.double (Matrix (0, 1)),
         Interfaces.C.double (Matrix (0, 2)), Interfaces.C.double (Matrix (1, 0)),
         Interfaces.C.double (Matrix (1, 1)), Interfaces.C.double (Matrix (1, 2)),
         Interfaces.C.double (Matrix (2, 0)), Interfaces.C.double (Matrix (2, 1)),
         Interfaces.C.double (Matrix (2, 2)));
      Native_Options :=
        (Interfaces.C.double (Options.Minimum_Second_Singular_Ratio),
         Interfaces.C.double (Options.Maximum_Third_Singular_Ratio),
         Interfaces.C.double (Options.Minimum_Epipolar_Line_Ratio),
         Interfaces.C.double (Options.Maximum_Corrected_Error_Pixels),
         Interfaces.C.double (Options.Absolute_Displacement_Excess_Pixels),
         Interfaces.C.double (Options.Relative_Displacement_Excess));
      --  Empty requests still reach the shim (typed 0x1) so the native SVD policy
      --  is enforced identically; the shim never runs correctMatches for them.
      First := Typed (First_Points);
      Second := Typed (Second_Points);
      Bridge.With_Input_Handle (First, First_Callback'Access);
      C.Check (Code, "Calib3D.Correct_Epipolar_Matches");
      if Output.Rows /= Count or else Output.Columns /= 2
        or else Output.Depth /= OpenCV.Core.Float64 or else Output.Channels /= 2
      then
         raise OpenCV.OpenCV_Error with "Invalid native correctMatches result";
      end if;
      return Result : Corrected_Epipolar_Matches (Count) do
         for I in 1 .. Count loop
            declare
               P1 : constant Image_Point := First_Points (First_Points'First + (I - 1));
               P2 : constant Image_Point := Second_Points (Second_Points'First + (I - 1));
               Q1 : constant Image_Point := Vec2_Access.Get (Output, I - 1, 0);
               Q2 : constant Image_Point := Vec2_Access.Get (Output, I - 1, 1);
               Corrected : Epipolar_Error_Result;
               Displacement, Allowed : OpenCV.Float64_Value;
            begin
               for V of Scalar_Array'[Q1 (0), Q1 (1), Q2 (0), Q2 (1)] loop
                  if not Is_Finite (V) then
                     raise OpenCV.OpenCV_Error with "Corrected coordinate is nonfinite";
                  end if;
               end loop;
               Corrected := Maximum_Epipolar_Error (Normalized, Q1, Q2);
               if not Corrected.Defined then
                  raise OpenCV.OpenCV_Error with "Corrected epipolar line is undefined";
               end if;
               Displacement := Scaled_Norm
                 ([Q1 (0) - P1 (0), Q1 (1) - P1 (1), Q2 (0) - P2 (0), Q2 (1) - P2 (1)]);
               Allowed := Bounds (I) * (1.0 + Options.Relative_Displacement_Excess)
                 + Options.Absolute_Displacement_Excess_Pixels;
               --  Independent Ada re-verification of the native acceptance policy.
               if not Is_Finite (Displacement) or else not Is_Finite (Allowed)
                 or else Corrected.Maximum_Error_Pixels > Options.Maximum_Corrected_Error_Pixels
                 or else Displacement > Allowed
               then
                  raise OpenCV.OpenCV_Error with
                    "Corrected correspondence failed Ada re-verification";
               end if;
               Result.First_Points (I) := Q1;
               Result.Second_Points (I) := Q2;
               Result.Diagnostics (I) :=
                 (Original_Error_Pixels     => Originals (I),
                  Corrected_Error_Pixels    => Corrected.Maximum_Error_Pixels,
                  Total_Displacement_Pixels => Displacement,
                  One_Sided_Bound_Pixels    => Bounds (I));
            end;
         end loop;
      end return;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error with "Epipolar correction arithmetic exceeds representable range";
   end Correct_Epipolar_Matches;
   function Disposition (Result : Camera_Relocalization_Result)
      return Camera_Relocalization_Disposition is (Result.State);
   function Found (Result : Camera_Relocalization_Result) return Boolean is (Result.Has_Initial);
   function Inlier_Count (Result : Camera_Relocalization_Result) return Natural is
     (if Result.Has_Initial then Result.Count else 0);
   function Inliers (Result : Camera_Relocalization_Result) return Inlier_Index_Array is
     (if Result.Has_Initial then Result.Positions else Inlier_Index_Array'[1 .. 0 => <>]);
   function Refinement_Attempted (Result : Camera_Relocalization_Result) return Boolean is (Result.Attempted);
   function Native_Refinement_Succeeded (Result : Camera_Relocalization_Result) return Boolean is
     (Result.Native_Succeeded);
   function Initial_Holdout_Valid (Result : Camera_Relocalization_Result) return Boolean is (Result.Initial_Valid);
   function Refined_Holdout_Valid (Result : Camera_Relocalization_Result) return Boolean is (Result.Refined_Valid);
   function Final_Inliers_Valid (Result : Camera_Relocalization_Result) return Boolean is (Result.Inliers_Valid);
   function Accepted (Result : Camera_Relocalization_Result) return Boolean is
     (Result.State = Accepted and then Result.Native_Succeeded and then Result.Inliers_Valid
      and then Result.Initial_Valid and then Result.Refined_Valid);

   function Initial_Pose (Result : Camera_Relocalization_Result) return World_To_Camera_Pose is
   begin
      if not Found (Result) then
         raise OpenCV.OpenCV_Error with "Relocalization has no native RANSAC pose";
      end if;
      return Result.Initial;
   end Initial_Pose;

   function Accepted_Pose (Result : Camera_Relocalization_Result) return World_To_Camera_Pose is
   begin
      if not Accepted (Result) then
         raise OpenCV.OpenCV_Error with "Relocalization has no accepted pose";
      end if;
      return Result.Final_Pose;
   end Accepted_Pose;

   function Camera_Center (Result : Camera_Relocalization_Result) return Object_Point is
   begin
      if not Accepted (Result) then
         raise OpenCV.OpenCV_Error with "Relocalization has no accepted camera center";
      end if;
      return Result.Center;
   end Camera_Center;

   function Initial_Holdout (Result : Camera_Relocalization_Result) return Reprojection_Summary is
   begin
      if not Result.Initial_Valid then
         raise OpenCV.OpenCV_Error with "Relocalization initial holdout unavailable";
      end if;
      return Result.Initial_Summary;
   end Initial_Holdout;
   function Refined_Holdout (Result : Camera_Relocalization_Result) return Reprojection_Summary is
   begin
      if not Result.Refined_Valid then
         raise OpenCV.OpenCV_Error with "Relocalization refined holdout unavailable";
      end if;
      return Result.Refined_Summary;
   end Refined_Holdout;
   function Final_Inlier_Summary (Result : Camera_Relocalization_Result) return Reprojection_Summary is
   begin
      if not Result.Inliers_Valid then
         raise OpenCV.OpenCV_Error with "Relocalization final inlier diagnostics unavailable";
      end if;
      return Result.Inlier_Summary;
   end Final_Inlier_Summary;
   function Final_Inlier_Errors (Result : Camera_Relocalization_Result) return Reprojection_Error_Array is
   begin
      if not Result.Inliers_Valid then
         raise OpenCV.OpenCV_Error with "Relocalization final inlier diagnostics unavailable";
      end if;
      return Result.Errors;
   end Final_Inlier_Errors;

   function Original_Position (Result : Three_View_Landmark_Result) return Positive is (Result.Position);
   function Disposition (Result : Three_View_Landmark_Result)
      return Three_View_Landmark_Disposition is (Result.State);
   function Epipolar_Disposition (Result : Three_View_Landmark_Result)
      return Three_View_Epipolar_Disposition is (Result.Screening);
   function Epipolar_Error (Result : Three_View_Landmark_Result)
      return Epipolar_Correspondence_Result is (Result.Sampson);
   function Has_Initial_Reconstruction (Result : Three_View_Landmark_Result) return Boolean is (Result.Has_Initial);
   function Original_Reconstruction (Result : Three_View_Landmark_Result)
      return Metric_Triangulated_Point is
   begin
      if not Result.Has_Initial then
         raise OpenCV.OpenCV_Error with "Three-view initial reconstruction unavailable";
      end if;
      return Result.Initial;
   end Original_Reconstruction;
   function Refinement_Requested (Result : Three_View_Landmark_Result) return Boolean is (Result.Wants_Refinement);
   function Has_Refinement_Diagnostics (Result : Three_View_Landmark_Result) return Boolean is (Result.Has_Refinement);
   function Refinement_Diagnostics (Result : Three_View_Landmark_Result)
      return Metric_Point_Refinement_Result is
   begin
      if not Result.Has_Refinement then
         raise OpenCV.OpenCV_Error with "Three-view refinement diagnostics unavailable";
      end if;
      return Result.Refined;
   end Refinement_Diagnostics;
   function Candidate_Source (Result : Three_View_Landmark_Result)
      return Three_View_Candidate_Source is (Result.Selected);
   function Selected_World_Position (Result : Three_View_Landmark_Result) return Object_Point is
   begin
      if Result.Selected = No_Selected_Candidate then
         raise OpenCV.OpenCV_Error with "Three-view fitting candidate unavailable";
      end if;
      return Result.Candidate;
   end Selected_World_Position;
   function Third_View_Evaluated (Result : Three_View_Landmark_Result) return Boolean is (Result.Has_Third);
   function Third_View_Disposition (Result : Three_View_Landmark_Result)
      return AP3P_Reference_Disposition is
   begin
      if not Result.Has_Third then
         raise OpenCV.OpenCV_Error with "Third-view evaluation unavailable";
      end if;
      return Result.Third.Disposition;
   end Third_View_Disposition;
   function Third_Camera_Depth (Result : Three_View_Landmark_Result) return OpenCV.Float64_Value is
   begin
      if Third_View_Disposition (Result) = Numerically_Unavailable then
         raise OpenCV.OpenCV_Error with "Third-view depth unavailable";
      end if;
      return Result.Third.Camera_Depth;
   end Third_Camera_Depth;
   function Third_Pixel_Error (Result : Three_View_Landmark_Result) return OpenCV.Float64_Value is
   begin
      if Third_View_Disposition (Result) /= Projected_Positive_Depth then
         raise OpenCV.OpenCV_Error with "Third-view pixel error unavailable";
      end if;
      return Result.Third.Error_Pixels;
   end Third_Pixel_Error;
   function Accepted (Result : Three_View_Landmark_Result) return Boolean is (Result.State = Three_View_Accepted);
   function Accepted_World_Position (Result : Three_View_Landmark_Result) return Object_Point is
   begin
      if not Accepted (Result) then
         raise OpenCV.OpenCV_Error with "Three-view landmark not accepted";
      end if;
      return Selected_World_Position (Result);
   end Accepted_World_Position;

   function Qualify_Three_View_Landmarks
     (First_Pixels, Second_Pixels, Third_Pixels : Image_Point_Array;
      First_Intrinsics, Second_Intrinsics, Third_Intrinsics : Camera_Intrinsics;
      First_Distortion, Second_Distortion, Third_Distortion : Distortion_Coefficients;
      First_Pose, Second_Pose, Third_Pose : World_To_Camera_Pose;
      Options : Three_View_Landmark_Options) return Three_View_Landmark_Result_Array
   is
   begin
      --  Validate the COMPLETE request before any native call or early return.
      Validate (First_Pixels); Validate (Second_Pixels); Validate (Third_Pixels);
      Validate (First_Intrinsics); Validate (Second_Intrinsics); Validate (Third_Intrinsics);
      Validate (First_Distortion); Validate (Second_Distortion); Validate (Third_Distortion);
      Validate (First_Pose); Validate (Second_Pose); Validate (Third_Pose);
      Validate (Options.Metric);
      Validate (Options.Refinement, Options.Refinement_Acceptance);
      if First_Pixels'Length /= Second_Pixels'Length or else
        First_Pixels'Length /= Third_Pixels'Length or else
        First_Pixels'Length > Interfaces.Integer_32'Last
      then
         raise OpenCV.OpenCV_Error with "Three-view counts must match and fit INT32";
      end if;
      for Limit of OpenCV.Core.Float64_Vec2.Vector'
        [Options.Maximum_Normalized_Sampson_Error, Options.Maximum_Third_Pixel_Error]
      loop
         if not Is_Finite (Limit) or else Limit < 0.0 then
            raise OpenCV.OpenCV_Error with "Invalid three-view acceptance threshold";
         end if;
      end loop;
      if First_Pixels'Length = 0 then
         return [1 .. 0 => <>];
      end if;
      declare
         --  Deliberately disable whole-set RMS rejection. Individual errors,
         --  NOT Accepted(Screen), decide eligibility even if a peer is undefined.
         Screen : constant Known_Pose_Epipolar_Result := Verify_Known_Pose_Epipolar
           (First_Pixels, Second_Pixels, First_Intrinsics, Second_Intrinsics,
            First_Distortion, Second_Distortion, First_Pose, Second_Pose,
            (1, OpenCV.Float64_Value'Last, Options.Maximum_Normalized_Sampson_Error));
         Errors : constant Epipolar_Correspondence_Result_Array := Correspondence_Errors (Screen);
         Eligible : Natural := 0;
      begin
         return Result : Three_View_Landmark_Result_Array (1 .. First_Pixels'Length) do
            for I in Result'Range loop
               Result (I).Position := I;
               Result (I).Wants_Refinement := Options.Source_Mode /= Initial_Only;
               Result (I).Sampson := Errors (I);
               if not Geometry (Screen).Defined then
                  Result (I).State := No_Stereo_Baseline;
               elsif not Errors (I).Defined then
                  Result (I).Screening := Epipolar_Metric_Undefined;
               elsif not Errors (I).Passes_Point_Limit then
                  Result (I).Screening := Epipolar_Limit_Rejected;
                  Result (I).State := Epipolar_Rejected;
               else
                  Result (I).Screening := Epipolar_Passed;
                  Eligible := Eligible + 1;
               end if;
            end loop;
            if Eligible > 0 then
               declare
                  P, Q : Image_Point_Array (1 .. Eligible);
                  Mapping : Inlier_Index_Array (1 .. Eligible);
                  N : Natural := 0;
               begin
                  for I in Result'Range loop
                     if Result (I).Screening = Epipolar_Passed then
                        N := N + 1;
                        Mapping (N) := I;
                        P (N) := First_Pixels (First_Pixels'First + (I - 1));
                        Q (N) := Second_Pixels (Second_Pixels'First + (I - 1));
                     end if;
                  end loop;
                  if Options.Source_Mode = Initial_Only then
                     declare
                        Initial : constant Metric_Triangulated_Point_Array := Triangulate_World_Points
                          (P, Q, First_Intrinsics, Second_Intrinsics, First_Distortion,
                           Second_Distortion, First_Pose, Second_Pose, Options.Metric);
                     begin
                        for J in Mapping'Range loop
                           Result (Mapping (J)).Initial := Initial (J);
                           Result (Mapping (J)).Has_Initial := True;
                        end loop;
                     end;
                  else
                     declare
                        Refined : constant Metric_Point_Refinement_Result_Array := Refine_Metric_World_Points
                          (P, Q, First_Intrinsics, Second_Intrinsics, First_Distortion,
                           Second_Distortion, First_Pose, Second_Pose, Options.Metric,
                           Options.Refinement, Options.Refinement_Acceptance);
                     begin
                        for J in Mapping'Range loop
                           Result (Mapping (J)).Refined := Refined (J);
                           Result (Mapping (J)).Has_Refinement := True;
                           Result (Mapping (J)).Initial := Original_Reconstruction (Refined (J));
                           Result (Mapping (J)).Has_Initial := True;
                        end loop;
                     end;
                  end if;
                  N := 0;
                  --  Candidate selection is complete before third pixels are read.
                  for J in Mapping'Range loop
                     declare
                        R : Three_View_Landmark_Result renames Result (Mapping (J));
                     begin
                        if R.Has_Refinement and then Accepted (R.Refined) then
                           R.Candidate := Accepted_World_Position (R.Refined);
                           R.Selected := Refined_Metric_Candidate;
                        elsif Options.Source_Mode = Require_Accepted_Refinement then
                           R.State := Refinement_Required_Unavailable;
                        elsif R.Initial.Disposition = Metric_Accepted and then R.Initial.Geometry.Available then
                           R.Candidate := R.Initial.Geometry.World_Position;
                           R.Selected := Initial_Metric_Candidate;
                        elsif R.Initial.Disposition = Metric_Policy_Rejected then
                           R.State := Stereo_Quality_Rejected;
                        else
                           R.State := Stereo_Source_Unusable;
                        end if;
                        if R.Selected /= No_Selected_Candidate then
                           N := N + 1;
                        end if;
                     end;
                  end loop;
                  if N > 0 then
                     declare
                        World : Object_Point_Array (1 .. N);
                        Pixels : Image_Point_Array (1 .. N);
                        Selected_Map : Inlier_Index_Array (1 .. N);
                     begin
                        N := 0;
                        for I in Result'Range loop
                           if Result (I).Selected /= No_Selected_Candidate then
                              N := N + 1;
                              Selected_Map (N) := I;
                              World (N) := Selected_World_Position (Result (I));
                              Pixels (N) := Third_Pixels (Third_Pixels'First + (I - 1));
                           end if;
                        end loop;
                        declare
                           Values : constant AP3P_Reference_Result_Matrix := Evaluate_AP3P_References
                             ([(Pose => Third_Pose)], World, Pixels, Third_Intrinsics, Third_Distortion);
                        begin
                           for J in Selected_Map'Range loop
                              declare
                                 R : Three_View_Landmark_Result renames Result (Selected_Map (J));
                              begin
                                 R.Third := Values (1, J);
                                 R.Has_Third := True;
                                 case R.Third.Disposition is
                                    when Non_Positive_Depth => R.State := Third_Non_Positive_Depth;
                                    when Numerically_Unavailable => R.State := Third_Numerically_Unavailable;
                                    when Projected_Positive_Depth =>
                                       R.State := (if R.Third.Error_Pixels <= Options.Maximum_Third_Pixel_Error
                                         then Three_View_Accepted else Third_Reprojection_Rejected);
                                 end case;
                              end;
                           end loop;
                        end;
                     end;
                  end if;
               end;
            end if;
         end return;
      end;
   end Qualify_Three_View_Landmarks;

   function Three_View_Landmark_Observations
     (Landmarks : Three_View_Landmark_Result_Array; Pixels : Image_Point_Array)
      return Accepted_Landmark_Set
   is
      Count : Natural := 0;
   begin
      if Landmarks'Length /= Pixels'Length then
         raise OpenCV.OpenCV_Error with "Three-view adapter pixel count mismatch";
      end if;
      Validate (Pixels);
      for Landmark of Landmarks loop
         if Accepted (Landmark) then
            Count := Count + 1;
         end if;
      end loop;
      return Result : Accepted_Landmark_Set (Count) do
         Count := 0;
         for I in 1 .. Landmarks'Length loop
            declare
               R : Three_View_Landmark_Result renames Landmarks (Landmarks'First + (I - 1));
            begin
               if Accepted (R) then
                  Count := Count + 1;
                  Result.World (Count) := Accepted_World_Position (R);
                  Result.Pixels (Count) := Pixels (Pixels'First + (I - 1));
                  Result.Positions (Count) := Original_Position (R);
               end if;
            end;
         end loop;
      end return;
   end Three_View_Landmark_Observations;

   function Original_Position (Result : Plane_Correspondence_Result) return Positive is (Result.Position);
   function Disposition (Result : Plane_Correspondence_Result)
      return Plane_Correspondence_Disposition is (Result.State);
   function Source_Intersection_Evaluated (Result : Plane_Correspondence_Result) return Boolean is
     (Result.Has_Source);
   function Source_Intersection (Result : Plane_Correspondence_Result)
      return World_Plane_Intersection is
   begin
      if not Result.Has_Source then
         raise OpenCV.OpenCV_Error with "Plane correspondence source intersection unavailable";
      end if;
      return Result.Source;
   end Source_Intersection;
   function Second_View_Evaluated (Result : Plane_Correspondence_Result) return Boolean is (Result.Has_Second);
   function Second_View_Disposition (Result : Plane_Correspondence_Result)
      return AP3P_Reference_Disposition is
   begin
      if not Result.Has_Second then
         raise OpenCV.OpenCV_Error with "Plane correspondence second view unavailable";
      end if;
      return Result.Second.Disposition;
   end Second_View_Disposition;
   function Second_Camera_Depth (Result : Plane_Correspondence_Result) return OpenCV.Float64_Value is
   begin
      if Second_View_Disposition (Result) = Numerically_Unavailable then
         raise OpenCV.OpenCV_Error with "Plane correspondence second depth unavailable";
      end if;
      return Result.Second.Camera_Depth;
   end Second_Camera_Depth;
   function Second_Pixel_Error (Result : Plane_Correspondence_Result) return OpenCV.Float64_Value is
   begin
      if Second_View_Disposition (Result) /= Projected_Positive_Depth then
         raise OpenCV.OpenCV_Error with "Plane correspondence second pixel error unavailable";
      end if;
      return Result.Second.Error_Pixels;
   end Second_Pixel_Error;
   function Accepted (Result : Plane_Correspondence_Result) return Boolean is
     (Result.State = Plane_Correspondence_Accepted);
   function Accepted_World_Position (Result : Plane_Correspondence_Result) return Object_Point is
   begin
      if not Accepted (Result) then
         raise OpenCV.OpenCV_Error with "Plane correspondence not accepted";
      end if;
      return Accepted_World_Position (Source_Intersection (Result));
   end Accepted_World_Position;

   procedure Validate
     (Plane : World_Plane; Options : Plane_Correspondence_Verification_Options) is
   begin
      Validate (Plane, Options.Intersection);
      if not Is_Finite (Options.Maximum_Second_Pixel_Error)
        or else Options.Maximum_Second_Pixel_Error < 0.0
      then
         raise OpenCV.OpenCV_Error with "Invalid plane correspondence pixel limit";
      end if;
   end Validate;

   function Verify_Plane_Constrained_Correspondences
     (First_Pixels, Second_Pixels : Image_Point_Array;
      First_Intrinsics, Second_Intrinsics : Camera_Intrinsics;
      First_Distortion, Second_Distortion : Distortion_Coefficients;
      First_Pose, Second_Pose : World_To_Camera_Pose;
      Plane : World_Plane; Options : Plane_Correspondence_Verification_Options)
      return Plane_Correspondence_Result_Array
   is
   begin
      if First_Pixels'Length /= Second_Pixels'Length then
         raise OpenCV.OpenCV_Error with "Plane correspondence pixel counts differ";
      end if;
      if Interfaces.Unsigned_64 (First_Pixels'Length) >
        Interfaces.Unsigned_64 (Interfaces.Integer_32'Last)
      then
         raise OpenCV.OpenCV_Error with "Too many plane correspondences";
      end if;
      Validate (First_Pixels);
      Validate (Second_Pixels);
      Validate (First_Intrinsics);
      Validate (Second_Intrinsics);
      Validate (First_Distortion);
      Validate (Second_Distortion);
      Validate (First_Pose);
      Validate (Second_Pose);
      Validate (Plane, Options);
      declare
         --  Exactly one source call; Camera B never participates in geometry.
         Sources : constant World_Plane_Intersection_Array := Unproject_Pixels_Onto_World_Plane
           (First_Pixels, First_Intrinsics, First_Distortion, First_Pose, Plane, Options.Intersection);
         Count : Natural := 0;
      begin
         for Source of Sources loop
            if Accepted (Source) then
               Count := Count + 1;
            end if;
         end loop;
         return Results : Plane_Correspondence_Result_Array (1 .. First_Pixels'Length) do
            for I in Results'Range loop
               Results (I).Position := I;
               Results (I).Source := Sources (I);
               Results (I).Has_Source := True;
            end loop;
            if Count > 0 then
               declare
                  World : Object_Point_Array (1 .. Count);
                  Pixels : Image_Point_Array (1 .. Count);
                  Mapping : Inlier_Index_Array (1 .. Count);
                  N : Natural := 0;
               begin
                  for I in Results'Range loop
                     if Accepted (Sources (I)) then
                        N := N + 1;
                        World (N) := Accepted_World_Position (Sources (I));
                        Pixels (N) := Second_Pixels (Second_Pixels'First + (I - 1));
                        Mapping (N) := I;
                     end if;
                  end loop;
                  declare
                     --  One evaluator invocation, one hypothesis, and at most
                     --  one positive-depth native projection batch. No handler:
                     --  native failures abort atomically, not per-point rejection.
                     Values : constant AP3P_Reference_Result_Matrix := Evaluate_AP3P_References
                       ([(Pose => Second_Pose)], World, Pixels, Second_Intrinsics, Second_Distortion);
                  begin
                     for J in Mapping'Range loop
                        declare
                           R : Plane_Correspondence_Result renames Results (Mapping (J));
                        begin
                           R.Second := Values (1, J);
                           R.Has_Second := True;
                           case R.Second.Disposition is
                              when Non_Positive_Depth => R.State := Second_Non_Positive_Depth;
                              when Numerically_Unavailable => R.State := Second_Numerically_Unavailable;
                              when Projected_Positive_Depth =>
                                 R.State := (if R.Second.Error_Pixels <= Options.Maximum_Second_Pixel_Error
                                   then Plane_Correspondence_Accepted else Second_Reprojection_Rejected);
                           end case;
                        end;
                     end loop;
                  end;
               end;
            end if;
         end return;
      end;
   end Verify_Plane_Constrained_Correspondences;

   function Plane_Constrained_Landmark_Observations
     (Landmarks : Plane_Correspondence_Result_Array; Pixels : Image_Point_Array)
      return Accepted_Landmark_Set
   is
      Count : Natural := 0;
   begin
      if Landmarks'Length /= Pixels'Length then
         raise OpenCV.OpenCV_Error with "Plane correspondence adapter pixel count mismatch";
      end if;
      Validate (Pixels);
      for Landmark of Landmarks loop
         if Accepted (Landmark) then
            Count := Count + 1;
         end if;
      end loop;
      return Result : Accepted_Landmark_Set (Count) do
         Count := 0;
         for I in 1 .. Landmarks'Length loop
            declare
               R : Plane_Correspondence_Result renames Landmarks (Landmarks'First + (I - 1));
            begin
               if Accepted (R) then
                  Count := Count + 1;
                  Result.World (Count) := Accepted_World_Position (R);
                  Result.Pixels (Count) := Pixels (Pixels'First + (I - 1));
                  Result.Positions (Count) := Original_Position (R);
               end if;
            end;
         end loop;
      end return;
   end Plane_Constrained_Landmark_Observations;

   function Original_Position (Result : Reciprocal_Plane_Result) return Positive is (Result.Position);
   function Disposition (Result : Reciprocal_Plane_Result) return Reciprocal_Plane_Disposition is (Result.State);
   function Directions_Evaluated (Result : Reciprocal_Plane_Result) return Boolean is (Result.Has_Directions);
   function Forward_Verification (Result : Reciprocal_Plane_Result) return Plane_Correspondence_Result is
   begin
      if not Directions_Evaluated (Result) then
         raise OpenCV.OpenCV_Error with "Reciprocal forward verification unavailable";
      end if;
      return Result.Forward;
   end Forward_Verification;
   function Reverse_Verification (Result : Reciprocal_Plane_Result) return Plane_Correspondence_Result is
   begin
      if not Directions_Evaluated (Result) then
         raise OpenCV.OpenCV_Error with "Reciprocal reverse verification unavailable";
      end if;
      return Result.Backward;
   end Reverse_Verification;
   function Both_Source_Intersections_Accepted (Result : Reciprocal_Plane_Result) return Boolean is
     (Result.Has_Directions and then Accepted (Source_Intersection (Result.Forward))
      and then Accepted (Source_Intersection (Result.Backward)));
   function World_Separation_Evaluated (Result : Reciprocal_Plane_Result) return Boolean is (Result.Has_Separation);
   function World_Separation (Result : Reciprocal_Plane_Result) return OpenCV.Float64_Value is
   begin
      if not World_Separation_Evaluated (Result) then
         raise OpenCV.OpenCV_Error with "Reciprocal world separation unavailable";
      end if;
      return Result.Separation;
   end World_Separation;
   function First_Source_World_Position (Result : Reciprocal_Plane_Result) return Object_Point is
     (Accepted_World_Position (Source_Intersection (Forward_Verification (Result))));
   function Second_Source_World_Position (Result : Reciprocal_Plane_Result) return Object_Point is
     (Accepted_World_Position (Source_Intersection (Reverse_Verification (Result))));
   function Accepted (Result : Reciprocal_Plane_Result) return Boolean is
     (Result.State = Reciprocal_Plane_Accepted);
   function Accepted_First_Source_World_Position (Result : Reciprocal_Plane_Result) return Object_Point is
   begin
      if not Accepted (Result) then
         raise OpenCV.OpenCV_Error with "Reciprocal correspondence not accepted";
      end if;
      return First_Source_World_Position (Result);
   end Accepted_First_Source_World_Position;
   function Accepted_Second_Source_World_Position (Result : Reciprocal_Plane_Result) return Object_Point is
   begin
      if not Accepted (Result) then
         raise OpenCV.OpenCV_Error with "Reciprocal correspondence not accepted";
      end if;
      return Second_Source_World_Position (Result);
   end Accepted_Second_Source_World_Position;

   function Verify_Reciprocal_Plane_Correspondences
     (First_Pixels, Second_Pixels : Image_Point_Array;
      First_Intrinsics, Second_Intrinsics : Camera_Intrinsics;
      First_Distortion, Second_Distortion : Distortion_Coefficients;
      First_Pose, Second_Pose : World_To_Camera_Pose;
      Plane : World_Plane; Options : Reciprocal_Plane_Verification_Options)
      return Reciprocal_Plane_Result_Array
   is
   begin
      --  Complete validation of BOTH policies precedes either directional call,
      --  even for empty requests. Reuse the exact Task 025 policy validation.
      if First_Pixels'Length /= Second_Pixels'Length then
         raise OpenCV.OpenCV_Error with "Reciprocal pixel counts differ";
      end if;
      if Interfaces.Unsigned_64 (First_Pixels'Length) >
        Interfaces.Unsigned_64 (Interfaces.Integer_32'Last)
      then
         raise OpenCV.OpenCV_Error with "Too many reciprocal correspondences";
      end if;
      Validate (First_Pixels);
      Validate (Second_Pixels);
      Validate (First_Intrinsics);
      Validate (Second_Intrinsics);
      Validate (First_Distortion);
      Validate (Second_Distortion);
      Validate (First_Pose);
      Validate (Second_Pose);
      Validate (Plane, Options.Forward);
      Validate (Plane, Options.Backward);
      if not Is_Finite (Options.Maximum_World_Separation) or else Options.Maximum_World_Separation < 0.0 then
         raise OpenCV.OpenCV_Error with "Invalid reciprocal world separation limit";
      end if;
      if First_Pixels'Length = 0 then
         return [1 .. 0 => <>];
      end if;
      declare
         --  Exactly one whole-array call per direction, never per match.
         Forward : constant Plane_Correspondence_Result_Array := Verify_Plane_Constrained_Correspondences
           (First_Pixels, Second_Pixels, First_Intrinsics, Second_Intrinsics,
            First_Distortion, Second_Distortion, First_Pose, Second_Pose, Plane, Options.Forward);
         Backward : constant Plane_Correspondence_Result_Array := Verify_Plane_Constrained_Correspondences
           (Second_Pixels, First_Pixels, Second_Intrinsics, First_Intrinsics,
            Second_Distortion, First_Distortion, Second_Pose, First_Pose, Plane, Options.Backward);
      begin
         return Results : Reciprocal_Plane_Result_Array (1 .. First_Pixels'Length) do
            for I in Results'Range loop
               declare
                  R : Reciprocal_Plane_Result renames Results (I);
                  F : constant Boolean := Accepted (Source_Intersection (Forward (I)));
                  B : constant Boolean := Accepted (Source_Intersection (Backward (I)));
               begin
                  R.Position := I;
                  R.Forward := Forward (I);
                  R.Backward := Backward (I);
                  R.Has_Directions := True;
                  if F and then B then
                     declare
                        --  Guarded accessors are OUTSIDE the arithmetic handler.
                        PA : constant Object_Point := First_Source_World_Position (R);
                        PB : constant Object_Point := Second_Source_World_Position (R);
                        Difference : Object_Point;
                     begin
                        --  Only checked subtraction and scaled norm can translate
                        --  OpenCV_Error here. No native calls or guarded reads.
                        begin
                           for Axis in 0 .. 2 loop
                              Difference (Axis) := Checked_Difference (PA (Axis), PB (Axis));
                           end loop;
                           R.Separation := Scaled_Vector_Norm (Difference);
                           R.Has_Separation := True;
                        exception
                           when OpenCV.OpenCV_Error => null;
                        end;
                     end;
                  end if;
                  --  Separation is diagnostic even when target verification fails.
                  if not F and then not B then
                     R.State := Both_Sources_Rejected;
                  elsif not F then
                     R.State := Forward_Source_Rejected;
                  elsif not B then
                     R.State := Reverse_Source_Rejected;
                  elsif not Accepted (Forward (I)) and then not Accepted (Backward (I)) then
                     R.State := Both_Directions_Rejected;
                  elsif not Accepted (Forward (I)) then
                     R.State := Forward_Verification_Rejected;
                  elsif not Accepted (Backward (I)) then
                     R.State := Reverse_Verification_Rejected;
                  elsif not R.Has_Separation then
                     R.State := World_Separation_Numerically_Unavailable;
                  elsif R.Separation > Options.Maximum_World_Separation then
                     R.State := World_Separation_Rejected;
                  else
                     R.State := Reciprocal_Plane_Accepted;
                  end if;
               end;
            end loop;
         end return;
      end;
   end Verify_Reciprocal_Plane_Correspondences;

   function Reciprocal_Plane_Landmark_Observations
     (Landmarks : Reciprocal_Plane_Result_Array; Pixels : Image_Point_Array) return Accepted_Landmark_Set
   is
      Count : Natural := 0;
   begin
      if Landmarks'Length /= Pixels'Length then
         raise OpenCV.OpenCV_Error with "Reciprocal adapter pixel count mismatch";
      end if;
      Validate (Pixels);
      for Landmark of Landmarks loop
         if Accepted (Landmark) then
            Count := Count + 1;
         end if;
      end loop;
      return Result : Accepted_Landmark_Set (Count) do
         Count := 0;
         for I in 1 .. Landmarks'Length loop
            declare
               R : Reciprocal_Plane_Result renames Landmarks (Landmarks'First + (I - 1));
            begin
               if Accepted (R) then
                  Count := Count + 1;
                  Result.World (Count) := Accepted_First_Source_World_Position (R);
                  Result.Pixels (Count) := Pixels (Pixels'First + (I - 1));
                  Result.Positions (Count) := Original_Position (R);
               end if;
            end;
         end loop;
      end return;
   end Reciprocal_Plane_Landmark_Observations;

   function World_Points (Landmarks : Accepted_Landmark_Set) return Object_Point_Array is (Landmarks.World);
   function Image_Points (Landmarks : Accepted_Landmark_Set) return Image_Point_Array is (Landmarks.Pixels);
   function Original_Positions (Landmarks : Accepted_Landmark_Set) return Inlier_Index_Array is (Landmarks.Positions);

   function Accepted_Landmark_Observations
     (Landmarks : Metric_Point_Refinement_Result_Array; Pixels : Image_Point_Array)
      return Accepted_Landmark_Set
   is
      Count : Natural := 0;
   begin
      if Landmarks'Length /= Pixels'Length then
         raise OpenCV.OpenCV_Error with "Landmark adapter world/pixel counts differ";
      end if;
      Validate (Pixels);
      for Landmark of Landmarks loop
         if Accepted (Landmark) then
            Count := Count + 1;
         end if;
      end loop;
      return Result : Accepted_Landmark_Set (Count) do
         Count := 0;
         for Position in 1 .. Landmarks'Length loop
            declare
               Landmark : Metric_Point_Refinement_Result renames
                 Landmarks (Landmarks'First + (Position - 1));
            begin
               if Accepted (Landmark) then
                  Count := Count + 1;
                  Result.World (Count) := Accepted_World_Position (Landmark);
                  Result.Pixels (Count) := Pixels (Pixels'First + (Position - 1));
                  Result.Positions (Count) := Position;
               end if;
            end;
         end loop;
      end return;
   end Accepted_Landmark_Observations;

   function Relocalize_Camera
     (Fitting_World : Object_Point_Array; Fitting_Pixels : Image_Point_Array;
      Holdout_World : Object_Point_Array; Holdout_Pixels : Image_Point_Array;
      Intrinsics : Camera_Intrinsics; Distortion : Distortion_Coefficients;
      RANSAC : RANSAC_Options; Verification : Relocalization_Verification_Options)
      return Camera_Relocalization_Result
   is
      procedure Require_Unique (Points : Object_Point_Array) is
      begin
         for Left in Points'Range loop
            for Right in Points'First .. Left - 1 loop
               if Same_World_Point (Points (Left), Points (Right)) then
                  raise OpenCV.OpenCV_Error with "Relocalization repeats a world landmark";
               end if;
            end loop;
         end loop;
      end Require_Unique;

      --  Existing evaluator obtains native R once, checks all camera coordinates,
      --  and calls native Project_Points only at positive depth. Its arithmetic
      --  unavailability does NOT swallow native exceptions. Never publish its
      --  partial-set summary as a whole-set score.
      procedure Evaluate
        (Pose : World_To_Camera_Pose; World : Object_Point_Array;
         Pixels : Image_Point_Array; Valid : out Boolean;
         Summary : out Reprojection_Summary; Errors : out Reprojection_Error_Array)
      is
         Values : constant AP3P_Reference_Result_Matrix :=
           Evaluate_AP3P_References ([(Pose => Pose)], World, Pixels, Intrinsics, Distortion);
      begin
         Valid := False;
         Summary := (others => <>);
         Errors := [others => 0.0];
         for I in 1 .. World'Length loop
            if Values (1, I).Disposition /= Projected_Positive_Depth then
               return;
            end if;
         end loop;
         for I in 1 .. World'Length loop
            Errors (Errors'First + (I - 1)) := Values (1, I).Error_Pixels;
         end loop;
         Summary := Summarize_Reprojection (Errors);
         Valid := True;
      end Evaluate;
   begin
      --  Validate everything before the insufficient-fitting early return.
      Validate (Intrinsics);
      Validate (Distortion);
      Validate (RANSAC);
      Validate (Fitting_World);
      Validate (Fitting_Pixels);
      Validate (Holdout_World);
      Validate (Holdout_Pixels);
      Validate (AP3P_Refinement_Acceptance_Options'
        (Verification.Maximum_Holdout_RMS_Pixels,
         Verification.Maximum_Holdout_Point_Error_Pixels,
         Verification.Maximum_Holdout_RMS_Worsening_Pixels));
      if Verification.Minimum_Consensus_Inliers < 4
        or else not Is_Finite (Verification.Maximum_Final_Inlier_RMS_Pixels)
        or else Verification.Maximum_Final_Inlier_RMS_Pixels < 0.0
      then
         raise OpenCV.OpenCV_Error with "Invalid relocalization verification policy";
      end if;
      if Fitting_World'Length /= Fitting_Pixels'Length
        or else Holdout_World'Length /= Holdout_Pixels'Length
        or else Holdout_World'Length = 0
        or else Fitting_World'Length > Natural (Interfaces.Integer_32'Last)
        or else Holdout_World'Length > Natural (Interfaces.Integer_32'Last)
      then
         raise OpenCV.OpenCV_Error with "Invalid relocalization correspondence counts";
      end if;
      Require_Unique (Fitting_World);
      Require_Unique (Holdout_World);
      for Point of Holdout_World loop
         for Other of Fitting_World loop
            if Same_World_Point (Point, Other) then
               raise OpenCV.OpenCV_Error with "Relocalization holdout overlaps fitting world";
            end if;
         end loop;
      end loop;
      if Fitting_World'Length < 6 then
         return Result : Camera_Relocalization_Result (0);
      end if;
      declare
         Estimate : constant Pose_Estimate := Solve_PnP_RANSAC
           (Fitting_World, Fitting_Pixels, Intrinsics, Distortion, RANSAC);
         Positions : constant Inlier_Index_Array := Inliers (Estimate);
      begin
         return Result : Camera_Relocalization_Result (Positions'Length) do
            Result.State := No_Native_RANSAC_Pose;
            if not Found (Estimate) then
               return;
            end if;
            Result.Has_Initial := True;
            Result.Initial := Pose (Estimate);
            Result.Positions := Positions;
            Result.State := Insufficient_Consensus_Inliers;
            if Positions'Length < Verification.Minimum_Consensus_Inliers then
               return;
            end if;
            declare
               World : Object_Point_Array (1 .. Positions'Length);
               Pixels : Image_Point_Array (1 .. Positions'Length);
               Holdout_Errors : Reprojection_Error_Array (1 .. Holdout_World'Length);
               Candidate : World_To_Camera_Pose := Result.Initial;
            begin
               Evaluate (Result.Initial, Holdout_World, Holdout_Pixels,
                         Result.Initial_Valid, Result.Initial_Summary, Holdout_Errors);
               Result.State := Initial_Holdout_Unusable;
               if not Result.Initial_Valid then
                  return;
               end if;
               for I in Positions'Range loop
                  World (I) := Fitting_World (Fitting_World'First + (Positions (I) - 1));
                  Pixels (I) := Fitting_Pixels (Fitting_Pixels'First + (Positions (I) - 1));
               end loop;
               Result.Attempted := True;
               Refine_Pose_Iterative
                 (World, Pixels, Intrinsics, Distortion, Candidate, Result.Native_Succeeded);
               Result.State := Native_Refinement_False;
               if not Result.Native_Succeeded then
                  return;
               end if;
               Evaluate (Candidate, Holdout_World, Holdout_Pixels,
                         Result.Refined_Valid, Result.Refined_Summary, Holdout_Errors);
               --  Evaluate both diagnostics before rejection, retaining any
               --  meaningful whole-set score even if the other set is unusable.
               Evaluate (Candidate, World, Pixels,
                         Result.Inliers_Valid, Result.Inlier_Summary, Result.Errors);
               Result.State := Refined_Holdout_Unusable;
               if not Result.Refined_Valid then
                  return;
               end if;
               Result.State := Final_Inlier_Verification_Unusable;
               if not Result.Inliers_Valid then
                  return;
               end if;
               Result.State := Rejected_By_Policy;
               if Result.Inlier_Summary.RMS_Error_Pixels > Verification.Maximum_Final_Inlier_RMS_Pixels
                 or else Result.Refined_Summary.RMS_Error_Pixels > Verification.Maximum_Holdout_RMS_Pixels
                 or else Result.Refined_Summary.Maximum_Error_Pixels > Verification.Maximum_Holdout_Point_Error_Pixels
                 or else not OpenCV.Calib3D.Internal.Refinement_Policy.Holdout_RMS_Worsening_Within_Allowance
                   (Result.Initial_Summary.RMS_Error_Pixels, Result.Refined_Summary.RMS_Error_Pixels,
                    Verification.Maximum_Holdout_RMS_Worsening_Pixels)
               then
                  return;
               end if;
               --  Stage every fallible accepted value before publishing State.
               Result.Center := Camera_Center (Candidate);
               Result.Final_Pose := Candidate;
               Result.State := Accepted;
            end;
         end return;
      end;
   end Relocalize_Camera;
end OpenCV.Calib3D;
