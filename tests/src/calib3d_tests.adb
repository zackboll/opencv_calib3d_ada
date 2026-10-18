with Ada.Exceptions;
with Ada.Numerics;
with Ada.Numerics.Generic_Elementary_Functions;
with Ada.Text_IO;
with AUnit.Assertions;
with AUnit.Test_Caller;
with AUnit.Test_Fixtures;
with Interfaces;
with Interfaces.C;
with Interfaces.C.Strings;
with OpenCV.Calib3D;
with OpenCV.Calib3D.Internal.C_API;
with OpenCV.Calib3D.Internal.Point_Refinement;
with OpenCV.Calib3D.Internal.Refinement_Policy;
with System;

package body Calib3D_Tests is
   use AUnit.Assertions;
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   use type Interfaces.Integer_32;
   use type Interfaces.C.double;
   use type Translation_Vector;
   use type Image_Point;

   package ABI renames OpenCV.Calib3D.Internal.C_API;
   package Math is new Ada.Numerics.Generic_Elementary_Functions
     (OpenCV.Float64_Value);
   function Nonfinite (Kind : Interfaces.Integer_32) return Interfaces.C.double
     with Import, Convention => C, External_Name => "calib3d_test_nonfinite";

   type Fixture is new AUnit.Test_Fixtures.Test_Fixture with null record;

   K : constant Camera_Intrinsics :=
     (Focal_X => 800.0, Focal_Y => 820.0, Center_X => 320.0, Center_Y => 240.0);
   Known_Pose : constant World_To_Camera_Pose :=
     (Rotation => [0 => 0.10, 1 => -0.05, 2 => 0.08],
      Translation => [0 => 0.20, 1 => -0.10, 2 => 6.00]);
   World : constant Object_Point_Array :=
     [[0 => -1.0, 1 => -1.0, 2 => 0.0],
      [0 =>  0.0, 1 => -1.0, 2 => 0.2],
      [0 =>  1.0, 1 => -1.0, 2 => 0.4],
      [0 => -1.2, 1 =>  0.0, 2 => 0.5],
      [0 =>  0.0, 1 =>  0.0, 2 => 0.8],
      [0 =>  1.2, 1 =>  0.0, 2 => 0.3],
      [0 => -1.0, 1 =>  1.0, 2 => 1.0],
      [0 =>  0.0, 1 =>  1.0, 2 => 1.3],
      [0 =>  1.0, 1 =>  1.0, 2 => 0.7],
      [0 => -0.5, 1 => -0.4, 2 => 1.7],
      [0 =>  0.6, 1 => -0.3, 2 => 1.9],
      [0 =>  0.3, 1 =>  0.7, 2 => 2.1],
      [0 => -1.4, 1 =>  0.6, 2 => 1.5],
      [0 =>  1.5, 1 =>  0.5, 2 => 1.2],
      [0 => -0.8, 1 => -1.4, 2 => 1.1],
      [0 =>  0.9, 1 => -1.3, 2 => 1.6],
      [0 => -1.5, 1 => -0.5, 2 => 2.0],
      [0 =>  1.4, 1 => -0.6, 2 => 2.2],
      [0 => -0.2, 1 =>  1.5, 2 => 1.8],
      [0 =>  0.8, 1 =>  1.4, 2 => 2.4]];

   function Finite (Value : OpenCV.Float64_Value) return Boolean is
      (Value = Value and then Value < OpenCV.Float64_Value'Last and then Value > OpenCV.Float64_Value'First);

   function Near (Left, Right : OpenCV.Float64_Value;
                  Epsilon : OpenCV.Float64_Value := 1.0E-9) return Boolean is
     (abs (Left - Right) <= Epsilon);

   function Contains (Values : Inlier_Index_Array; Value : Positive) return Boolean is
   begin
      for Item of Values loop
         if Item = Value then
            return True;
         end if;
      end loop;
      return False;
   end Contains;

   procedure Projection_Identity (T : in out Fixture) is
      pragma Unreferenced (T);
      Points : constant Object_Point_Array := [[0 => 1.0, 1 => 2.0, 2 => 10.0]];
      Intrinsics : constant Camera_Intrinsics :=
        (Focal_X => 100.0, Focal_Y => 200.0, Center_X => 10.0, Center_Y => 20.0);
      Identity : constant World_To_Camera_Pose :=
        (Rotation => [others => 0.0], Translation => [others => 0.0]);
      Result : constant Image_Point_Array := Project_Points (Points, Intrinsics, No_Distortion, Identity);
   begin
      Assert (Result'Length = 1, "projection result count");
      Assert (Near (Result (1) (0), 20.0) and then Near (Result (1) (1), 60.0),
              "identity pinhole projection differs");
   end Projection_Identity;

   procedure Projection_Translation (T : in out Fixture) is
      pragma Unreferenced (T);
      Points : constant Object_Point_Array := [[0 => 0.0, 1 => 0.0, 2 => 5.0]];
      Intrinsics : constant Camera_Intrinsics :=
        (Focal_X => 100.0, Focal_Y => 100.0, Center_X => 10.0, Center_Y => 20.0);
      Shift : constant World_To_Camera_Pose :=
        (Rotation => [others => 0.0], Translation => [0 => 1.0, 1 => 0.0, 2 => 0.0]);
      Result : constant Image_Point_Array := Project_Points (Points, Intrinsics, No_Distortion, Shift);
   begin
      Assert (Near (Result (1) (0), 30.0) and then Near (Result (1) (1), 20.0),
              "translation projection differs");
   end Projection_Translation;

   procedure Projection_Rotation (T : in out Fixture) is
      pragma Unreferenced (T);
      Points : constant Object_Point_Array := [[1.0, 2.0, 10.0]];
      Quarter_Turn : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, OpenCV.Float64_Value (Ada.Numerics.Pi / 2.0)],
         Translation => [others => 0.0]);
      Result : constant Image_Point_Array := Project_Points
        (Points, (100.0, 200.0, 10.0, 20.0), No_Distortion, Quarter_Turn);
   begin
      --  Independent Rz(pi/2): (X,Y,Z) -> (-Y,X,Z).
      Assert (Near (Result (1) (0), -10.0) and then Near (Result (1) (1), 40.0),
              "quarter-turn projection oracle differs");
   end Projection_Rotation;

   procedure Projection_Distortion (T : in out Fixture) is
      pragma Unreferenced (T);
      Points : constant Object_Point_Array := [[0 => 1.0, 1 => 2.0, 2 => 10.0]];
      Intrinsics : constant Camera_Intrinsics :=
        (Focal_X => 100.0, Focal_Y => 100.0, Center_X => 0.0, Center_Y => 0.0);
      Identity : constant World_To_Camera_Pose :=
        (Rotation => [others => 0.0], Translation => [others => 0.0]);
      Distortion : constant Distortion_Coefficients :=
        (K1 => 0.1, K2 => -0.02, P1 => 0.003, P2 => -0.004, K3 => 0.005);
      Result : constant Image_Point_Array := Project_Points (Points, Intrinsics, Distortion, Identity);
   begin
      --  x=.1, y=.2, r2=.05, radial=1+k1*r2+k2*r2^2+k3*r2^3.
      --  xd=x*radial+2*p1*x*y+p2*(r2+2*x^2); analogous yd.
      Assert (Near (Result (1) (0), 10.03350625, 1.0E-9)
              and then Near (Result (1) (1), 20.1220125, 1.0E-9),
              "five-coefficient distortion oracle differs");
   end Projection_Distortion;

   procedure Camera_Center_Translation (T : in out Fixture) is
      pragma Unreferenced (T);
      P : constant World_To_Camera_Pose :=
        (Rotation => [others => 0.0], Translation => [0 => 1.0, 1 => 2.0, 2 => 3.0]);
      Center : constant Object_Point := Camera_Center (P);
   begin
      Assert (Near (Center (0), -1.0) and then Near (Center (1), -2.0)
              and then Near (Center (2), -3.0), "identity camera center differs");
   end Camera_Center_Translation;

   procedure Camera_Center_Rotation (T : in out Fixture) is
      pragma Unreferenced (T);
      P : constant World_To_Camera_Pose :=
        (Rotation => [0 => 0.0, 1 => 0.0,
                      2 => OpenCV.Float64_Value (Ada.Numerics.Pi / 2.0)],
         Translation => [0 => 1.0, 1 => 2.0, 2 => 3.0]);
      Center : constant Object_Point := Camera_Center (P);
   begin
      Assert (Near (Center (0), -2.0, 1.0E-10)
              and then Near (Center (1), 1.0, 1.0E-10)
              and then Near (Center (2), -3.0, 1.0E-10),
              "camera center is not -R^T*t");
   end Camera_Center_Rotation;

   procedure PnP_Clean (T : in out Fixture) is
      pragma Unreferenced (T);
      Images : constant Image_Point_Array := Project_Points (World, K, No_Distortion, Known_Pose);
      Estimate : constant Pose_Estimate := Solve_PnP_RANSAC
        (World, Images, K, Options =>
           (Maximum_Iterations => 500, Reprojection_Error_Pixels => 1.0, Confidence => 0.999));
   begin
      Assert (Found (Estimate), "clean synthetic pose not found");
      Assert (Inlier_Count (Estimate) >= 4, "clean synthetic pose has too few inliers");
      declare
         Reprojected : constant Image_Point_Array := Project_Points (World, K, No_Distortion, Pose (Estimate));
         Expected_Center : constant Object_Point := Camera_Center (Known_Pose);
         Actual_Center : constant Object_Point := Camera_Center (Pose (Estimate));
      begin
         for I in Images'Range loop
            declare
               DX : constant OpenCV.Float64_Value := Reprojected (I) (0) - Images (I) (0);
               DY : constant OpenCV.Float64_Value := Reprojected (I) (1) - Images (I) (1);
            begin
               Assert (DX * DX + DY * DY < 0.25, "clean PnP reprojection error too large");
            end;
         end loop;
         for Axis in Expected_Center'Range loop
            Assert (Near (Actual_Center (Axis), Expected_Center (Axis), 0.05),
                    "clean PnP camera center differs");
         end loop;
      end;
   end PnP_Clean;

   procedure PnP_Outliers (T : in out Fixture) is
      pragma Unreferenced (T);
      Images : Image_Point_Array := Project_Points (World, K, No_Distortion, Known_Pose);
   begin
      declare
         type Index_List is array (Positive range <>) of Positive;
         Outliers : constant Index_List := [2, 7, 15];
      begin
         for I of Outliers loop
            Images (I) (0) := Images (I) (0) + 5_000.0;
            Images (I) (1) := Images (I) (1) - 4_000.0;
         end loop;
      end;
      declare
         Estimate : constant Pose_Estimate := Solve_PnP_RANSAC
           (World, Images, K, Options =>
              (Maximum_Iterations => 1_000, Reprojection_Error_Pixels => 2.0, Confidence => 0.999));
      begin
         Assert (Found (Estimate), "outlier RANSAC pose not found");
         Assert (Inlier_Count (Estimate) >= 4, "too few robust inliers");
         declare
            Accepted : constant Inlier_Index_Array := Inliers (Estimate);
            Reprojected : constant Image_Point_Array :=
              Project_Points (World, K, No_Distortion, Pose (Estimate));
            Expected_Center : constant Object_Point := Camera_Center (Known_Pose);
            Actual_Center : constant Object_Point := Camera_Center (Pose (Estimate));
            Max_Error_Squared : OpenCV.Float64_Value := 0.0;
         begin
            Assert (not Contains (Accepted, 2) and then not Contains (Accepted, 7)
                    and then not Contains (Accepted, 15), "gross synthetic outlier accepted");
            for I in Accepted'First + 1 .. Accepted'Last loop
               Assert (Accepted (I) > Accepted (I - 1), "inliers are not strictly ascending");
            end loop;
            for I of Accepted loop
               Assert (I in World'Range, "inlier outside correspondence range");
               declare
                  DX : constant OpenCV.Float64_Value := Reprojected (I) (0) - Images (I) (0);
                  DY : constant OpenCV.Float64_Value := Reprojected (I) (1) - Images (I) (1);
               begin
                  Max_Error_Squared := OpenCV.Float64_Value'Max (Max_Error_Squared, DX * DX + DY * DY);
               end;
            end loop;
            Assert (Max_Error_Squared < 0.25, "robust inlier reprojection exceeds 0.5 pixels");
            for Axis in Expected_Center'Range loop
               Assert (Near (Actual_Center (Axis), Expected_Center (Axis), 0.05),
                       "robust camera center differs");
            end loop;
            Ada.Text_IO.Put_Line ("robust PnP: inliers=" & Natural'Image (Accepted'Length) &
              ", rejected=2,7,15; max squared pixel error=" &
              OpenCV.Float64_Value'Image (Max_Error_Squared));
         end;
      end;
   end PnP_Outliers;

   procedure PnP_Not_Found (T : in out Fixture) is
      pragma Unreferenced (T);
      function Inconsistent_Images return Image_Point_Array is
         Result : Image_Point_Array (World'Range);
      begin
         for I in Result'Range loop
            Result (I) := [OpenCV.Float64_Value ((I * 7919) mod 997),
                           OpenCV.Float64_Value ((I * 104729) mod 991)];
         end loop;
         return Result;
      end Inconsistent_Images;
      --  More than five points reaches consensus rather than direct P3P/EPNP.
      Estimate : constant Pose_Estimate := Solve_PnP_RANSAC
        (World, Inconsistent_Images, K, Options => (100, 1.0E-6, 0.99));
   begin
      Assert (not Found (Estimate), "inconsistent fixture unexpectedly found a pose");
      Assert (Inlier_Count (Estimate) = 0 and then Inliers (Estimate)'Length = 0
              and then Inliers (Estimate)'First = 1 and then Inliers (Estimate)'Last = 0,
              "not-found estimate exposes inliers");
      begin
         declare
            Value : constant World_To_Camera_Pose := Pose (Estimate);
            pragma Unreferenced (Value);
         begin
            Assert (False, "not-found estimate exposes pose");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
   end PnP_Not_Found;

   procedure Invalid_Intrinsics (T : in out Fixture) is
      pragma Unreferenced (T);
      Points : constant Object_Point_Array := [[0 => 0.0, 1 => 0.0, 2 => 5.0]];
      Bad : constant Camera_Intrinsics :=
        (Focal_X => 0.0, Focal_Y => 100.0, Center_X => 10.0, Center_Y => 20.0);
      Identity : constant World_To_Camera_Pose :=
        (Rotation => [others => 0.0], Translation => [others => 0.0]);
   begin
      begin
         declare
            Result : constant Image_Point_Array := Project_Points (Points, Bad, No_Distortion, Identity);
            pragma Unreferenced (Result);
         begin
            Assert (False, "zero focal length accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
   end Invalid_Intrinsics;

   procedure Invalid_Options_And_Counts (T : in out Fixture) is
      pragma Unreferenced (T);
      Images : constant Image_Point_Array := Project_Points (World, K, No_Distortion, Known_Pose);
      Short_World : constant Object_Point_Array := World (1 .. 3);
      Short_Images : constant Image_Point_Array := Images (1 .. 3);
      procedure Expect_Error (Objects : Object_Point_Array; Pixels : Image_Point_Array;
                              Options : RANSAC_Options) is
      begin
         declare
            Result : constant Pose_Estimate := Solve_PnP_RANSAC (Objects, Pixels, K, Options => Options);
            pragma Unreferenced (Result);
         begin
            Assert (False, "invalid PnP request accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Expect_Error;
   begin
      Expect_Error (Short_World, Short_Images, (others => <>));
      Expect_Error (World, Images (1 .. Images'Last - 1), (others => <>));
      Expect_Error (World, Images, (Maximum_Iterations => 100,
                                    Reprojection_Error_Pixels => 0.0, Confidence => 0.99));
      Expect_Error (World, Images, (Maximum_Iterations => 100,
                                    Reprojection_Error_Pixels => 8.0, Confidence => 1.0));
      Expect_Error (World, Images, (100, 1.0E-300, 0.99));
   end Invalid_Options_And_Counts;

   procedure ABI_Layout (T : in out Fixture) is
      pragma Unreferenced (T);
      function Intrinsics_Layout (Field : Interfaces.Integer_32) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_intrinsics_layout";
      function Distortion_Layout (Field : Interfaces.Integer_32) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_distortion_layout";
      function Pose_Layout (Field : Interfaces.Integer_32) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_pose_layout";
      procedure Fill_Intrinsics (Value : access ABI.C_Camera_Intrinsics)
        with Import, Convention => C, External_Name => "calib3d_test_fill_intrinsics";
      procedure Fill_Distortion (Value : access ABI.C_Distortion5)
        with Import, Convention => C, External_Name => "calib3d_test_fill_distortion";
      procedure Fill_Pose (Value : access ABI.C_Pose)
        with Import, Convention => C, External_Name => "calib3d_test_fill_pose";
      Intrinsics : aliased ABI.C_Camera_Intrinsics;
      Distortion : aliased ABI.C_Distortion5;
      Native_Pose : aliased ABI.C_Pose;
      type Positions is array (Natural range <>) of Natural;
      Intrinsic_Offsets : constant Positions :=
        [Intrinsics.Focal_X'Position, Intrinsics.Focal_Y'Position,
         Intrinsics.Center_X'Position, Intrinsics.Center_Y'Position];
      Distortion_Offsets : constant Positions :=
        [Distortion.K1'Position, Distortion.K2'Position, Distortion.P1'Position,
         Distortion.P2'Position, Distortion.K3'Position];
      Pose_Offsets : constant Positions :=
        [Native_Pose.RX'Position, Native_Pose.RY'Position, Native_Pose.RZ'Position,
         Native_Pose.TX'Position, Native_Pose.TY'Position, Native_Pose.TZ'Position];
   begin
      Assert (ABI.C_Camera_Intrinsics'Size = Natural (Intrinsics_Layout (0)) * System.Storage_Unit,
              "intrinsics C/Ada size mismatch");
      Assert (ABI.C_Camera_Intrinsics'Alignment = Natural (Intrinsics_Layout (1)),
              "intrinsics C/Ada alignment mismatch");
      for I in Intrinsic_Offsets'Range loop
         Assert (Intrinsic_Offsets (I) = Natural (Intrinsics_Layout (Interfaces.Integer_32 (I + 2))),
                 "intrinsics field offset mismatch");
      end loop;
      Assert (ABI.C_Distortion5'Size = Natural (Distortion_Layout (0)) * System.Storage_Unit,
              "distortion C/Ada size mismatch");
      Assert (ABI.C_Distortion5'Alignment = Natural (Distortion_Layout (1)),
              "distortion C/Ada alignment mismatch");
      for I in Distortion_Offsets'Range loop
         Assert (Distortion_Offsets (I) = Natural (Distortion_Layout (Interfaces.Integer_32 (I + 2))),
                 "distortion field offset mismatch");
      end loop;
      Assert (ABI.C_Pose'Size = Natural (Pose_Layout (0)) * System.Storage_Unit,
              "pose C/Ada size mismatch");
      Assert (ABI.C_Pose'Alignment = Natural (Pose_Layout (1)), "pose C/Ada alignment mismatch");
      for I in Pose_Offsets'Range loop
         Assert (Pose_Offsets (I) = Natural (Pose_Layout (Interfaces.Integer_32 (I + 2))),
                 "pose field offset mismatch");
      end loop;
      Fill_Intrinsics (Intrinsics'Access);
      Fill_Distortion (Distortion'Access);
      Fill_Pose (Native_Pose'Access);
      Assert (Intrinsics.Focal_X = 800.0 and then Intrinsics.Focal_Y = 820.0
              and then Intrinsics.Center_X = 320.0 and then Intrinsics.Center_Y = 240.0,
              "C-written intrinsics interchange");
      Assert (Distortion.K1 = 0.1 and then Distortion.K2 = -0.2
              and then Distortion.P1 = 0.01 and then Distortion.P2 = -0.02
              and then Distortion.K3 = 0.03,
              "C-written distortion interchange");
      Assert (Native_Pose.RX = 0.1 and then Native_Pose.RY = 0.2
              and then Native_Pose.RZ = 0.3 and then Native_Pose.TX = 4.0
              and then Native_Pose.TY = 5.0 and then Native_Pose.TZ = 6.0,
              "C-written pose interchange");
   end ABI_Layout;

   procedure Diagnostic_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      Objects : constant Object_Point_Array (4 .. 6) :=
        [others => [0 => 1.0, 1 => 2.0, 2 => 10.0]];
      Pixels : constant Image_Point_Array (7 .. 9) :=
        [[0 => 17.0, 1 => 56.0], [0 => 20.0, 1 => 60.0], [0 => 25.0, 1 => 72.0]];
      Errors : constant Reprojection_Error_Array := Reprojection_Errors
        (Objects, Pixels, (100.0, 200.0, 10.0, 20.0), No_Distortion, (others => <>));
      Summary : constant Reprojection_Summary := Summarize_Reprojection (Errors);
   begin
      Assert (Errors'First = 1 and then Errors'Last = 3, "diagnostic result bounds");
      Assert (Near (Errors (1), 5.0, 1.0E-13) and then Errors (2) = 0.0
              and then Near (Errors (3), 13.0, 1.0E-13), "independent pixel offsets");
      Assert (Summary.Count = 3 and then Near (Summary.Maximum_Error_Pixels, 13.0, 1.0E-13)
              and then Near (Summary.RMS_Error_Pixels, Math.Sqrt (194.0 / 3.0), 1.0E-13),
              "pixel RMS/maximum oracle");
      Ada.Text_IO.Put_Line ("diagnostic oracle: errors=5,0,13 RMS=" &
        OpenCV.Float64_Value'Image (Summary.RMS_Error_Pixels) & " max=13");
   end Diagnostic_Oracle;

   procedure Diagnostic_Empty_And_Counts (T : in out Fixture) is
      pragma Unreferenced (T);
      Errors : constant Reprojection_Error_Array := Reprojection_Errors
        ([1 .. 0 => <>], [1 .. 0 => <>], K, No_Distortion, Known_Pose);
      Summary : constant Reprojection_Summary := Summarize_Reprojection (Errors);
   begin
      Assert (Errors'First = 1 and then Errors'Last = 0 and then Summary.Count = 0
              and then Summary.RMS_Error_Pixels = 0.0
              and then Summary.Maximum_Error_Pixels = 0.0, "empty diagnostic contract");
      begin
         declare
            Bad : constant Reprojection_Error_Array := Reprojection_Errors
              (World, [1 .. 0 => <>], K, No_Distortion, Known_Pose);
            pragma Unreferenced (Bad);
         begin
            Assert (False, "diagnostic count mismatch accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
   end Diagnostic_Empty_And_Counts;

   procedure Diagnostic_Range (T : in out Fixture) is
      pragma Unreferenced (T);
      --  Deliberately construct IEEE NaN/infinity from C to exercise binding
      --  validation, rather than GNAT rejecting the test fixture on assignment.
      pragma Suppress (Validity_Check);
      Objects : constant Object_Point_Array := [[0 => 0.0, 1 => 0.0, 2 => 1.0]];
      Errors : constant Reprojection_Error_Array := Reprojection_Errors
        (Objects, [[0 => 3.0E200, 1 => 4.0E200]], (1.0, 1.0, 0.0, 0.0),
         No_Distortion, (others => <>));
      Large : constant Reprojection_Summary := Summarize_Reprojection
        ([3 => OpenCV.Float64_Value'Last, 4 => OpenCV.Float64_Value'Last]);
      Tiny : constant Reprojection_Summary := Summarize_Reprojection ([1.0E-200, 1.0E-200]);
   begin
      Assert (abs (Errors (1) / 5.0E200 - 1.0) < 1.0E-14, "scaled hypot overflow");
      Assert (Large.RMS_Error_Pixels = OpenCV.Float64_Value'Last
              and then Large.Maximum_Error_Pixels = OpenCV.Float64_Value'Last,
              "scaled RMS overflow at Float64 last");
      Assert (abs (Tiny.RMS_Error_Pixels / 1.0E-200 - 1.0) < 1.0E-14,
              "scaled RMS underflow");
      for Kind in Interfaces.Integer_32 range 0 .. 2 loop
         begin
            declare
               Bad : constant Reprojection_Summary := Summarize_Reprojection
                 ([OpenCV.Float64_Value (Nonfinite (Kind))]);
               pragma Unreferenced (Bad);
            begin
               Assert (False, "nonfinite error accepted");
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end loop;
      begin
         declare
            Bad : constant Reprojection_Summary := Summarize_Reprojection ([-1.0]);
            pragma Unreferenced (Bad);
         begin
            Assert (False, "negative error accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
      begin
         declare
            Bad : constant Reprojection_Error_Array := Reprojection_Errors
              (Objects, [[0 => -OpenCV.Float64_Value'Last, 1 => 0.0]],
               (1.0, 1.0, OpenCV.Float64_Value'Last, 0.0), No_Distortion, (others => <>));
            pragma Unreferenced (Bad);
         begin
            Assert (False, "nonfinite subtraction accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
   end Diagnostic_Range;

   Perturbed : constant World_To_Camera_Pose :=
     (Rotation => [0.16, -0.09, 0.12], Translation => [0.45, -0.30, 6.40]);
   Distorted : constant Distortion_Coefficients := (0.10, -0.04, 0.003, -0.002, 0.01);

   function Center_Error (P : World_To_Camera_Pose) return OpenCV.Float64_Value is
      C1 : constant Object_Point := Camera_Center (P);
      C2 : constant Object_Point := Camera_Center (Known_Pose);
   begin
      return Math.Sqrt ((C1 (0) - C2 (0)) ** 2 + (C1 (1) - C2 (1)) ** 2 +
                        (C1 (2) - C2 (2)) ** 2);
   end Center_Error;

   procedure Report_Refinement
     (Name : String; Before, After : Reprojection_Summary;
      Center_Before, Center_After : OpenCV.Float64_Value) is
   begin
      Ada.Text_IO.Put_Line (Name & " before RMS/max=" &
        OpenCV.Float64_Value'Image (Before.RMS_Error_Pixels) & "/" &
        OpenCV.Float64_Value'Image (Before.Maximum_Error_Pixels) & " after RMS/max=" &
        OpenCV.Float64_Value'Image (After.RMS_Error_Pixels) & "/" &
        OpenCV.Float64_Value'Image (After.Maximum_Error_Pixels) & " center before/after=" &
        OpenCV.Float64_Value'Image (Center_Before) & "/" &
        OpenCV.Float64_Value'Image (Center_After));
   end Report_Refinement;

   procedure Check_Refinement (D : Distortion_Coefficients; Name : String) is
      Images : constant Image_Point_Array := Project_Points (World, K, D, Known_Pose);
      P : World_To_Camera_Pose := Perturbed;
      Before : constant Reprojection_Summary :=
        Summarize_Reprojection (Reprojection_Errors (World, Images, K, D, P));
      Center_Before : constant OpenCV.Float64_Value := Center_Error (P);
      Refined : Boolean := False;
   begin
      Refine_Pose_Iterative (World, Images, K, D, P, Refined);
      Assert (Refined, "iterative refinement returned false");
      declare
         After : constant Reprojection_Summary :=
           Summarize_Reprojection (Reprojection_Errors (World, Images, K, D, P));
      begin
         Report_Refinement (Name, Before, After, Center_Before, Center_Error (P));
         Assert (After.RMS_Error_Pixels < Before.RMS_Error_Pixels
                 and then After.Maximum_Error_Pixels < Before.Maximum_Error_Pixels,
                 "refinement did not improve reprojection");
         Assert (Center_Error (P) < Center_Before, "refined camera center not closer to truth");
         --  OpenCV 5's changed optimizer empirically finishes around 1.7e-6 px
         --  on macOS; retain a small cross-version bound, not bit identity.
         Assert (After.Maximum_Error_Pixels < 1.0E-5, "noiseless final pixel error");
      end;
   end Check_Refinement;

   procedure Refinement_Exact (T : in out Fixture) is
      pragma Unreferenced (T);
      Images : constant Image_Point_Array := Project_Points (World, K, No_Distortion, Known_Pose);
      P : World_To_Camera_Pose := Perturbed;
      Refined : Boolean;
   begin
      Check_Refinement (No_Distortion, "exact iterative");
      Refine_Pose_Iterative (World (1 .. 4), Images (1 .. 4), K, Pose => P, Refined => Refined);
      Assert (Refined and then Summarize_Reprojection
        (Reprojection_Errors (World (1 .. 4), Images (1 .. 4), K, No_Distortion, P)).
          Maximum_Error_Pixels < 1.0E-5, "conservative four-point refinement contract");
   end Refinement_Exact;

   procedure Refinement_Distorted (T : in out Fixture) is
      pragma Unreferenced (T);
   begin
      Check_Refinement (Distorted, "distorted iterative");
   end Refinement_Distorted;

   procedure Refinement_Inliers (T : in out Fixture) is
      pragma Unreferenced (T);
      Images : Image_Point_Array := Project_Points (World, K, No_Distortion, Known_Pose);
   begin
      for I in Images'Range loop
         if I = 2 or else I = 7 or else I = 15 then
            Images (I) (0) := Images (I) (0) + 5_000.0;
            Images (I) (1) := Images (I) (1) - 4_000.0;
         end if;
      end loop;
      declare
         Estimate : constant Pose_Estimate := Solve_PnP_RANSAC
           (World, Images, K, Options => (1_000, 2.0, 0.999));
      begin
         Assert (Found (Estimate), "RANSAC to refinement: no pose");
         declare
            Accepted : constant Inlier_Index_Array := Inliers (Estimate);
            Objects : Object_Point_Array (1 .. Accepted'Length);
            Pixels : Image_Point_Array (1 .. Accepted'Length);
            P : World_To_Camera_Pose := Pose (Estimate);
            Refined : Boolean;
         begin
            for I in Accepted'Range loop
               Assert (Accepted (I) /= 2 and then Accepted (I) /= 7 and then Accepted (I) /= 15,
                       "refinement subset contains gross outlier");
               Objects (I) := World (Accepted (I));
               Pixels (I) := Images (Accepted (I));
            end loop;
            declare
               Before : constant Reprojection_Summary := Summarize_Reprojection
                 (Reprojection_Errors (Objects, Pixels, K, No_Distortion, P));
               Center_Before : constant OpenCV.Float64_Value := Center_Error (P);
            begin
               Refine_Pose_Iterative (Objects, Pixels, K, No_Distortion, P, Refined);
               Assert (Refined, "RANSAC inlier refinement false");
               declare
                  After : constant Reprojection_Summary := Summarize_Reprojection
                    (Reprojection_Errors (Objects, Pixels, K, No_Distortion, P));
               begin
                  Report_Refinement ("RANSAC inliers", Before, After, Center_Before, Center_Error (P));
                  Assert (After.RMS_Error_Pixels <= Before.RMS_Error_Pixels + 1.0E-9,
                          "inlier refinement materially worsened RMS");
                  Assert (Center_Error (P) < 0.05, "refined inlier camera center differs");
               end;
            end;
         end;
      end;
   end Refinement_Inliers;

   procedure Refinement_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      --  Only this negative-test scope permits deliberate IEEE invalid values.
      pragma Suppress (Validity_Check);
      Images : constant Image_Point_Array := Project_Points (World, K, No_Distortion, Known_Pose);
      procedure Expect_Error (Objects : Object_Point_Array; Pixels : Image_Point_Array;
                              Intrinsics : Camera_Intrinsics := K;
                              D : Distortion_Coefficients := No_Distortion;
                              Initial : World_To_Camera_Pose := Perturbed) is
         P : World_To_Camera_Pose := Initial;
         Refined : Boolean := False;
      begin
         begin
            Refine_Pose_Iterative (Objects, Pixels, Intrinsics, D, P, Refined);
            Assert (False, "invalid refinement accepted");
         exception
            when OpenCV.OpenCV_Error => null;
         end;
         --  NaN is not value-equal to itself: verify it remains NaN and every
         --  finite component remains equal. Finite fault tests use record equality.
         if Initial.Rotation (0) = Initial.Rotation (0) then
            Assert (P = Initial and then not Refined, "failed refinement changed caller pose");
         else
            Assert (P.Rotation (0) /= P.Rotation (0) and then
                    P.Rotation (1) = Initial.Rotation (1) and then
                    P.Rotation (2) = Initial.Rotation (2) and then
                    P.Translation (0) = Initial.Translation (0) and then
                    P.Translation (1) = Initial.Translation (1) and then
                    P.Translation (2) = Initial.Translation (2) and then not Refined,
                    "failed refinement changed nonfinite initial pose");
         end if;
      end Expect_Error;
   begin
      Expect_Error (World (1 .. 3), Images (1 .. 3));
      Expect_Error (World, Images (1 .. 19));
      Expect_Error (World, Images, (0.0, 820.0, 320.0, 240.0));
      for Kind in Interfaces.Integer_32 range 0 .. 2 loop
         declare
            Invalid : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Nonfinite (Kind));
            Objects : Object_Point_Array := World;
            Pixels : Image_Point_Array := Images;
            Initial : World_To_Camera_Pose := Perturbed;
         begin
            Objects (1) (0) := Invalid;
            Pixels (1) (1) := Invalid;
            Initial.Rotation (0) := Invalid;
            Expect_Error (Objects, Images);
            Expect_Error (World, Pixels);
            Expect_Error (World, Images, D => (K3 => Invalid, others => 0.0));
            Expect_Error (World, Images, Initial => Initial);
         end;
      end loop;
   end Refinement_Invalid;

   Ray_K : constant Camera_Intrinsics := (100.0, 200.0, 10.0, 20.0);
   Quarter_Pose : constant World_To_Camera_Pose :=
     (Rotation => [0.0, 0.0, Ada.Numerics.Pi / 2.0], Translation => [1.0, 2.0, 3.0]);
   Ray_D : constant Distortion_Coefficients := (0.1, -0.04, 0.003, -0.002, 0.01);

   --  Independent forward Brown fixture, never calls Project_Points or
   --  Undistort_To_Normalized to manufacture expected normalized values.
   function Brown_Pixel (X, Y : OpenCV.Float64_Value) return Image_Point is
      R2 : constant OpenCV.Float64_Value := X * X + Y * Y;
      Radial : constant OpenCV.Float64_Value :=
        1.0 + Ray_D.K1 * R2 + Ray_D.K2 * R2 * R2 + Ray_D.K3 * R2 * R2 * R2;
   begin
      return [Ray_K.Focal_X *
        (X * Radial + 2.0 * Ray_D.P1 * X * Y + Ray_D.P2 * (R2 + 2.0 * X * X)) + Ray_K.Center_X,
        Ray_K.Focal_Y *
        (Y * Radial + Ray_D.P1 * (R2 + 2.0 * Y * Y) + 2.0 * Ray_D.P2 * X * Y) + Ray_K.Center_Y];
   end Brown_Pixel;

   procedure Check_Vector
     (Actual, Expected : Object_Point; Epsilon : OpenCV.Float64_Value := 1.0E-12) is
   begin
      for Axis in 0 .. 2 loop
         Assert (Near (Actual (Axis), Expected (Axis), Epsilon), "vector component oracle");
      end loop;
   end Check_Vector;

   procedure Normalized_Zero (T : in out Fixture) is
      pragma Unreferenced (T);
      Pixels : constant Image_Point_Array (4 .. 6) := [[10.0, 20.0], [110.0, 20.0], [-40.0, 70.0]];
      Values : constant Normalized_Image_Point_Array := Undistort_To_Normalized (Pixels, Ray_K);
   begin
      Assert (Values'First = 1 and then Values'Last = 3, "normalized count/bounds");
      for I in Values'Range loop
         Assert (Near (Values (I) (0), (Pixels (I + 3) (0) - Ray_K.Center_X) / Ray_K.Focal_X, 1.0E-14)
                 and then Near (Values (I) (1), (Pixels (I + 3) (1) - Ray_K.Center_Y) / Ray_K.Focal_Y, 1.0E-14),
                 "independent zero-distortion normalized oracle/order");
      end loop;
      Ada.Text_IO.Put_Line ("normalized zero oracle: principal=(0,0), off-axis=(1,0), negative=(-0.5,0.25)");
   end Normalized_Zero;

   procedure Normalized_Brown (T : in out Fixture) is
      pragma Unreferenced (T);
      Known : constant Image_Point_Array := [[0.3, -0.2], [-0.4, 0.25], [0.0, 0.0], [1.0, 0.0]];
      Pixels : Image_Point_Array (Known'Range);
      Maximum : OpenCV.Float64_Value := 0.0;
   begin
      for I in Known'Range loop
         Pixels (I) := Brown_Pixel (Known (I) (0), Known (I) (1));
      end loop;
      declare
         Values : constant Normalized_Image_Point_Array :=
           Undistort_To_Normalized (Pixels, Ray_K, Ray_D);
      begin
         for I in Known'Range loop
            for Axis in 0 .. 1 loop
               Maximum := OpenCV.Float64_Value'Max (Maximum, abs (Values (I) (Axis) - Known (I) (Axis)));
               Assert (Near (Values (I) (Axis), Known (I) (Axis), 1.0E-10), "independent all-five Brown inversion");
            end loop;
         end loop;
      end;
      Ada.Text_IO.Put_Line ("five-coefficient inversion max component error=" & OpenCV.Float64_Value'Image (Maximum));
   end Normalized_Brown;

   procedure Rotation_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      Identity : constant Rotation_Matrix := Rotation_Matrix_Of ((others => <>));
      Quarter : constant Rotation_Matrix := Rotation_Matrix_Of (Quarter_Pose);
      Expected : constant Rotation_Matrix := [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
      General : constant Rotation_Matrix := Rotation_Matrix_Of (Known_Pose);
      type Matrix_List is array (Positive range <>) of Rotation_Matrix;
   begin
      for Row in 0 .. 2 loop
         for Col in 0 .. 2 loop
            Assert (Near (Identity (Row, Col), (if Row = Col then 1.0 else 0.0), 1.0E-14), "identity Rodrigues");
            Assert (Near (Quarter (Row, Col), Expected (Row, Col), 1.0E-14), "Rz(pi/2) Rodrigues oracle");
         end loop;
      end loop;
      for Matrix of Matrix_List'[Identity, Quarter, General] loop
         for Row in 0 .. 2 loop
            for Col in 0 .. 2 loop
               declare
                  Rows, Columns : OpenCV.Float64_Value := 0.0;
               begin
                  for Axis in 0 .. 2 loop
                     Rows := Rows + Matrix (Row, Axis) * Matrix (Col, Axis);
                     Columns := Columns + Matrix (Axis, Row) * Matrix (Axis, Col);
                  end loop;
                  Assert (Near (Rows, (if Row = Col then 1.0 else 0.0), 1.0E-12), "row orthonormality");
                  Assert (Near (Columns, (if Row = Col then 1.0 else 0.0), 1.0E-12), "R^T R / column orthonormality");
               end;
            end loop;
         end loop;
         Assert (Near (
           Matrix (0, 0) * (Matrix (1, 1) * Matrix (2, 2) - Matrix (1, 2) * Matrix (2, 1))
           - Matrix (0, 1) * (Matrix (1, 0) * Matrix (2, 2) - Matrix (1, 2) * Matrix (2, 0))
           + Matrix (0, 2) * (Matrix (1, 0) * Matrix (2, 1) - Matrix (1, 1) * Matrix (2, 0)), 1.0, 1.0E-12),
           "det(R) = +1");
      end loop;
      Ada.Text_IO.Put_Line ("Rodrigues oracle: identity and Rz(pi/2); rows/columns/R^T R/det qualified");
   end Rotation_Oracle;

   procedure Point_Transforms (T : in out Fixture) is
      pragma Unreferenced (T);
      Points : constant Object_Point_Array := [[0.0, 0.0, 0.0], [1.2, -3.4, 5.6], [-9.0, -2.0, -1.0],
                                              [1.0E6, -2.0E6, 3.0E6]];
      Maximum : OpenCV.Float64_Value := 0.0;
   begin
      Check_Vector (World_To_Camera_Point (Quarter_Pose, [1.0, 0.0, 0.0]), [1.0, 3.0, 3.0]);
      Check_Vector (Camera_To_World_Point (Quarter_Pose, [1.0, 3.0, 3.0]), [1.0, 0.0, 0.0]);
      Check_Vector (World_To_Camera_Point (Quarter_Pose, Camera_Center (Quarter_Pose)), [others => 0.0]);
      for Point of Points loop
         declare
            Roundtrip : constant Object_Point := Camera_To_World_Point
              (Known_Pose, World_To_Camera_Point (Known_Pose, Point));
         begin
            Check_Vector (Roundtrip, Point, 1.0E-8);
            for Axis in 0 .. 2 loop
               Maximum := OpenCV.Float64_Value'Max (Maximum, abs (Roundtrip (Axis) - Point (Axis)));
            end loop;
         end;
      end loop;
      Ada.Text_IO.Put_Line ("point oracle: (1,0,0)->(1,3,3); camera center->zero; roundtrip max=" &
        OpenCV.Float64_Value'Image (Maximum));
   end Point_Transforms;

   procedure Direction_Transforms (T : in out Fixture) is
      pragma Unreferenced (T);
      Changed : World_To_Camera_Pose := Quarter_Pose;
      Directions : constant Object_Point_Array := [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [1.2, -3.4, 5.6],
                                                  [-9.0, -2.0, -1.0], [1.0E6, -2.0E6, 3.0E6]];
   begin
      Changed.Translation := [-9.0, 8.0, 7.0];
      Check_Vector (Object_Point (World_To_Camera_Direction (Quarter_Pose, [1.0, 0.0, 0.0])), [0.0, 1.0, 0.0]);
      Check_Vector (Object_Point (Camera_To_World_Direction (Quarter_Pose, [1.0, 0.0, 0.0])), [0.0, -1.0, 0.0]);
      for Direction of Directions loop
         Check_Vector (Object_Point (Camera_To_World_Direction
           (Known_Pose, World_To_Camera_Direction (Known_Pose, World_Direction (Direction)))),
           Direction, 1.0E-8);
         declare
            Camera : constant Camera_Direction := World_To_Camera_Direction (Quarter_Pose, World_Direction (Direction));
            World : constant World_Direction := Camera_To_World_Direction (Quarter_Pose, Camera);
         begin
            Check_Vector (Object_Point (World), Direction, 1.0E-8);
            Check_Vector (Object_Point (World_To_Camera_Direction (Changed, World_Direction (Direction))), Object_Point (Camera));
            Check_Vector (Object_Point (Camera_To_World_Direction (Changed, Camera)), Object_Point (World));
         end;
      end loop;
      Check_Vector (Object_Point (World_To_Camera_Direction (Quarter_Pose, [2.0, 0.0, 0.0])), [0.0, 2.0, 0.0]);
      Ada.Text_IO.Put_Line ("direction oracle: world X->camera Y; camera X->world -Y; magnitude/translation invariance");
   end Direction_Transforms;

   procedure Camera_Bearings (T : in out Fixture) is
      pragma Unreferenced (T);
      Values : constant Camera_Direction_Array := Camera_Bearing_Rays
        ([[10.0, 20.0], [110.0, 20.0], [-40.0, 70.0]], Ray_K);
      Inv_Sqrt2 : constant OpenCV.Float64_Value := 1.0 / Math.Sqrt (2.0);
   begin
      Check_Vector (Object_Point (Values (1)), [0.0, 0.0, 1.0], 1.0E-14);
      Check_Vector (Object_Point (Values (2)), [Inv_Sqrt2, 0.0, Inv_Sqrt2], 1.0E-14);
      for Value of Values loop
         Assert (Value (2) > 0.0, "camera Z must be positive");
         Assert (Near (Value (0)**2 + Value (1)**2 + Value (2)**2, 1.0, 1.0E-12), "camera bearing unit norm");
      end loop;
      Ada.Text_IO.Put_Line ("camera bearing oracle: principal=(0,0,1); off-axis=(1,0,1)/sqrt(2)");
   end Camera_Bearings;

   procedure World_Bearings (T : in out Fixture) is
      pragma Unreferenced (T);
      Values : constant World_Ray_Array := World_Bearing_Rays
        ([[110.0, 20.0], [10.0, 20.0]], Ray_K, No_Distortion, Quarter_Pose);
      Inv_Sqrt2 : constant OpenCV.Float64_Value := 1.0 / Math.Sqrt (2.0);
   begin
      for Ray of Values loop
         Check_Vector (Ray.Origin, [-2.0, 1.0, -3.0]);
         Check_Vector (Ray.Origin, Camera_Center (Quarter_Pose));
         Assert (Near (Ray.Direction (0)**2 + Ray.Direction (1)**2 + Ray.Direction (2)**2, 1.0, 1.0E-12),
                 "world bearing unit norm");
      end loop;
      Check_Vector (Object_Point (Values (1).Direction), [0.0, -Inv_Sqrt2, Inv_Sqrt2]);
      Check_Vector (Object_Point (Values (2).Direction), [0.0, 0.0, 1.0]);
      Ada.Text_IO.Put_Line ("world ray oracle: origin=(-2,1,-3), off-axis direction=(0,-1,1)/sqrt(2)");
   end World_Bearings;

   procedure Distorted_World_Bearings (T : in out Fixture) is
      pragma Unreferenced (T);
      X : constant OpenCV.Float64_Value := 0.3;
      Y : constant OpenCV.Float64_Value := -0.2;
      Norm : constant OpenCV.Float64_Value := Math.Sqrt (X * X + Y * Y + 1.0);
      Values : constant World_Ray_Array := World_Bearing_Rays
        ([Brown_Pixel (X, Y)], Ray_K, Ray_D, Quarter_Pose);
   begin
      Check_Vector (Values (1).Origin, [-2.0, 1.0, -3.0]);
      Check_Vector (Object_Point (Values (1).Direction), [Y / Norm, -X / Norm, 1.0 / Norm], 1.0E-10);
      Ada.Text_IO.Put_Line ("distorted world ray: direction=(" &
        OpenCV.Float64_Value'Image (Values (1).Direction (0)) & "," &
        OpenCV.Float64_Value'Image (Values (1).Direction (1)) & "," &
        OpenCV.Float64_Value'Image (Values (1).Direction (2)) & "); analytic R^T Brown oracle");
   end Distorted_World_Bearings;

   procedure Rotation_Layout (T : in out Fixture) is
      pragma Unreferenced (T);
      function Layout (Field : Interfaces.Integer_32) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_rotation_layout";
      procedure Fill (Value : access ABI.C_Rotation_Matrix)
        with Import, Convention => C, External_Name => "calib3d_test_fill_rotation";
      Value : aliased ABI.C_Rotation_Matrix;
      type Positions is array (Natural range <>) of Natural;
      Offsets : constant Positions := [Value.M00'Position, Value.M01'Position, Value.M02'Position,
        Value.M10'Position, Value.M11'Position, Value.M12'Position,
        Value.M20'Position, Value.M21'Position, Value.M22'Position];
   begin
      Assert (ABI.C_Rotation_Matrix'Size = Natural (Layout (0)) * System.Storage_Unit, "rotation C/Ada size");
      Assert (ABI.C_Rotation_Matrix'Alignment = Natural (Layout (1)), "rotation C/Ada alignment");
      for I in Offsets'Range loop
         Assert (Offsets (I) = Natural (Layout (Interfaces.Integer_32 (I + 2))), "rotation field offset");
      end loop;
      Fill (Value'Access);
      Assert (Value.M00 = 1.0 and then Value.M01 = 2.0 and then Value.M02 = 3.0 and then
              Value.M10 = 4.0 and then Value.M11 = 5.0 and then Value.M12 = 6.0 and then
              Value.M20 = 7.0 and then Value.M21 = 8.0 and then Value.M22 = 9.0, "C-written all-fields rotation interchange");
      Ada.Text_IO.Put_Line ("rotation layout: compiler size=" & Interfaces.Integer_32'Image (Layout (0)) &
        " alignment=" & Interfaces.Integer_32'Image (Layout (1)) & "; all nine offsets/interchange passed");
   end Rotation_Layout;

   procedure Ray_Empty (T : in out Fixture) is
      pragma Unreferenced (T);
      N : constant Normalized_Image_Point_Array := Undistort_To_Normalized ([1 .. 0 => <>], Ray_K);
      C : constant Camera_Direction_Array := Camera_Bearing_Rays ([1 .. 0 => <>], Ray_K);
      W : constant World_Ray_Array := World_Bearing_Rays ([1 .. 0 => <>], Ray_K, No_Distortion, Quarter_Pose);
   begin
      Assert (N'First = 1 and then N'Last = 0 and then C'First = 1 and then C'Last = 0
              and then W'First = 1 and then W'Last = 0, "empty ray bounds");
   end Ray_Empty;

   procedure Ray_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check);
      procedure Expect_Error
        (Pixels : Image_Point_Array; Intrinsics : Camera_Intrinsics := Ray_K;
         D : Distortion_Coefficients := No_Distortion; P : World_To_Camera_Pose := Quarter_Pose;
         Operation : Positive := 1)
      is
      begin
         case Operation is
            when 1 =>
               declare
                  V : constant Normalized_Image_Point_Array := Undistort_To_Normalized (Pixels, Intrinsics, D);
                  pragma Unreferenced (V);
               begin null; end;
            when 2 =>
               declare
                  V : constant Camera_Direction_Array := Camera_Bearing_Rays (Pixels, Intrinsics, D);
                  pragma Unreferenced (V);
               begin null; end;
            when others =>
               declare
                  V : constant World_Ray_Array := World_Bearing_Rays (Pixels, Intrinsics, D, P);
                  pragma Unreferenced (V);
               begin null; end;
         end case;
         Assert (False, "invalid camera ray input accepted");
      exception
         when OpenCV.OpenCV_Error => null;
      end Expect_Error;
   begin
      for Operation in 1 .. 3 loop
         Expect_Error ([1 .. 0 => <>], Intrinsics => (0.0, 200.0, 10.0, 20.0), Operation => Operation);
         for Kind in Interfaces.Integer_32 range 0 .. 2 loop
            declare
               Invalid : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Nonfinite (Kind));
            begin
               Expect_Error ([[Invalid, 20.0]], Operation => Operation);
               Expect_Error ([1 .. 0 => <>], D => (P1 => Invalid, others => 0.0), Operation => Operation);
               Expect_Error ([[10.0, 20.0]], Intrinsics => (100.0, 200.0, Invalid, 20.0), Operation => Operation);
               if Operation = 3 then
                  Expect_Error ([1 .. 0 => <>], P => (Rotation => [Invalid, 0.0, 0.0], others => <>), Operation => Operation);
               end if;
            end;
         end loop;
      end loop;
      Expect_Error ([[OpenCV.Float64_Value'Last, 20.0]],
                    Intrinsics => (1.0E-300, 200.0, 10.0, 20.0));
   end Ray_Invalid;

   procedure Transform_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check);
      procedure Expect_Error (P : World_To_Camera_Pose; V : Object_Point; Operation : Positive) is
      begin
         case Operation is
            when 1 =>
               declare M : constant Rotation_Matrix := Rotation_Matrix_Of (P);
                  pragma Unreferenced (M);
               begin null; end;
            when 2 =>
               declare X : constant Object_Point := World_To_Camera_Point (P, V);
                  pragma Unreferenced (X);
               begin null; end;
            when 3 =>
               declare X : constant Object_Point := Camera_To_World_Point (P, V);
                  pragma Unreferenced (X);
               begin null; end;
            when 4 =>
               declare X : constant Camera_Direction := World_To_Camera_Direction (P, World_Direction (V));
                  pragma Unreferenced (X);
               begin null; end;
            when others =>
               declare X : constant World_Direction := Camera_To_World_Direction (P, Camera_Direction (V));
                  pragma Unreferenced (X);
               begin null; end;
         end case;
         Assert (False, "invalid transform accepted");
      exception
         when OpenCV.OpenCV_Error => null;
      end Expect_Error;
   begin
      for Operation in 1 .. 5 loop
         for Kind in Interfaces.Integer_32 range 0 .. 2 loop
            declare
               Invalid : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Nonfinite (Kind));
               P : World_To_Camera_Pose := Quarter_Pose;
            begin
               P.Rotation (1) := Invalid;
               Expect_Error (P, [others => 0.0], Operation);
               P := Quarter_Pose;
               P.Translation (2) := Invalid;
               Expect_Error (P, [others => 0.0], Operation);
               if Operation /= 1 then
                  Expect_Error (Quarter_Pose, [0.0, Invalid, 0.0], Operation);
               end if;
            end;
         end loop;
      end loop;
      Expect_Error ((Rotation => [OpenCV.Float64_Value'Last, 0.0, 0.0], others => <>), [others => 0.0], 1);
      Expect_Error ((Rotation => [others => 0.0], Translation => [OpenCV.Float64_Value'Last, 0.0, 0.0]),
                    [OpenCV.Float64_Value'Last, 0.0, 0.0], 2);
      Expect_Error ((Rotation => [others => 0.0], Translation => [-OpenCV.Float64_Value'Last, 0.0, 0.0]),
                    [OpenCV.Float64_Value'Last, 0.0, 0.0], 3);
   end Transform_Invalid;

   procedure Bearing_Range (T : in out Fixture) is
      pragma Unreferenced (T);
      --  Squaring these finite normalized components naively would overflow.
      Values : constant Camera_Direction_Array := Camera_Bearing_Rays
        ([[1.0E300, -1.0E300]], (1.0, 1.0, 0.0, 0.0));
   begin
      Assert (Near (Values (1) (0)**2 + Values (1) (1)**2 + Values (1) (2)**2, 1.0, 1.0E-12)
              and then Values (1) (2) > 0.0, "scaled bearing norm / positive Z");
   end Bearing_Range;

   H_Policy : constant Homography_RANSAC_Options := (2_000, 0.1, 0.999);
   function H_Source return Image_Point_Array is
   begin
      return Values : Image_Point_Array (1 .. 24) do
         for I in Values'Range loop
            Values (I) := [OpenCV.Float64_Value ((I - 1) mod 6) * 40.0 - 100.0,
                           OpenCV.Float64_Value ((I - 1) / 6) * 35.0 - 50.0];
         end loop;
      end return;
   end H_Source;
   --  Independent fixture generation: neither new mapping API nor OpenCV.
   function H_Destination (Source : Image_Point_Array) return Image_Point_Array is
   begin
      return Values : Image_Point_Array (Source'Range) do
         for I in Source'Range loop
            declare
               X : constant OpenCV.Float64_Value := Source (I) (0);
               Y : constant OpenCV.Float64_Value := Source (I) (1);
               W : constant OpenCV.Float64_Value := 0.001 * X - 0.002 * Y + 1.0;
            begin
               Values (I) := [(1.2 * X + 0.1 * Y + 20.0) / W,
                              (-0.05 * X + 0.9 * Y + 10.0) / W];
            end;
         end loop;
      end return;
   end H_Destination;

   procedure Homography_Map (T : in out Fixture) is
      pragma Unreferenced (T);
      H : constant Homography_Matrix := [[2.0, 0.5, 10.0], [0.0, 3.0, -5.0], [0.01, 0.02, 1.0]];
      Scaled : Homography_Matrix := H;
      Points : constant Image_Point_Array := [[0.0, 0.0], [10.0, 20.0], [-20.0, 10.0]];
      Expected : constant Image_Point_Array := [[10.0, -5.0], [40.0 / 1.5, 55.0 / 1.5], [-25.0, 25.0]];
   begin
      for V of Scaled loop
         V := -2.0 * V;
      end loop;
      for I in Points'Range loop
         declare
            M : constant Homography_Point_Result := Map_With_Homography (H, Points (I));
            S : constant Homography_Point_Result := Map_With_Homography (Scaled, Points (I));
         begin
            Assert (M.Finite and then S.Finite, "hand-authored mappings finite");
            for J in 0 .. 1 loop
               Assert (Near (M.Point (J), Expected (I) (J)) and then Near (S.Point (J), Expected (I) (J)),
                       "independent projective/scale-invariance oracle");
            end loop;
         end;
      end loop;
      Assert (not Map_With_Homography (H, [-100.0, 0.0]).Finite, "w=0 point at infinity");
      Ada.Text_IO.Put_Line ("homography mapping: hand-authored oracle, negative common scale, w=0 PASS");
   end Homography_Map;

   procedure Homography_Map_Range (T : in out Fixture) is
      pragma Unreferenced (T);
      --  Deliberate IEEE nonfinite fixtures, as in existing negative tests.
      pragma Suppress (Validity_Check);
      H : Homography_Matrix := [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0E-200]];
      Tiny : constant Homography_Point_Result := Map_With_Homography (H, [1.0E-200, 2.0E-200]);
   begin
      Assert (Tiny.Finite and then Near (Tiny.Point (0), 1.0) and then Near (Tiny.Point (1), 2.0),
              "small nonzero denominator not rejected");
      Assert (not Map_With_Homography (H, [1.0E200, 1.0E200]).Finite, "division overflow not finite");
      H (2, 2) := 1.0;
      H (0, 0) := 2.0;
      Assert (not Map_With_Homography (H, [OpenCV.Float64_Value'Last, 0.0]).Finite, "product overflow not finite");
      for Kind in Interfaces.Integer_32 range 0 .. 2 loop
         for Row in 0 .. 2 loop
            for Col in 0 .. 2 loop
               declare
                  Bad : Homography_Matrix := H;
               begin
                  Bad (Row, Col) := OpenCV.Float64_Value (Nonfinite (Kind));
                  begin
                     declare
                        M : constant Homography_Point_Result := Map_With_Homography (Bad, [1.0, 2.0]);
                        pragma Unreferenced (M);
                     begin
                        Assert (False, "nonfinite H accepted");
                     end;
                  exception
                     when OpenCV.OpenCV_Error => null;
                  end;
               end;
            end loop;
         end loop;
         for J in 0 .. 1 loop
            declare
               P : Image_Point := [1.0, 2.0];
            begin
               P (J) := OpenCV.Float64_Value (Nonfinite (Kind));
               begin
                  declare
                     M : constant Homography_Point_Result := Map_With_Homography (H, P);
                     pragma Unreferenced (M);
                  begin
                     Assert (False, "nonfinite map input accepted");
                  end;
               exception
                  when OpenCV.OpenCV_Error => null;
               end;
            end;
         end loop;
      end loop;
   end Homography_Map_Range;

   procedure Check_Homography_Final
     (Estimate : Homography_Estimate; Source, Destination : Image_Point_Array;
      Maximum_Error : out OpenCV.Float64_Value)
   is
      Indices : constant Inlier_Index_Array := Inliers (Estimate);
      H : constant Homography_Matrix := Homography (Estimate);
   begin
      Maximum_Error := 0.0;
      Assert (Found (Estimate) and then Inlier_Count (Estimate) >= 4, "usable homography support");
      for I in Indices'Range loop
         Assert (Indices (I) in 1 .. Source'Length and then Inlier (Estimate, I) = Indices (I), "valid correspondence index");
         if I > 1 then
            Assert (Indices (I) > Indices (I - 1), "unique ascending indices");
         end if;
      end loop;
      for I in 1 .. Source'Length loop
         declare
            P : constant Image_Point := Source (Source'First + I - 1);
            D : constant Image_Point := Destination (Destination'First + I - 1);
            W : constant OpenCV.Float64_Value := H (2, 0) * P (0) + H (2, 1) * P (1) + H (2, 2);
            --  Independent forward formula, not Map_With_Homography.
            X : constant OpenCV.Float64_Value := (H (0, 0) * P (0) + H (0, 1) * P (1) + H (0, 2)) / W;
            Y : constant OpenCV.Float64_Value := (H (1, 0) * P (0) + H (1, 1) * P (1) + H (1, 2)) / W;
            Error : constant OpenCV.Float64_Value := Math.Sqrt ((X - D (0))**2 + (Y - D (1))**2);
         begin
            Assert (Contains (Indices, I) = (Error <= H_Policy.Reprojection_Threshold_Pixels),
                    "every included/excluded point matches final-model threshold");
            Maximum_Error := OpenCV.Float64_Value'Max (Maximum_Error, Error);
         end;
      end loop;
      begin
         declare
            Index : constant Positive := Inlier (Estimate, Inlier_Count (Estimate) + 1);
            pragma Unreferenced (Index);
         begin
            Assert (False, "invalid public homography index accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
   end Check_Homography_Final;

   procedure Homography_Clean (T : in out Fixture) is
      pragma Unreferenced (T);
      --  Non-one array bounds prove public indices are correspondence positions.
      Source : constant Image_Point_Array (11 .. 34) := H_Source;
      Destination : constant Image_Point_Array (51 .. 74) := H_Destination (Source);
      Estimate : constant Homography_Estimate := Estimate_Homography_RANSAC (Source, Destination, H_Policy);
      Error : OpenCV.Float64_Value;
   begin
      Check_Homography_Final (Estimate, Source, Destination, Error);
      Assert (Inlier_Count (Estimate) = 24 and then Error < 1.0E-3, "clean known projective mapping");
      Ada.Text_IO.Put_Line ("clean homography: 24/24 final inliers; max mapping error=" & OpenCV.Float64_Value'Image (Error));
   end Homography_Clean;

   procedure Homography_Outliers (T : in out Fixture) is
      pragma Unreferenced (T);
      Source : constant Image_Point_Array := H_Source;
      Destination : Image_Point_Array := H_Destination (Source);
      Error : OpenCV.Float64_Value;
   begin
      for I in Destination'Range loop
         if I = 2 or else I = 7 or else I = 15 then
            Destination (I) := [Destination (I) (0) + 5_000.0, Destination (I) (1) - 4_000.0];
         end if;
      end loop;
      declare
         Estimate : constant Homography_Estimate := Estimate_Homography_RANSAC (Source, Destination, H_Policy);
      begin
         Check_Homography_Final (Estimate, Source, Destination, Error);
         Assert (not Contains (Inliers (Estimate), 2) and then not Contains (Inliers (Estimate), 7) and then
                 not Contains (Inliers (Estimate), 15), "gross outlier rejection");
         Ada.Text_IO.Put_Line ("outlier homography: final inliers=" & Natural'Image (Inlier_Count (Estimate)) &
           "; 2/7/15 rejected; every final-model inclusion/exclusion threshold checked");
      end;
   end Homography_Outliers;

   procedure Homography_No_Model (T : in out Fixture) is
      pragma Unreferenced (T);
      Source, Destination : Image_Point_Array (1 .. 24);
   begin
      for I in Source'Range loop
         Source (I) := [OpenCV.Float64_Value (I), 2.0 * OpenCV.Float64_Value (I)];
         Destination (I) := [3.0 * OpenCV.Float64_Value (I), 4.0 * OpenCV.Float64_Value (I)];
      end loop;
      declare
         Estimate : constant Homography_Estimate := Estimate_Homography_RANSAC (Source, Destination, H_Policy);
         Indices : constant Inlier_Index_Array := Inliers (Estimate);
      begin
         Assert (not Found (Estimate) and then Inlier_Count (Estimate) = 0 and then
                 Indices'First = 1 and then Indices'Last = 0, "collinear no-model contract");
         begin
            declare
               H : constant Homography_Matrix := Homography (Estimate);
               pragma Unreferenced (H);
            begin
               Assert (False, "no-model homography accessible");
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
         begin
            declare
               I : constant Positive := Inlier (Estimate, 1);
               pragma Unreferenced (I);
            begin
               Assert (False, "no-model inlier accessible");
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end;
      Ada.Text_IO.Put_Line ("homography collinear: success status, Found=False, empty inliers, inaccessible H");
   end Homography_No_Model;

   procedure Reject_Homography
     (Source, Destination : Image_Point_Array;
      Options : Homography_RANSAC_Options := H_Policy) is
   begin
      declare
         Estimate : constant Homography_Estimate := Estimate_Homography_RANSAC (Source, Destination, Options);
         pragma Unreferenced (Estimate);
      begin
         Assert (False, "invalid homography accepted");
      end;
   exception
      when OpenCV.OpenCV_Error => null;
   end Reject_Homography;

   procedure Homography_Counts (T : in out Fixture) is
      pragma Unreferenced (T);
      Source : constant Image_Point_Array := H_Source;
      Destination : constant Image_Point_Array := H_Destination (Source);
      Five : constant Image_Point_Array := [Source (1), Source (6), Source (19), Source (24), Source (11)];
      Five_D : constant Image_Point_Array := H_Destination (Five);
      Estimate : constant Homography_Estimate := Estimate_Homography_RANSAC (Five, Five_D, H_Policy);
   begin
      for N in 0 .. 4 loop
         Reject_Homography (Source (1 .. N), Destination (1 .. N));
      end loop;
      Reject_Homography (Source, Destination (1 .. 23));
      Reject_Homography (Source (1 .. 23), Destination);
      Assert (Found (Estimate), "five points may attempt robust path");
      Ada.Text_IO.Put_Line ("homography counts: 0..4 rejected; 5 nondegenerate points robust PASS");
   end Homography_Counts;

   procedure Homography_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check);
      Source : Image_Point_Array := H_Source;
      Destination : Image_Point_Array := H_Destination (Source);
      Bad : Homography_RANSAC_Options := H_Policy;
      function Wider_Than_Native (Value : Interfaces.Unsigned_64) return Boolean is
         use type Interfaces.Unsigned_64;
      begin
         return Value > Interfaces.Unsigned_64 (Interfaces.Integer_32'Last);
      end Wider_Than_Native;
      function Above_Native_Limit (Native_Limit : Interfaces.Unsigned_64) return Positive is
         use type Interfaces.Unsigned_64;
      begin
         return Positive (Native_Limit + 1);
      end Above_Native_Limit;
   begin
      for Kind in Interfaces.Integer_32 range 0 .. 2 loop
         for J in 0 .. 1 loop
            declare
               S : constant Image_Point := Source (1);
               D : constant Image_Point := Destination (1);
            begin
               Source (1) (J) := OpenCV.Float64_Value (Nonfinite (Kind));
               Reject_Homography (Source, Destination);
               Source (1) := S;
               Destination (1) (J) := OpenCV.Float64_Value (Nonfinite (Kind));
               Reject_Homography (Source, Destination);
               Destination (1) := D;
            end;
         end loop;
         Bad := H_Policy; Bad.Confidence := OpenCV.Float64_Value (Nonfinite (Kind));
         Reject_Homography (Source, Destination, Bad);
         Bad := H_Policy; Bad.Reprojection_Threshold_Pixels := OpenCV.Float64_Value (Nonfinite (Kind));
         Reject_Homography (Source, Destination, Bad);
      end loop;
      for Value of Reprojection_Error_Array'[0.0, -1.0, 1.0E-300, OpenCV.Float64_Value'Last] loop
         Bad := H_Policy; Bad.Reprojection_Threshold_Pixels := Value;
         Reject_Homography (Source, Destination, Bad);
      end loop;
      for Value of Reprojection_Error_Array'[-1.0, 0.0, 1.0, 2.0] loop
         Bad := H_Policy; Bad.Confidence := Value;
         Reject_Homography (Source, Destination, Bad);
      end loop;
      --  Positive excludes zero/negative iterations at Ada assignment. On
      --  wider targets separately exercise the native INT32 bound.
      if Wider_Than_Native (Interfaces.Unsigned_64 (Positive'Last)) then
         Bad := H_Policy;
         Bad.Maximum_Iterations := Above_Native_Limit (Interfaces.Unsigned_64 (Interfaces.Integer_32'Last));
         Reject_Homography (Source, Destination, Bad);
      end if;
   end Homography_Invalid;

   procedure Homography_Layout (T : in out Fixture) is
      pragma Unreferenced (T);
      function Layout (Field : Interfaces.Integer_32) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_homography_layout";
      procedure Fill (Value : access ABI.C_Homography)
        with Import, Convention => C, External_Name => "calib3d_test_fill_homography";
      function Options_Layout (Field : Interfaces.Integer_32) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_homography_options_layout";
      procedure Fill_Options (Value : access ABI.C_Homography_Options)
        with Import, Convention => C, External_Name => "calib3d_test_fill_homography_options";
      Value : aliased ABI.C_Homography;
      Options : aliased ABI.C_Homography_Options;
      type Positions is array (Natural range <>) of Natural;
      Offsets : constant Positions := [Value.H00'Position, Value.H01'Position, Value.H02'Position,
        Value.H10'Position, Value.H11'Position, Value.H12'Position,
        Value.H20'Position, Value.H21'Position, Value.H22'Position];
      Option_Offsets : constant Positions := [Options.Maximum_Iterations'Position,
        Options.Reprojection_Threshold_Pixels'Position, Options.Confidence'Position];
   begin
      Assert (ABI.C_Homography'Size = Natural (Layout (0)) * System.Storage_Unit, "homography C/Ada size");
      Assert (ABI.C_Homography'Alignment = Natural (Layout (1)), "homography C/Ada alignment");
      for I in Offsets'Range loop
         Assert (Offsets (I) = Natural (Layout (Interfaces.Integer_32 (I + 2))), "homography field offset");
      end loop;
      Fill (Value'Access);
      Assert (Value.H00 = 1.0 and then Value.H01 = 2.0 and then Value.H02 = 3.0 and then
              Value.H10 = 4.0 and then Value.H11 = 5.0 and then Value.H12 = 6.0 and then
              Value.H20 = 7.0 and then Value.H21 = 8.0 and then Value.H22 = 9.0, "C-written homography interchange");
      Assert (ABI.C_Homography_Options'Size = Natural (Options_Layout (0)) * System.Storage_Unit and then
              ABI.C_Homography_Options'Alignment = Natural (Options_Layout (1)), "homography options C/Ada layout");
      for I in Option_Offsets'Range loop
         Assert (Option_Offsets (I) = Natural (Options_Layout (Interfaces.Integer_32 (I + 2))), "options field offset");
      end loop;
      Fill_Options (Options'Access);
      Assert (Options.Maximum_Iterations = 2000 and then Options.Reprojection_Threshold_Pixels = 3.0 and then
              Options.Confidence = 0.995, "C-written homography options interchange");
      Ada.Text_IO.Put_Line ("homography layout: compiler size=" & Interfaces.Integer_32'Image (Layout (0)) &
        " alignment=" & Interfaces.Integer_32'Image (Layout (1)) & "; all nine offsets/interchange; options layout PASS");
   end Homography_Layout;

   procedure Fundamental_Layout (T : in out Fixture) is
      pragma Unreferenced (T);
      function Layout (Field : Interfaces.Integer_32) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_fundamental_layout";
      procedure Fill (Value : access ABI.C_Fundamental)
        with Import, Convention => C, External_Name => "calib3d_test_fill_fundamental";
      function Options_Layout (Field : Interfaces.Integer_32) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_fundamental_options_layout";
      procedure Fill_Options (Value : access ABI.C_Fundamental_Options)
        with Import, Convention => C, External_Name => "calib3d_test_fill_fundamental_options";
      Value : aliased ABI.C_Fundamental;
      Options : aliased ABI.C_Fundamental_Options;
      type Positions is array (Natural range <>) of Natural;
      Offsets : constant Positions := [Value.F00'Position, Value.F01'Position, Value.F02'Position,
        Value.F10'Position, Value.F11'Position, Value.F12'Position,
        Value.F20'Position, Value.F21'Position, Value.F22'Position];
      Option_Offsets : constant Positions :=
        [Options.Epipolar_Threshold_Pixels'Position, Options.Confidence'Position];
   begin
      Assert (ABI.C_Fundamental'Size = Natural (Layout (0)) * System.Storage_Unit, "fundamental C/Ada size");
      Assert (ABI.C_Fundamental'Alignment = Natural (Layout (1)), "fundamental C/Ada alignment");
      for I in Offsets'Range loop
         Assert (Offsets (I) = Natural (Layout (Interfaces.Integer_32 (I + 2))), "fundamental field offset");
      end loop;
      Fill (Value'Access);
      Assert (Value.F00 = 1.0 and then Value.F01 = 2.0 and then Value.F02 = 3.0 and then
              Value.F10 = 4.0 and then Value.F11 = 5.0 and then Value.F12 = 6.0 and then
              Value.F20 = 7.0 and then Value.F21 = 8.0 and then Value.F22 = 9.0, "C-written fundamental interchange");
      Assert (ABI.C_Fundamental_Options'Size = Natural (Options_Layout (0)) * System.Storage_Unit and then
              ABI.C_Fundamental_Options'Alignment = Natural (Options_Layout (1)), "fundamental options C/Ada layout");
      for I in Option_Offsets'Range loop
         Assert (Option_Offsets (I) = Natural (Options_Layout (Interfaces.Integer_32 (I + 2))), "options field offset");
      end loop;
      Fill_Options (Options'Access);
      Assert (Options.Epipolar_Threshold_Pixels = 3.0 and then
              Options.Confidence = 0.99, "C-written fundamental options interchange");
      Ada.Text_IO.Put_Line ("fundamental layout: compiler size=" & Interfaces.Integer_32'Image (Layout (0)) &
        " alignment=" & Interfaces.Integer_32'Image (Layout (1)) & "; all nine offsets/interchange; options layout PASS");
   end Fundamental_Layout;

   F_Policy : constant Fundamental_RANSAC_Options := (0.1, 0.999);
   F_Known : constant Fundamental_Matrix :=
     [[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]];
   function Stereo_Points (Second : Boolean) return Image_Point_Array is
   begin
      return Points : Image_Point_Array (1 .. 32) do
         for I in Points'Range loop
            declare
               J : constant Natural := I - 1;
               X : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((J * 7) mod 17 - 8) * 0.24;
               Y : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((J * 11) mod 19 - 9) * 0.19;
               Z : constant OpenCV.Float64_Value := 4.0 + OpenCV.Float64_Value ((J * 13) mod 23) * 0.17;
            begin
               Points (I) := [800.0 * (X - (if Second then 0.75 else 0.0)) / Z + 320.0,
                              820.0 * Y / Z + 240.0];
            end;
         end loop;
      end return;
   end Stereo_Points;

   procedure Fundamental_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      F : Fundamental_Matrix;
   begin
      for Scale of Reprojection_Error_Array'[1.0, -7.0, 1.0E6] loop
         F := F_Known;
         for V of F loop
            V := V * Scale;
         end loop;
         declare
            Zero : constant Epipolar_Error_Result := Maximum_Epipolar_Error (F, [100.0, 50.0], [80.0, 50.0]);
            Three : constant Epipolar_Error_Result := Maximum_Epipolar_Error (F, [100.0, 50.0], [80.0, 53.0]);
         begin
            Assert (Zero.Defined and then Zero.Maximum_Error_Pixels = 0.0, "independent zero-pixel oracle");
            Assert (Three.Defined and then Near (Three.Maximum_Error_Pixels, 3.0), "independent three-pixel/scale oracle");
         end;
      end loop;
      --  Only one of the two normals is zero, then both zero.
      F := [[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]];
      Assert (not Maximum_Epipolar_Error (F, [1.0, 2.0], [3.0, 4.0]).Defined, "zero first line normal");
      F := [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, -1.0, 0.0]];
      Assert (not Maximum_Epipolar_Error (F, [1.0, 2.0], [3.0, 4.0]).Defined, "zero second line normal");
      Assert (not Maximum_Epipolar_Error ([others => [others => 0.0]], [1.0, 2.0], [3.0, 4.0]).Defined,
              "zero matrix undefined");
      Ada.Text_IO.Put_Line ("known-F independent 0/3 pixel oracle; F/-7F/1e6F scale invariance; zero normals PASS");
   end Fundamental_Oracle;

   procedure Fundamental_Range (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check);
      F : Fundamental_Matrix := F_Known;
      E : Epipolar_Error_Result;
   begin
      for V of F loop
         V := V * 1.0E200;
      end loop;
      E := Maximum_Epipolar_Error (F, [100.0, 50.0], [80.0, 53.0]);
      Assert (E.Defined and then Near (E.Maximum_Error_Pixels, 3.0), "scaled hypot avoids naive squaring overflow");
      F (0, 0) := OpenCV.Float64_Value'Last;
      Assert (not Maximum_Epipolar_Error (F, [2.0, 50.0], [80.0, 53.0]).Defined, "product overflow undefined");
      F := F_Known; F (1, 2) := 1.0E-300; F (2, 1) := -1.0E-300;
      E := Maximum_Epipolar_Error (F, [100.0, 50.0], [80.0, 53.0]);
      Assert (E.Defined and then Near (E.Maximum_Error_Pixels, 3.0), "tiny normal is not epsilon-rejected");
      F := [[1.0E200, 1.0E200, 0.0], [0.0, 0.0, 1.0E200], [0.0, 0.0, 0.0]];
      E := Maximum_Epipolar_Error (F, [0.0, 0.0], [1.0, 1.0]);
      Assert (E.Defined and then Near (E.Maximum_Error_Pixels, 1.0), "two large components use scaled hypot");
      F := [[OpenCV.Float64_Value'Last, OpenCV.Float64_Value'Last, 0.0],
            [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]];
      Assert (not Maximum_Epipolar_Error (F, [1.0, 1.0], [0.0, 0.0]).Defined, "line sum overflow undefined");
      Assert (not Maximum_Epipolar_Error (F, [0.0, 0.0], [1.0, 0.0]).Defined, "norm overflow undefined");
      Assert (not Maximum_Epipolar_Error (F_Known, [0.0, -OpenCV.Float64_Value'Last],
                         [0.0, OpenCV.Float64_Value'Last]).Defined, "numerator overflow undefined");
      for Kind in Interfaces.Integer_32 range 0 .. 2 loop
         for Row in 0 .. 2 loop
            for Col in 0 .. 2 loop
               F := F_Known; F (Row, Col) := OpenCV.Float64_Value (Nonfinite (Kind));
               begin
                  E := Maximum_Epipolar_Error (F, [100.0, 50.0], [80.0, 53.0]);
                  Assert (False, "nonfinite F accepted");
               exception
                  when OpenCV.OpenCV_Error => null;
               end;
            end loop;
         end loop;
         for Which in 1 .. 2 loop
            for Component in 0 .. 1 loop
               declare
                  P1, P2 : Image_Point := [100.0, 50.0];
               begin
                  if Which = 1 then
                     P1 (Component) := OpenCV.Float64_Value (Nonfinite (Kind));
                  else
                     P2 (Component) := OpenCV.Float64_Value (Nonfinite (Kind));
                  end if;
                  begin
                     E := Maximum_Epipolar_Error (F_Known, P1, P2);
                     Assert (False, "nonfinite epipolar point accepted");
                  exception
                     when OpenCV.OpenCV_Error => null;
                  end;
               end;
            end loop;
         end loop;
      end loop;
   end Fundamental_Range;

   procedure Check_Fundamental_Final
     (Estimate : Fundamental_Estimate; First, Second : Image_Point_Array;
      Maximum : out OpenCV.Float64_Value)
   is
      Indices : constant Inlier_Index_Array := Inliers (Estimate);
      F : constant Fundamental_Matrix := Fundamental (Estimate);
   begin
      Maximum := 0.0;
      Assert (Found (Estimate) and then Inlier_Count (Estimate) >= 7, "minimum final support");
      for I in Indices'Range loop
         Assert (Indices (I) in 1 .. First'Length and then Inlier (Estimate, I) = Indices (I), "valid position");
         if I > 1 then
            Assert (Indices (I) > Indices (I - 1), "unique ascending positions");
         end if;
      end loop;
      for I in 1 .. First'Length loop
         declare
            P1 : constant Image_Point := First (First'First + I - 1);
            P2 : constant Image_Point := Second (Second'First + I - 1);
            E : constant Epipolar_Error_Result := Maximum_Epipolar_Error (F, P1, P2);
            --  Independent bounded scalar formula, separate from the API.
            A2 : constant OpenCV.Float64_Value := F (0, 0)*P1 (0)+F (0, 1)*P1 (1)+F (0, 2);
            B2 : constant OpenCV.Float64_Value := F (1, 0)*P1 (0)+F (1, 1)*P1 (1)+F (1, 2);
            C2 : constant OpenCV.Float64_Value := F (2, 0)*P1 (0)+F (2, 1)*P1 (1)+F (2, 2);
            A1 : constant OpenCV.Float64_Value := F (0, 0)*P2 (0)+F (1, 0)*P2 (1)+F (2, 0);
            B1 : constant OpenCV.Float64_Value := F (0, 1)*P2 (0)+F (1, 1)*P2 (1)+F (2, 1);
            C1 : constant OpenCV.Float64_Value := F (0, 2)*P2 (0)+F (1, 2)*P2 (1)+F (2, 2);
            Independent : constant OpenCV.Float64_Value := OpenCV.Float64_Value'Max
              (abs (A1*P1 (0)+B1*P1 (1)+C1)/Math.Sqrt (A1*A1+B1*B1),
               abs (A2*P2 (0)+B2*P2 (1)+C2)/Math.Sqrt (A2*A2+B2*B2));
         begin
            Assert (E.Defined and then Near (Independent, E.Maximum_Error_Pixels), "independent metric matches");
            Assert (Contains (Indices, I) = (E.Defined and then E.Maximum_Error_Pixels <= F_Policy.Epipolar_Threshold_Pixels),
                    "every included/excluded pair matches final F Float64 threshold");
            if Contains (Indices, I) then
               Maximum := OpenCV.Float64_Value'Max (Maximum, E.Maximum_Error_Pixels);
            end if;
         end;
      end loop;
      begin
         declare
            I : constant Positive := Inlier (Estimate, Inlier_Count (Estimate) + 1);
            pragma Unreferenced (I);
         begin
            Assert (False, "out-of-range public index accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
   end Check_Fundamental_Final;

   procedure Fundamental_Clean (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant Image_Point_Array (11 .. 42) := Stereo_Points (False);
      Second : constant Image_Point_Array (51 .. 82) := Stereo_Points (True);
      Estimate : constant Fundamental_Estimate := Estimate_Fundamental_RANSAC (First, Second, F_Policy);
      Maximum : OpenCV.Float64_Value;
   begin
      Check_Fundamental_Final (Estimate, First, Second, Maximum);
      Assert (Inlier_Count (Estimate) >= 31, "clean stereo nearly all accepted");
      Ada.Text_IO.Put_Line ("clean fundamental: final support=" & Natural'Image (Inlier_Count (Estimate)) &
        "; maximum final-model epipolar error=" & OpenCV.Float64_Value'Image (Maximum));
   end Fundamental_Clean;

   procedure Fundamental_Outliers (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant Image_Point_Array := Stereo_Points (False);
      Second : Image_Point_Array := Stereo_Points (True);
      Maximum : OpenCV.Float64_Value;
   begin
      for I in Second'Range loop
         if I = 2 or else I = 7 or else I = 15 then
            Second (I) := [Second (I) (0) + 50.0, Second (I) (1) + 500.0];
         end if;
      end loop;
      declare
         Estimate : constant Fundamental_Estimate := Estimate_Fundamental_RANSAC (First, Second, F_Policy);
      begin
         Check_Fundamental_Final (Estimate, First, Second, Maximum);
         Assert (not Contains (Inliers (Estimate), 2) and then not Contains (Inliers (Estimate), 7) and then
                 not Contains (Inliers (Estimate), 15), "gross vertical outliers rejected");
         Ada.Text_IO.Put_Line ("outlier fundamental: final support=" & Natural'Image (Inlier_Count (Estimate)) &
           "; 2/7/15 rejected; final Float64 inclusion/exclusion PASS");
      end;
   end Fundamental_Outliers;

   procedure Reject_Fundamental
     (First, Second : Image_Point_Array; Options : Fundamental_RANSAC_Options := F_Policy) is
   begin
      declare
         Estimate : constant Fundamental_Estimate := Estimate_Fundamental_RANSAC (First, Second, Options);
         pragma Unreferenced (Estimate);
      begin
         Assert (False, "invalid fundamental arguments accepted");
      end;
   exception
      when OpenCV.OpenCV_Error => null;
   end Reject_Fundamental;

   procedure Fundamental_Minimum (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant Image_Point_Array := Stereo_Points (False);
      Second : constant Image_Point_Array := Stereo_Points (True);
   begin
      for N in 0 .. 14 loop
         Reject_Fundamental (First (1 .. N), Second (1 .. N));
      end loop;
      declare
         Estimate : constant Fundamental_Estimate := Estimate_Fundamental_RANSAC
           (First (1 .. 15), Second (1 .. 15), F_Policy);
      begin
         Assert (Found (Estimate), "15 correspondences allowed true RANSAC");
      end;
      Ada.Text_IO.Put_Line ("fundamental 0..14 rejected / 15 native true-RANSAC permitted PASS");
   end Fundamental_Minimum;

   procedure Fundamental_No_Model (T : in out Fixture) is
      pragma Unreferenced (T);
      First, Second : Image_Point_Array (1 .. 32);
   begin
      for I in First'Range loop
         First (I) := [OpenCV.Float64_Value (I), OpenCV.Float64_Value (2 * I)];
         Second (I) := [OpenCV.Float64_Value (3 * I), OpenCV.Float64_Value (4 * I)];
      end loop;
      declare
         Estimate : constant Fundamental_Estimate := Estimate_Fundamental_RANSAC (First, Second, F_Policy);
         Indices : constant Inlier_Index_Array := Inliers (Estimate);
      begin
         Assert (not Found (Estimate) and then Inlier_Count (Estimate) = 0 and then
                 Indices'First = 1 and then Indices'Last = 0, "real collinear no-model semantics");
         begin
            declare
               F : constant Fundamental_Matrix := Fundamental (Estimate);
               pragma Unreferenced (F);
            begin
               Assert (False, "no-model matrix accessible");
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end;
      Ada.Text_IO.Put_Line ("fundamental collinear: native success, Found=False, empty 1..0, inaccessible matrix PASS");
   end Fundamental_No_Model;

   procedure Fundamental_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check);
      First : Image_Point_Array := Stereo_Points (False);
      Second : Image_Point_Array := Stereo_Points (True);
      Bad : Fundamental_RANSAC_Options;
      Epsilon : constant OpenCV.Float64_Value := 2.0**(-52);
   begin
      Reject_Fundamental (First, Second (1 .. 31));
      for V of Reprojection_Error_Array'[0.0, -1.0, 1.0E-30, 1.0E20, 1.0E-300, OpenCV.Float64_Value'Last] loop
         Bad := F_Policy; Bad.Epipolar_Threshold_Pixels := V;
         Reject_Fundamental (First, Second, Bad);
      end loop;
      for V of Reprojection_Error_Array'[0.0, -1.0, 1.0, 2.0, Epsilon / 2.0, 1.0 - Epsilon / 2.0] loop
         Bad := F_Policy; Bad.Confidence := V;
         Reject_Fundamental (First, Second, Bad);
      end loop;
      for Kind in Interfaces.Integer_32 range 0 .. 2 loop
         Bad := F_Policy; Bad.Confidence := OpenCV.Float64_Value (Nonfinite (Kind));
         Reject_Fundamental (First, Second, Bad);
         Bad := F_Policy; Bad.Epipolar_Threshold_Pixels := OpenCV.Float64_Value (Nonfinite (Kind));
         Reject_Fundamental (First, Second, Bad);
         First (1) (0) := OpenCV.Float64_Value (Nonfinite (Kind));
         Reject_Fundamental (First, Second); First := Stereo_Points (False);
         Second (1) (1) := OpenCV.Float64_Value (Nonfinite (Kind));
         Reject_Fundamental (First, Second); Second := Stereo_Points (True);
      end loop;
   end Fundamental_Invalid;

   E_Policy : constant Essential_RANSAC_Options := (1.0E-6, 0.999);
   E_Rotation : constant Rotation_Matrix :=
     [[Math.Cos (0.08), 0.0, Math.Sin (0.08)], [0.0, 1.0, 0.0],
      [-Math.Sin (0.08), 0.0, Math.Cos (0.08)]];
   E_Translation : constant Camera_Direction := [-0.75, 0.08, 0.12];
   E_Norm : constant OpenCV.Float64_Value := Math.Sqrt (0.75**2 + 0.08**2 + 0.12**2);
   function Two_View_Points (Second : Boolean) return Normalized_Image_Point_Array is
   begin
      return Points : Normalized_Image_Point_Array (1 .. 40) do
         for I in Points'Range loop
            declare
               J : constant Integer := I - 1;
               X : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((J * 7) mod 17 - 8) * 0.24;
               Y : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((J * 11) mod 19 - 9) * 0.19;
               Z : constant OpenCV.Float64_Value := 4.0 + OpenCV.Float64_Value ((J * 13) mod 23) * 0.17;
               X2 : constant OpenCV.Float64_Value := E_Rotation (0, 0) * X + E_Rotation (0, 2) * Z - 0.75;
               Y2 : constant OpenCV.Float64_Value := Y + 0.08;
               Z2 : constant OpenCV.Float64_Value := E_Rotation (2, 0) * X + E_Rotation (2, 2) * Z + 0.12;
            begin
               Assert (Z > 0.0 and then Z2 > 0.0, "synthetic fixture positive depths");
               Points (I) := (if Second then [X2 / Z2, Y2 / Z2] else [X / Z, Y / Z]);
            end;
         end loop;
      end return;
   end Two_View_Points;

   function Known_Essential return Essential_Matrix is
      TX : constant OpenCV.Float64_Value := E_Translation (0) / E_Norm;
      TY : constant OpenCV.Float64_Value := E_Translation (1) / E_Norm;
      TZ : constant OpenCV.Float64_Value := E_Translation (2) / E_Norm;
      Cross : constant Rotation_Matrix := [[0.0, -TZ, TY], [TZ, 0.0, -TX], [-TY, TX, 0.0]];
   begin
      return E : Essential_Matrix := [others => [others => 0.0]] do
         for Row in 0 .. 2 loop
            for Col in 0 .. 2 loop
               for K in 0 .. 2 loop
                  E (Row, Col) := E (Row, Col) + Cross (Row, K) * E_Rotation (K, Col);
               end loop;
            end loop;
         end loop;
      end return;
   end Known_Essential;

   procedure Essential_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      Simple : constant Essential_Matrix := [[0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]];
      Known : constant Essential_Matrix := Known_Essential;
      First : constant Normalized_Image_Point_Array := Two_View_Points (False);
      Second : constant Normalized_Image_Point_Array := Two_View_Points (True);
      Scaled : Essential_Matrix;
   begin
      for Scale of Reprojection_Error_Array'[1.0, -7.0, 1.0E6] loop
         for Row in 0 .. 2 loop
            for Col in 0 .. 2 loop
               Scaled (Row, Col) := Scale * Simple (Row, Col);
            end loop;
         end loop;
         declare
            Zero : constant Normalized_Sampson_Error_Result :=
              Normalized_Sampson_Error (Scaled, [0.2, 0.1], [-0.1, 0.1]);
            Nonzero : constant Normalized_Sampson_Error_Result :=
              Normalized_Sampson_Error (Scaled, [0.2, 0.1], [-0.1, 0.3]);
         begin
            Assert (Zero.Defined and then Zero.Error = 0.0, "translation-only exact Sampson");
            --  r=y1-y2, four-normal norm=sqrt(2): independently derived.
            Assert (Nonzero.Defined and then Near (Nonzero.Error, 0.2 / Math.Sqrt (2.0), 1.0E-15),
                    "independent nonzero normalized Sampson and scale/sign oracle");
         end;
         for Row in 0 .. 2 loop
            for Col in 0 .. 2 loop
               Scaled (Row, Col) := Scale * Known (Row, Col);
            end loop;
         end loop;
         for I in First'Range loop
            declare
               Error : constant Normalized_Sampson_Error_Result :=
                 Normalized_Sampson_Error (Scaled, First (I), Second (I));
            begin
               Assert (Error.Defined and then Error.Error < 1.0E-15, "known [t]_x R exact geometry");
            end;
         end loop;
      end loop;
      Ada.Text_IO.Put_Line ("independent Essential [t]_x R / normalized Sampson zero/nonzero/scale/sign oracle PASS");
   end Essential_Oracle;

   procedure Essential_Error_Range (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check);
      E : Essential_Matrix := [others => [others => 0.0]];
   begin
      Assert (not Normalized_Sampson_Error (E, [0.2, 0.1], [-0.1, 0.1]).Defined, "zero denominator");
      E (0, 0) := OpenCV.Float64_Value'Last;
      Assert (not Normalized_Sampson_Error (E, [2.0, 0.0], [0.0, 0.0]).Defined, "product overflow");
      E (0, 1) := OpenCV.Float64_Value'Last;
      Assert (not Normalized_Sampson_Error (E, [1.0, 1.0], [0.0, 0.0]).Defined, "sum overflow");
      E := [others => [others => 0.0]]; E (0, 2) := OpenCV.Float64_Value'Last;
      Assert (not Normalized_Sampson_Error (E, [0.0, 0.0], [2.0, 0.0]).Defined, "residual overflow");
      E := [others => [others => 0.0]];
      E (0, 2) := 1.0E-300; E (2, 2) := OpenCV.Float64_Value'Last;
      Assert (not Normalized_Sampson_Error (E, [0.0, 0.0], [0.0, 0.0]).Defined, "division overflow");
      E := [others => [others => 0.0]]; E (0, 2) := 1.0E200; E (2, 1) := -1.0E200;
      declare
         Error : constant Normalized_Sampson_Error_Result :=
           Normalized_Sampson_Error (E, [0.2, 0.1], [-0.1, 0.3]);
      begin
         Assert (Error.Defined and then Near (Error.Error, 0.2 / Math.Sqrt (2.0)), "scaled robust norm");
      end;
      for Kind in Interfaces.Integer_32 range 0 .. 2 loop
         for Input in 0 .. 12 loop
            declare
               Matrix : Essential_Matrix := Known_Essential;
               First, Second : Normalized_Image_Point := [0.2, 0.1];
               Bad : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Nonfinite (Kind));
            begin
               if Input < 9 then
                  Matrix (Input / 3, Input mod 3) := Bad;
               elsif Input < 11 then
                  First (Input - 9) := Bad;
               else
                  Second (Input - 11) := Bad;
               end if;
               begin
                  declare
                     Error : constant Normalized_Sampson_Error_Result := Normalized_Sampson_Error (Matrix, First, Second);
                     pragma Unreferenced (Error);
                  begin
                     Assert (False, "nonfinite Sampson input accepted");
                  end;
               exception
                  when OpenCV.OpenCV_Error => null;
               end;
            end;
         end loop;
      end loop;
   end Essential_Error_Range;

   procedure Check_Essential (Estimate : Essential_Estimate;
                              First, Second : Normalized_Image_Point_Array; Label : String) is
      E : constant Essential_Matrix := Essential (Estimate);
      EI : constant Inlier_Index_Array := Inliers (Estimate);
      PI : constant Inlier_Index_Array := Pose_Inliers (Estimate);
      Pose : constant Relative_Camera_Pose := Recovered_Pose (Estimate);
      R : constant Rotation_Matrix := Pose.Rotation_First_To_Second;
      Center : constant Camera_Direction := Second_Camera_Center_Direction_In_First (Pose);
      Maximum, Trace, Dot, Center_Error, Unit_Norm, Orthogonal_Error : OpenCV.Float64_Value := 0.0;
      Angle, Determinant : OpenCV.Float64_Value;
   begin
      Assert (Found (Estimate) and then Pose_Recovered (Estimate), "Essential and relative pose found");
      Assert (EI'Length >= 5 and then PI'Length >= 5, "Essential/pose support minimum");
      for I in EI'Range loop
         Assert (EI (I) <= First'Length and then Inlier (Estimate, I) = EI (I), "valid Essential index");
         if I > 1 then
            Assert (EI (I) > EI (I - 1), "unique ascending Essential indices");
         end if;
      end loop;
      for I in PI'Range loop
         Assert (Contains (EI, PI (I)) and then Pose_Inlier (Estimate, I) = PI (I), "pose subset/index");
         if I > 1 then
            Assert (PI (I) > PI (I - 1), "unique ascending pose indices");
         end if;
      end loop;
      for I in 1 .. First'Length loop
         declare
            Error : constant Normalized_Sampson_Error_Result := Normalized_Sampson_Error
              (E, First (First'First + I - 1), Second (Second'First + I - 1));
            Accepted : constant Boolean := Error.Defined and then Error.Error <= E_Policy.Normalized_Epipolar_Threshold;
         begin
            Assert (Contains (EI, I) = Accepted, "exact final-E original Float64 inclusion/exclusion");
            if Accepted then
               Maximum := OpenCV.Float64_Value'Max (Maximum, Error.Error);
            end if;
         end;
      end loop;
      for Row in 0 .. 2 loop
         declare
            Expected_Center : OpenCV.Float64_Value := 0.0;
         begin
            for Col in 0 .. 2 loop
               Trace := Trace + R (Row, Col) * E_Rotation (Row, Col);
               Expected_Center := Expected_Center - E_Rotation (Col, Row) * E_Translation (Col) / E_Norm;
               declare
                  Product : OpenCV.Float64_Value := 0.0;
               begin
                  for Axis in 0 .. 2 loop
                     Product := Product + R (Axis, Row) * R (Axis, Col);
                  end loop;
                  Orthogonal_Error := OpenCV.Float64_Value'Max
                    (Orthogonal_Error, abs (Product - (if Row = Col then 1.0 else 0.0)));
               end;
            end loop;
            Dot := Dot + Pose.Translation_Direction (Row) * E_Translation (Row) / E_Norm;
            Unit_Norm := Unit_Norm + Pose.Translation_Direction (Row)**2;
            Center_Error := Center_Error + (Center (Row) - Expected_Center)**2;
         end;
      end loop;
      Angle := Math.Arccos (OpenCV.Float64_Value'Max (-1.0, OpenCV.Float64_Value'Min (1.0, (Trace - 1.0) / 2.0)));
      Determinant := R (0, 0) * (R (1, 1) * R (2, 2) - R (1, 2) * R (2, 1))
        - R (0, 1) * (R (1, 0) * R (2, 2) - R (1, 2) * R (2, 0))
        + R (0, 2) * (R (1, 0) * R (2, 1) - R (1, 1) * R (2, 0));
      Assert (Angle < 1.0E-5 and then Dot > 1.0 - 1.0E-8, "rotation angular and signed translation oracle");
      Assert (Math.Sqrt (Center_Error) < 1.0E-5, "camera center normalize(-R^T t), not t");
      Assert (Near (Unit_Norm, 1.0, 1.0E-12) and then Near (Determinant, 1.0, 1.0E-12)
              and then Orthogonal_Error < 1.0E-12, "SO(3) and unit direction qualification");
      for Which in 0 .. 1 loop
         begin
            declare
               Index : constant Positive := (if Which = 0 then Inlier (Estimate, EI'Length + 1)
                                             else Pose_Inlier (Estimate, PI'Length + 1));
               pragma Unreferenced (Index);
            begin
               Assert (False, "out of range inlier accepted");
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end loop;
      Ada.Text_IO.Put_Line (Label & " Essential support=" & Natural'Image (EI'Length) &
        " Pose support=" & Natural'Image (PI'Length) & " max Sampson=" & OpenCV.Float64_Value'Image (Maximum));
      Ada.Text_IO.Put_Line ("rotation angle=" & OpenCV.Float64_Value'Image (Angle) &
        " signed t dot=" & OpenCV.Float64_Value'Image (Dot) & " center direction error=" &
        OpenCV.Float64_Value'Image (Math.Sqrt (Center_Error)) & " orthogonality error=" &
        OpenCV.Float64_Value'Image (Orthogonal_Error) & " det=" & OpenCV.Float64_Value'Image (Determinant));
   end Check_Essential;

   procedure Essential_Clean (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant Normalized_Image_Point_Array := Two_View_Points (False);
      Second : constant Normalized_Image_Point_Array := Two_View_Points (True);
      Estimate : constant Essential_Estimate := Estimate_Essential_RANSAC (First, Second, E_Policy);
   begin
      Check_Essential (Estimate, First, Second, "clean");
   end Essential_Clean;
   procedure Essential_Outliers (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant Normalized_Image_Point_Array := Two_View_Points (False);
      Second : Normalized_Image_Point_Array := Two_View_Points (True);
   begin
      for I of Inlier_Index_Array'[2, 7, 15] loop
         Second (I) (0) := Second (I) (0) + 2.0;
         Second (I) (1) := Second (I) (1) - 1.5;
      end loop;
      declare
         Estimate : constant Essential_Estimate := Estimate_Essential_RANSAC (First, Second, E_Policy);
      begin
         Check_Essential (Estimate, First, Second, "outlier");
         for I of Inlier_Index_Array'[2, 7, 15] loop
            Assert (not Contains (Inliers (Estimate), I) and then not Contains (Pose_Inliers (Estimate), I),
                    "gross Essential/pose outlier survived");
         end loop;
      end;
      Ada.Text_IO.Put_Line ("Essential/pose 2/7/15 absent; final Float64 classification PASS");
   end Essential_Outliers;

   procedure Reject_Essential (First, Second : Normalized_Image_Point_Array;
                               Options : Essential_RANSAC_Options := E_Policy) is
   begin
      declare
         Estimate : constant Essential_Estimate := Estimate_Essential_RANSAC (First, Second, Options);
         pragma Unreferenced (Estimate);
      begin
         Assert (False, "invalid Essential input accepted");
      end;
   exception
      when OpenCV.OpenCV_Error => null;
   end Reject_Essential;
   procedure Essential_Minimum (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant Normalized_Image_Point_Array := Two_View_Points (False);
      Second : constant Normalized_Image_Point_Array := Two_View_Points (True);
   begin
      for N in 0 .. 5 loop
         Reject_Essential (First (1 .. N), Second (1 .. N));
      end loop;
      declare
         Estimate : constant Essential_Estimate := Estimate_Essential_RANSAC (First (1 .. 6), Second (1 .. 6), E_Policy);
      begin
         Assert (Found (Estimate), "six points permitted actual Essential RANSAC");
      end;
      Ada.Text_IO.Put_Line ("Essential 0..5 rejection / 6 native subset RANSAC PASS (armed proof in fault helper)");
   end Essential_Minimum;
   procedure Essential_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check);
      First : Normalized_Image_Point_Array := Two_View_Points (False);
      Second : Normalized_Image_Point_Array := Two_View_Points (True);
      Bad : Essential_RANSAC_Options;
   begin
      Reject_Essential (First, Second (1 .. 39));
      for V of Reprojection_Error_Array'[0.0, -1.0, 1.0E-30, 1.0E20, OpenCV.Float64_Value'Last] loop
         Bad := E_Policy; Bad.Normalized_Epipolar_Threshold := V;
         Reject_Essential (First, Second, Bad);
      end loop;
      for V of Reprojection_Error_Array'[0.0, -1.0, 1.0, 2.0] loop
         Bad := E_Policy; Bad.Confidence := V;
         Reject_Essential (First, Second, Bad);
      end loop;
      for Kind in Interfaces.Integer_32 range 0 .. 2 loop
         Bad := E_Policy; Bad.Confidence := OpenCV.Float64_Value (Nonfinite (Kind));
         Reject_Essential (First, Second, Bad);
         Bad := E_Policy; Bad.Normalized_Epipolar_Threshold := OpenCV.Float64_Value (Nonfinite (Kind));
         Reject_Essential (First, Second, Bad);
         for Axis in 0 .. 1 loop
            First (1) (Axis) := OpenCV.Float64_Value (Nonfinite (Kind));
            Reject_Essential (First, Second); First := Two_View_Points (False);
            Second (1) (Axis) := OpenCV.Float64_Value (Nonfinite (Kind));
            Reject_Essential (First, Second); Second := Two_View_Points (True);
         end loop;
      end loop;
      for V of Reprojection_Error_Array'[2.0**(-53), 1.0 - 2.0**(-53)] loop
         declare
            Options : aliased constant ABI.C_Essential_Options := (1.0E-3, Interfaces.C.double (V));
         begin
            ABI.Check (ABI.Validate_Essential_Options (Options'Access), "Essential exact open confidence interval");
         end;
      end loop;
   end Essential_Invalid;

   procedure Relative_Center_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check);
      Pose : Relative_Camera_Pose := (E_Rotation, E_Translation);
   begin
      for Input in 0 .. 12 loop
         Pose := (E_Rotation, E_Translation);
         if Input < 9 then
            Pose.Rotation_First_To_Second (Input / 3, Input mod 3) := OpenCV.Float64_Value (Nonfinite (0));
         elsif Input < 12 then
            Pose.Translation_Direction (Input - 9) := OpenCV.Float64_Value (Nonfinite (0));
         else
            Pose.Translation_Direction := [others => 0.0];
         end if;
         begin
            declare
               Center : constant Camera_Direction := Second_Camera_Center_Direction_In_First (Pose);
               pragma Unreferenced (Center);
            begin
               Assert (False, "invalid relative pose direction accepted");
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end loop;
   end Relative_Center_Invalid;

   procedure Essential_Layout (T : in out Fixture) is
      pragma Unreferenced (T);
      function Layout (Field : Interfaces.Integer_32) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_essential_layout";
      procedure Fill (Value : access ABI.C_Essential)
        with Import, Convention => C, External_Name => "calib3d_test_fill_essential";
      function Pose_Layout (Field : Interfaces.Integer_32) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_relative_pose_layout";
      procedure Fill_Pose (Value : access ABI.C_Relative_Pose)
        with Import, Convention => C, External_Name => "calib3d_test_fill_relative_pose";
      function Options_Layout (Field : Interfaces.Integer_32) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_essential_options_layout";
      procedure Fill_Options (Value : access ABI.C_Essential_Options)
        with Import, Convention => C, External_Name => "calib3d_test_fill_essential_options";
      Value : aliased ABI.C_Essential;
      Pose : aliased ABI.C_Relative_Pose;
      Options : aliased ABI.C_Essential_Options;
      type Positions is array (Natural range <>) of Natural;
      Offsets : constant Positions := [Value.E00'Position, Value.E01'Position, Value.E02'Position,
        Value.E10'Position, Value.E11'Position, Value.E12'Position,
        Value.E20'Position, Value.E21'Position, Value.E22'Position];
      Pose_Offsets : constant Positions := [Pose.R00'Position, Pose.R01'Position, Pose.R02'Position,
        Pose.R10'Position, Pose.R11'Position, Pose.R12'Position,
        Pose.R20'Position, Pose.R21'Position, Pose.R22'Position,
        Pose.TX'Position, Pose.TY'Position, Pose.TZ'Position];
      Option_Offsets : constant Positions := [Options.Normalized_Epipolar_Threshold'Position, Options.Confidence'Position];
   begin
      Assert (ABI.C_Essential'Size = Natural (Layout (0)) * System.Storage_Unit and then
              ABI.C_Essential'Alignment = Natural (Layout (1)), "Essential C/Ada size/alignment");
      for I in Offsets'Range loop
         Assert (Offsets (I) = Natural (Layout (Interfaces.Integer_32 (I + 2))), "Essential field position");
      end loop;
      Fill (Value'Access);
      Assert (Value.E00 = 1.0 and then Value.E01 = 2.0 and then Value.E02 = 3.0 and then
              Value.E10 = 4.0 and then Value.E11 = 5.0 and then Value.E12 = 6.0 and then
              Value.E20 = 7.0 and then Value.E21 = 8.0 and then Value.E22 = 9.0, "C-written all-nine Essential interchange");
      Assert (ABI.C_Relative_Pose'Size = Natural (Pose_Layout (0)) * System.Storage_Unit and then
              ABI.C_Relative_Pose'Alignment = Natural (Pose_Layout (1)), "relative pose C/Ada size/alignment");
      for I in Pose_Offsets'Range loop
         Assert (Pose_Offsets (I) = Natural (Pose_Layout (Interfaces.Integer_32 (I + 2))), "relative pose field position");
      end loop;
      Fill_Pose (Pose'Access);
      Assert (Pose.R00 = 1.0 and then Pose.R01 = 2.0 and then Pose.R02 = 3.0 and then
              Pose.R10 = 4.0 and then Pose.R11 = 5.0 and then Pose.R12 = 6.0 and then
              Pose.R20 = 7.0 and then Pose.R21 = 8.0 and then Pose.R22 = 9.0 and then
              Pose.TX = 10.0 and then Pose.TY = 11.0 and then Pose.TZ = 12.0, "C-written all-twelve relative pose interchange");
      Assert (ABI.C_Essential_Options'Size = Natural (Options_Layout (0)) * System.Storage_Unit and then
              ABI.C_Essential_Options'Alignment = Natural (Options_Layout (1)), "Essential options C/Ada layout");
      for I in Option_Offsets'Range loop
         Assert (Option_Offsets (I) = Natural (Options_Layout (Interfaces.Integer_32 (I + 2))), "Essential options position");
      end loop;
      Fill_Options (Options'Access);
      Assert (Options.Normalized_Epipolar_Threshold = 0.001 and then Options.Confidence = 0.999, "C-written Essential options");
      Ada.Text_IO.Put_Line ("Essential/relative pose/options compiler sizes=" &
        Interfaces.Integer_32'Image (Layout (0)) & Interfaces.Integer_32'Image (Pose_Layout (0)) &
        Interfaces.Integer_32'Image (Options_Layout (0)) & " alignments=" &
        Interfaces.Integer_32'Image (Layout (1)) & Interfaces.Integer_32'Image (Pose_Layout (1)) &
        Interfaces.Integer_32'Image (Options_Layout (1)) & "; all 9/12/2 positions and C-written interchange PASS");
   end Essential_Layout;

   procedure Essential_Far_Points (T : in out Fixture) is
      pragma Unreferenced (T);
      First, Second : Normalized_Image_Point_Array (1 .. 40);
   begin
      for I in First'Range loop
         declare
            J : constant Integer := I - 1;
            X : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((J * 7) mod 17 - 8) * 0.24;
            Y : constant OpenCV.Float64_Value := OpenCV.Float64_Value ((J * 11) mod 19 - 9) * 0.19;
            Z : constant OpenCV.Float64_Value := 100.0 + OpenCV.Float64_Value ((J * 13) mod 23) * 1.7;
            X2 : constant OpenCV.Float64_Value := E_Rotation (0, 0) * X + E_Rotation (0, 2) * Z - 0.75;
            Z2 : constant OpenCV.Float64_Value := E_Rotation (2, 0) * X + E_Rotation (2, 2) * Z + 0.12;
         begin
            --  Every depth exceeds 50 baseline units. The explicit distance
            --  threshold must not reject this supported positive-depth fixture.
            Assert (Z / E_Norm > 50.0 and then Z2 / E_Norm > 50.0, "far fixture depths");
            First (I) := [X / Z, Y / Z];
            Second (I) := [X2 / Z2, (Y + 0.08) / Z2];
         end;
      end loop;
      declare
         Estimate : constant Essential_Estimate := Estimate_Essential_RANSAC (First, Second, E_Policy);
      begin
         Check_Essential (Estimate, First, Second, "far (>50 baseline units)");
      end;
   end Essential_Far_Points;

   package Caller is new AUnit.Test_Caller (Fixture);

   procedure Triangulation_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check);
      Identity : constant Rotation_Matrix := [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
      Pose : Relative_Camera_Pose := (Identity, [-1.0, 0.0, 0.0]);
      procedure Reject_Points (First, Second : Normalized_Image_Point_Array) is
      begin
         declare
            Points : constant Triangulated_Point_Array := Triangulate_Normalized
              (First, Second, (Identity, [-1.0, 0.0, 0.0]));
            pragma Unreferenced (Points);
         begin
            Assert (False, "invalid correspondences accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Reject_Points;
      procedure Reject (P : Relative_Camera_Pose) is
      begin
         declare
            Points : constant Triangulated_Point_Array := Triangulate_Normalized
              ([1 .. 0 => <>], [1 .. 0 => <>], P);
            pragma Unreferenced (Points);
         begin
            Assert (False, "invalid empty-request pose accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Reject;
   begin
      Pose.Rotation_First_To_Second := [others => [others => 0.0]]; Reject (Pose);
      Pose.Rotation_First_To_Second := Identity;
      Pose.Rotation_First_To_Second (0, 0) := -1.0; Reject (Pose);
      Pose.Rotation_First_To_Second (0, 0) := 0.9; Reject (Pose);
      Pose.Rotation_First_To_Second := Identity;
      Pose.Translation_Direction := [others => 0.0]; Reject (Pose);
      for Kind in 0 .. 2 loop
         Pose := (Identity, [-1.0, 0.0, 0.0]);
         Pose.Rotation_First_To_Second (0, 0) := OpenCV.Float64_Value (Nonfinite (Interfaces.Integer_32 (Kind))); Reject (Pose);
         Pose := (Identity, [-1.0, 0.0, 0.0]);
         Pose.Translation_Direction (1) := OpenCV.Float64_Value (Nonfinite (Interfaces.Integer_32 (Kind))); Reject (Pose);
         for Axis in 0 .. 1 loop
            declare
               Bad : Normalized_Image_Point_Array := [[0.4, 0.2]];
            begin
               Bad (1) (Axis) := OpenCV.Float64_Value (Nonfinite (Interfaces.Integer_32 (Kind)));
               Reject_Points (Bad, [[0.2, 0.2]]);
               Reject_Points ([[0.4, 0.2]], Bad);
            end;
         end loop;
      end loop;
      Reject_Points ([[0.4, 0.2]], [1 .. 0 => <>]);
   end Triangulation_Invalid;

   procedure Triangulation_Depths (T : in out Fixture) is
      pragma Unreferenced (T);
      Identity : constant Rotation_Matrix := [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
      Forward : constant Triangulated_Point_Array := Triangulate_Normalized
        ([[0.4, 0.0]], [[-0.4, 0.0]], (Identity, [0.0, 0.0, -1.0]));
      Reverse_View : constant Triangulated_Point_Array := Triangulate_Normalized
        ([[-0.4, 0.0]], [[0.4, 0.0]], (Identity, [0.0, 0.0, 1.0]));
   begin
      --  Independent known points: (0.2,0,0.5) and (0.2,0,-0.5).
      --  Forward depths +0.5/-0.5; reversed depths -0.5/+0.5.
      Assert (Forward (1).Status = Non_Positive_Depth, "second-camera-only negative depth");
      Assert (Reverse_View (1).Status = Non_Positive_Depth, "first-camera-only negative depth");
      Ada.Text_IO.Put_Line ("Triangulation analytic depth oracles: first/second +0.5/-0.5 and -0.5/+0.5 PASS");
   end Triangulation_Depths;

   procedure Triangulation_Noisy (T : in out Fixture) is
      pragma Unreferenced (T);
      Pose : constant Relative_Camera_Pose :=
        ([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [-1.0, 0.0, 0.0]);
      Points : constant Triangulated_Point_Array := Triangulate_Normalized
        ([[0.4, 0.2]], [[0.2, 0.23]], Pose);
      P : Triangulated_Point renames Points (1);
      X, Y, Z, A, B : OpenCV.Float64_Value;
      function Hypot (A, B : OpenCV.Float64_Value) return OpenCV.Float64_Value is
         Scale : constant OpenCV.Float64_Value := OpenCV.Float64_Value'Max (abs A, abs B);
      begin
         return (if Scale = 0.0 then 0.0 else Scale * Math.Sqrt ((A / Scale)**2 + (B / Scale)**2));
      end Hypot;
   begin
      Assert (P.Status = Usable, "no automatic noisy residual rejection");
      X := P.Position_In_First_Camera (0);
      Y := P.Position_In_First_Camera (1);
      Z := P.Position_In_First_Camera (2);
      A := X / Z - 0.4; B := Y / Z - 0.2;
      Assert (Near (P.First_Normalized_Error, Hypot (A, B), 1.0E-12),
              "independent first normalized reprojection");
      A := (X - 1.0) / Z - 0.2; B := Y / Z - 0.23;
      Assert (Near (P.Second_Normalized_Error, Hypot (A, B), 1.0E-12),
              "independent second normalized reprojection");
      Assert (P.First_Normalized_Error > 0.0 and then P.Second_Normalized_Error > 0.0,
              "noise produces errors in both images");
   end Triangulation_Noisy;

   procedure Triangulation_Layout (T : in out Fixture) is
      pragma Unreferenced (T);
      function Layout (Field : Interfaces.Integer_32) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_triangulation_layout";
      procedure Fill (Value : access ABI.C_Triangulated_Point)
        with Import, Convention => C, External_Name => "calib3d_test_fill_triangulation";
      Point : aliased ABI.C_Triangulated_Point;
      type Positions is array (Natural range <>) of Natural;
      Offsets : constant Positions := [Point.Point_Status'Position, Point.X'Position,
        Point.Y'Position, Point.Z'Position, Point.Depth_First'Position,
        Point.Depth_Second'Position, Point.Error_First'Position, Point.Error_Second'Position];
   begin
      Assert (ABI.C_Triangulated_Point'Size = Natural (Layout (0)) * System.Storage_Unit and then
              ABI.C_Triangulated_Point'Alignment = Natural (Layout (1)), "triangulation size/alignment");
      for I in Offsets'Range loop
         Assert (Offsets (I) = Natural (Layout (Interfaces.Integer_32 (I + 2))), "triangulation component position");
      end loop;
      Fill (Point'Access);
      Assert (Point.Point_Status = 4 and then Point.X = 1.0 and then Point.Y = 2.0 and then
              Point.Z = 3.0 and then Point.Depth_First = 4.0 and then Point.Depth_Second = 5.0 and then
              Point.Error_First = 6.0 and then Point.Error_Second = 7.0, "C-written full triangulation record");
      Ada.Text_IO.Put_Line ("Triangulation compiler size=" & Interfaces.Integer_32'Image (Layout (0)) &
        " alignment=" & Interfaces.Integer_32'Image (Layout (1)) & " all eight positions/interchange PASS");
   end Triangulation_Layout;

   procedure Triangulation_Rectified (T : in out Fixture) is
      pragma Unreferenced (T);
      Pose : constant Relative_Camera_Pose :=
        ([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [-7.0, 0.0, 0.0]);
      First : constant Normalized_Image_Point_Array (5 .. 6) := [[0.4, 0.2], [0.4, 0.2]];
      Second : constant Normalized_Image_Point_Array (9 .. 10) := [[0.2, 0.2], [0.6, 0.2]];
      Result : constant Triangulated_Point_Array := Triangulate_Normalized (First, Second, Pose);
      One : constant Triangulated_Point_Array := Triangulate_Normalized (First (5 .. 5), Second (9 .. 9), Pose);
      Empty : constant Triangulated_Point_Array := Triangulate_Normalized
        ([1 .. 0 => <>], [1 .. 0 => <>], Pose);
   begin
      Assert (Result'First = 1 and then Result'Length = 2 and then One'Length = 1,
              "one point and iteration-order pairing");
      Assert (Empty'First = 1 and then Empty'Last = 0, "empty bounds");
      Assert (Result (1).Status = Usable and then Result (2).Status = Non_Positive_Depth,
              "rectified positive/negative disparity");
      Assert (Near (Result (1).Position_In_First_Camera (0), 2.0) and then
              Near (Result (1).Position_In_First_Camera (1), 1.0) and then
              Near (Result (1).Position_In_First_Camera (2), 5.0) and then
              Near (Result (1).First_Depth, 5.0) and then Near (Result (1).Second_Depth, 5.0),
              "independent disparity geometry");
      Assert (Result (1).First_Normalized_Error < 1.0E-12 and then
              Result (1).Second_Normalized_Error < 1.0E-12, "normalized residuals");
      Ada.Text_IO.Put_Line ("Triangulation rectified coordinate/depth absolute errors=" &
        OpenCV.Float64_Value'Image (abs (Result (1).Position_In_First_Camera (0) - 2.0)) &
        OpenCV.Float64_Value'Image (abs (Result (1).Position_In_First_Camera (1) - 1.0)) &
        OpenCV.Float64_Value'Image (abs (Result (1).Position_In_First_Camera (2) - 5.0)));
   end Triangulation_Rectified;

   procedure Triangulation_Frames (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant Normalized_Image_Point_Array := Two_View_Points (False);
      Second : constant Normalized_Image_Point_Array := Two_View_Points (True);
      Pose : constant Relative_Camera_Pose := (E_Rotation, E_Translation);
      Reverse_Pose : Relative_Camera_Pose;
      Forward : constant Triangulated_Point_Array := Triangulate_Normalized (First, Second, Pose);
      Maximum_Forward, Maximum_Reverse : OpenCV.Float64_Value := 0.0;
   begin
      for I in 0 .. 2 loop
         Reverse_Pose.Translation_Direction (I) := 0.0;
         for J in 0 .. 2 loop
            Reverse_Pose.Rotation_First_To_Second (I, J) := E_Rotation (J, I);
            Reverse_Pose.Translation_Direction (I) := Reverse_Pose.Translation_Direction (I) -
              E_Rotation (J, I) * E_Translation (J) / E_Norm;
         end loop;
      end loop;
      declare
         Backward : constant Triangulated_Point_Array := Triangulate_Normalized (Second, First, Reverse_Pose);
      begin
         for I in Forward'Range loop
            declare
               J : constant Integer := I - 1;
               Expected : constant Object_Point :=
                 [OpenCV.Float64_Value ((J * 7) mod 17 - 8) * 0.24 / E_Norm,
                  OpenCV.Float64_Value ((J * 11) mod 19 - 9) * 0.19 / E_Norm,
                  (4.0 + OpenCV.Float64_Value ((J * 13) mod 23) * 0.17) / E_Norm];
               Second_Position : OpenCV.Float64_Value;
            begin
               Assert (Forward (I).Status = Usable and then Backward (I).Status = Usable, "both frames usable");
               for Axis in 0 .. 2 loop
                  Assert (Near (Forward (I).Position_In_First_Camera (Axis), Expected (Axis), 1.0E-8),
                          "independent nonidentity baseline-scaled oracle");
                  Maximum_Forward := OpenCV.Float64_Value'Max (Maximum_Forward,
                    abs (Forward (I).Position_In_First_Camera (Axis) - Expected (Axis)));
                  Second_Position := E_Translation (Axis) / E_Norm;
                  for K in 0 .. 2 loop
                     Second_Position := Second_Position + E_Rotation (Axis, K) * Expected (K);
                  end loop;
                  Assert (Near (Backward (I).Position_In_First_Camera (Axis), Second_Position, 1.0E-8),
                          "inverse-frame geometry oracle");
                  Maximum_Reverse := OpenCV.Float64_Value'Max (Maximum_Reverse,
                    abs (Backward (I).Position_In_First_Camera (Axis) - Second_Position));
               end loop;
            end;
         end loop;
      end;
      Ada.Text_IO.Put_Line ("Triangulation maximum nonidentity/inverse coordinate errors=" &
        OpenCV.Float64_Value'Image (Maximum_Forward) & OpenCV.Float64_Value'Image (Maximum_Reverse));
   end Triangulation_Frames;

   procedure Triangulation_Composition (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant Normalized_Image_Point_Array := Two_View_Points (False);
      Second : constant Normalized_Image_Point_Array := Two_View_Points (True);
      Estimate : constant Essential_Estimate := Estimate_Essential_RANSAC (First, Second, E_Policy);
      Selected_First, Selected_Second : Normalized_Image_Point_Array (1 .. Pose_Inlier_Count (Estimate));
   begin
      Assert (Found (Estimate) and then Pose_Recovered (Estimate), "composition recovered pose");
      for I in Selected_First'Range loop
         Selected_First (I) := First (Pose_Inlier (Estimate, I));
         Selected_Second (I) := Second (Pose_Inlier (Estimate, I));
      end loop;
      declare
         Points : constant Triangulated_Point_Array :=
           Triangulate_Normalized (Selected_First, Selected_Second, Recovered_Pose (Estimate));
         Angles : constant Stereo_Parallax_Array := Measure_Stereo_Parallax
           (Selected_First, Selected_Second, Recovered_Pose (Estimate));
         Quality : constant Triangulation_Quality_Array := Assess_Triangulation
           (Selected_First, Selected_Second, Recovered_Pose (Estimate), Points);
         Angular_Quality : constant Triangulation_Quality_Array := Assess_Triangulation
           (Selected_First, Selected_Second, Recovered_Pose (Estimate), Points,
            (Minimum_Acute_Parallax_Radians => 0.1, others => <>));
         Residual_Quality : constant Triangulation_Quality_Array := Assess_Triangulation
           (Selected_First, Selected_Second, Recovered_Pose (Estimate), Points,
            (Maximum_Normalized_Reprojection_Error => 1.0E-10, others => <>));
         Minimum_Angle : OpenCV.Float64_Value := OpenCV.Float64_Value'Last;
         Refined : constant Point_Refinement_Result_Array := Refine_Triangulated_Points
           (Selected_First, Selected_Second, Recovered_Pose (Estimate), Points);
         Snapshot : constant Triangulated_Point_Array := Points;
         Counts : array (Point_Refinement_Outcome) of Natural := [others => 0];
         Maximum_Angle, Maximum : OpenCV.Float64_Value := 0.0;
         Usable_Count, Default_Count, Angular_Count, Residual_Count : Natural := 0;
      begin
         Assert (Points'Length = 40 and then Quality'Length = 40 and then Angles'Length = 40,
           "composition forty selected/triangulated/assessed");
         Assert (Refined'Length = 40, "composition forty refined");
         for I in Refined'Range loop
            Counts (Refined (I).Outcome) := Counts (Refined (I).Outcome) + 1;
            Assert (not Refined (I).Residuals_Evaluated or else
              Refined (I).Final_Normalized_RMS <= Refined (I).Initial_Normalized_RMS, "composition nonworsening");
            if Refined (I).Outcome = Improved then
               Assert (Refined (I).Point.Status = Usable and then Refined (I).Point.First_Depth > 0.0 and then
                 Refined (I).Point.Second_Depth > 0.0 and then Refined (I).Point.First_Normalized_Error >= 0.0 and then
                 Refined (I).Point.First_Normalized_Error <= OpenCV.Float64_Value'Last and then
                 Refined (I).Point.Second_Normalized_Error >= 0.0 and then
                 Refined (I).Point.Second_Normalized_Error <= OpenCV.Float64_Value'Last, "composition updated diagnostics");
            else
               Assert (Refined (I).Point = Points (I), "composition correspondence preservation");
            end if;
         end loop;
         Assert (Points = Snapshot, "composition no initial mutation");
         for Outcome in Point_Refinement_Outcome loop
            Ada.Text_IO.Put_Line ("Refinement composition " & Point_Refinement_Outcome'Image (Outcome) & Natural'Image (Counts (Outcome)));
         end loop;
         for I in Quality'Range loop
            Assert (Quality (I).Original_Status = Points (I).Status and then
              Angles (I).Forward_Ray_Angle_Radians >= 0.0 and then
              Angles (I).Forward_Ray_Angle_Radians <= OpenCV.Float64_Value (Ada.Numerics.Pi) and then
              Angles (I).Acute_Line_Angle_Radians >= 0.0 and then
              Angles (I).Acute_Line_Angle_Radians <= OpenCV.Float64_Value (Ada.Numerics.Pi) / 2.0,
              "finite composition angles in range and status preserved");
            Minimum_Angle := OpenCV.Float64_Value'Min (Minimum_Angle, Angles (I).Acute_Line_Angle_Radians);
            Maximum_Angle := OpenCV.Float64_Value'Max (Maximum_Angle, Angles (I).Acute_Line_Angle_Radians);
            if Points (I).Status = Usable then Usable_Count := Usable_Count + 1; end if;
            if Quality (I).Accepted then Default_Count := Default_Count + 1; end if;
            if Angular_Quality (I).Accepted then Angular_Count := Angular_Count + 1; end if;
            if Residual_Quality (I).Accepted then Residual_Count := Residual_Count + 1; end if;
         end loop;
         Assert (Default_Count = Usable_Count, "defaults preserve triangulation validity");
         Ada.Text_IO.Put_Line ("Quality composition min/max acute=" &
           OpenCV.Float64_Value'Image (Minimum_Angle) & OpenCV.Float64_Value'Image (Maximum_Angle) &
           " usable/default/angle(0.1)/residual(1e-10)=" & Natural'Image (Usable_Count) &
           Natural'Image (Default_Count) & Natural'Image (Angular_Count) & Natural'Image (Residual_Count));
         Assert (Points'Length = Pose_Inlier_Count (Estimate), "composition preserves count");
         for P of Points loop
            Assert (P.Status = Usable, "clean composition usable");
            Assert (P.First_Depth > 0.0 and then P.Second_Depth > 0.0 and then
                    P.First_Normalized_Error >= 0.0 and then P.Second_Normalized_Error >= 0.0,
                    "composition depths and diagnostics");
            Maximum := OpenCV.Float64_Value'Max (Maximum,
              OpenCV.Float64_Value'Max (P.First_Normalized_Error, P.Second_Normalized_Error));
         end loop;
         Ada.Text_IO.Put_Line ("Triangulation composition selected/usable=" & Natural'Image (Points'Length) &
           " invalid=0 max normalized error=" & OpenCV.Float64_Value'Image (Maximum));
      end;
   end Triangulation_Composition;

   Stereo_Pose : constant Relative_Camera_Pose :=
     ([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [-1.0, 0.0, 0.0]);

   procedure Parallax_Rectified (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant Stereo_Parallax_Array := Measure_Stereo_Parallax
        ([5 => [0.4, 0.0]], [9 => [0.2, 0.0]], Stereo_Pose);
      Expected : constant OpenCV.Float64_Value := Math.Arctan (0.4) - Math.Arctan (0.2);
   begin
      Assert (A'First = 1 and then A'Length = 1, "one-based one-point parallax");
      Assert (Near (A (1).Forward_Ray_Angle_Radians, Expected, 1.0E-15) and then
        Near (A (1).Acute_Line_Angle_Radians, Expected, 1.0E-15), "independent atan difference");
      Ada.Text_IO.Put_Line ("Rectified angle/error=" & OpenCV.Float64_Value'Image (Expected) &
        OpenCV.Float64_Value'Image (abs (A (1).Forward_Ray_Angle_Radians - Expected)));
   end Parallax_Rectified;

   procedure Parallax_Tiny (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant Stereo_Parallax_Array := Measure_Stereo_Parallax
        ([[0.0, 0.0], [0.0, 0.0]], [[1.0E-12, 0.0], [0.0, 0.0]], Stereo_Pose);
   begin
      Assert (A (1).Forward_Ray_Angle_Radians > 0.0 and then
        Near (A (1).Forward_Ray_Angle_Radians, Math.Arctan (1.0E-12), 1.0E-27) and then
        Near (A (1).Acute_Line_Angle_Radians, Math.Arctan (1.0E-12), 1.0E-27), "tiny angle not acos zero");
      Assert (A (2).Forward_Ray_Angle_Radians = 0.0 and then A (2).Acute_Line_Angle_Radians = 0.0,
        "exact parallel zero");
      Ada.Text_IO.Put_Line ("Tiny angle=" & OpenCV.Float64_Value'Image (A (1).Forward_Ray_Angle_Radians));
   end Parallax_Tiny;

   procedure Parallax_Transpose (T : in out Fixture) is
      pragma Unreferenced (T);
      Pose : constant Relative_Camera_Pose :=
        ([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], [-1.0, 0.0, 0.0]);
      A : constant Stereo_Parallax_Array := Measure_Stereo_Parallax ([[1.0, 0.0]], [[0.0, 1.0]], Pose);
   begin
      Assert (Near (A (1).Forward_Ray_Angle_Radians, 0.0, 1.0E-15) and then
        Near (A (1).Acute_Line_Angle_Radians, 0.0, 1.0E-15), "R transpose quarter-turn");
   end Parallax_Transpose;

   procedure Parallax_Antiparallel (T : in out Fixture) is
      pragma Unreferenced (T);
      Pose : constant Relative_Camera_Pose :=
        ([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]], [-1.0, 0.0, 0.0]);
      A : constant Stereo_Parallax_Array := Measure_Stereo_Parallax ([[0.0, 0.0]], [[0.0, 0.0]], Pose);
   begin
      Assert (Near (A (1).Forward_Ray_Angle_Radians, OpenCV.Float64_Value (Ada.Numerics.Pi), 1.0E-15)
        and then A (1).Acute_Line_Angle_Radians = 0.0, "antiparallel pi / zero");
   end Parallax_Antiparallel;

   procedure Parallax_Frames (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant Normalized_Image_Point_Array := Two_View_Points (False);
      Second : constant Normalized_Image_Point_Array := Two_View_Points (True);
      Pose : constant Relative_Camera_Pose := (E_Rotation, E_Translation);
      Reverse_Pose, Scaled_Pose : Relative_Camera_Pose := Pose;
      A : constant Stereo_Parallax_Array := Measure_Stereo_Parallax (First, Second, Pose);
      Maximum : OpenCV.Float64_Value := 0.0;
   begin
      for I in 0 .. 2 loop
         Reverse_Pose.Translation_Direction (I) := 0.0;
         Scaled_Pose.Translation_Direction (I) := 7.0 * E_Translation (I);
         for J in 0 .. 2 loop
            Reverse_Pose.Rotation_First_To_Second (I, J) := E_Rotation (J, I);
            Reverse_Pose.Translation_Direction (I) := Reverse_Pose.Translation_Direction (I) -
              E_Rotation (J, I) * E_Translation (J) / E_Norm;
         end loop;
      end loop;
      declare
         B : constant Stereo_Parallax_Array := Measure_Stereo_Parallax (Second, First, Reverse_Pose);
         C : constant Stereo_Parallax_Array := Measure_Stereo_Parallax (First, Second, Scaled_Pose);
      begin
         for I in A'Range loop
            Maximum := OpenCV.Float64_Value'Max (Maximum,
              abs (A (I).Forward_Ray_Angle_Radians - B (I).Forward_Ray_Angle_Radians));
            Assert (Near (A (I).Forward_Ray_Angle_Radians, B (I).Forward_Ray_Angle_Radians, 1.0E-15)
              and then Near (A (I).Acute_Line_Angle_Radians, B (I).Acute_Line_Angle_Radians, 1.0E-15),
              "inverse-frame symmetry");
            Assert (A (I) = C (I), "translation magnitude invariance");
         end loop;
      end;
      Ada.Text_IO.Put_Line ("Parallax inverse maximum error=" & OpenCV.Float64_Value'Image (Maximum));
   end Parallax_Frames;

   package Refinement_Kernel renames OpenCV.Calib3D.Internal.Point_Refinement;

   function Independent_Projection (X : Object_Point; P : Relative_Camera_Pose)
      return Refinement_Kernel.Residual_Vector
   is
      Q : Object_Point := [others => 0.0];
      Norm : constant OpenCV.Float64_Value := Math.Sqrt
        (P.Translation_Direction (0) ** 2 + P.Translation_Direction (1) ** 2 +
         P.Translation_Direction (2) ** 2);
   begin
      for I in 0 .. 2 loop
         Q (I) := P.Translation_Direction (I) / Norm;
         for J in 0 .. 2 loop
            Q (I) := Q (I) + P.Rotation_First_To_Second (I, J) * X (J);
         end loop;
      end loop;
      return [X (0) / X (2), X (1) / X (2), Q (0) / Q (2), Q (1) / Q (2)];
   end Independent_Projection;

   function Independent_RMS (X : Object_Point; P : Relative_Camera_Pose;
                             A, B : Normalized_Image_Point) return OpenCV.Float64_Value is
      V : constant Refinement_Kernel.Residual_Vector := Independent_Projection (X, P);
   begin
      --  Bounded fixtures only: this independent oracle may safely square.
      return Math.Sqrt ((V (0) - A (0)) ** 2 + (V (1) - A (1)) ** 2 +
                        (V (2) - B (0)) ** 2 + (V (3) - B (1)) ** 2) / 2.0;
   end Independent_RMS;

   procedure Refinement_Jacobian (T : in out Fixture) is
      pragma Unreferenced (T);
      P : Relative_Camera_Pose;
      X : constant Object_Point := [2.0, 1.0, 5.0];
      Plus, Minus : Object_Point;
      V : Refinement_Kernel.Evaluation;
      Fp, Fm : Refinement_Kernel.Residual_Vector;
      H : constant OpenCV.Float64_Value := 1.0E-5;
      Maximum : OpenCV.Float64_Value := 0.0;
   begin
      for Fixture_Index in 1 .. 2 loop
         P := (if Fixture_Index = 1 then Stereo_Pose else
           (E_Rotation, [E_Translation (0) / E_Norm, E_Translation (1) / E_Norm, E_Translation (2) / E_Norm]));
         Assert (Refinement_Kernel.Evaluate (X, [0.4, 0.2], [0.2, 0.2], P, V), "Jacobian evaluated");
         for Col in 0 .. 2 loop
            Plus := X; Minus := X;
            Plus (Col) := Plus (Col) + H;
            Minus (Col) := Minus (Col) - H;
            Fp := Independent_Projection (Plus, P);
            Fm := Independent_Projection (Minus, P);
            for Row in 0 .. 3 loop
               Maximum := OpenCV.Float64_Value'Max (Maximum,
                 abs (V.Jacobian (Row, Col) - (Fp (Row) - Fm (Row)) / (2.0 * H)));
               Assert (Near (V.Jacobian (Row, Col), (Fp (Row) - Fm (Row)) / (2.0 * H), 1.0E-9),
                 "independent central difference component");
            end loop;
         end loop;
      end loop;
      Ada.Text_IO.Put_Line ("Refinement Jacobian maximum absolute error=" & OpenCV.Float64_Value'Image (Maximum));
   end Refinement_Jacobian;

   procedure Refinement_Solver (T : in out Fixture) is
      pragma Unreferenced (T);
      V : Refinement_Kernel.Evaluation := (Residual => [0.1, -0.2, 0.3, 0.0],
        Jacobian => [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0], [0.0, 0.0, 0.0]],
        others => 0.0);
      Update : Object_Point;
      E : Refinement_Kernel.Evaluation;
      Product : OpenCV.Float64_Value;
   begin
      Assert (Refinement_Kernel.Step (V, 0.001, Update), "SPD damped solve");
      for I in 0 .. 2 loop
         Assert (Near (1.001 * Update (I) * 2.0, -V.Residual (I), 1.0E-14), "scaled normal residual and negative gradient");
      end loop;
      V.Jacobian := [[1.0, 0.2, -0.3], [0.1, 1.0, 0.4], [0.3, -0.2, 1.0], [0.7, 0.2, -0.5]];
      Assert (Refinement_Kernel.Step (V, 0.001, Update), "coupled SPD solve");
      for I in 0 .. 2 loop
         Product := 0.001 * Update (I);
         for Row in 0 .. 3 loop
            Product := Product + V.Jacobian (Row, I) * V.Residual (Row);
            for Col in 0 .. 2 loop
               Product := Product + V.Jacobian (Row, I) * V.Jacobian (Row, Col) * Update (Col);
            end loop;
         end loop;
         Assert (abs Product < 1.0E-13, "independent coupled damped normal residual");
      end loop;
      V.Jacobian := [others => [others => 0.0]];
      Assert (not Refinement_Kernel.Step (V, 0.001, Update), "zero Jacobian not zero solve");
      Assert (Update (0) = 0.0, "failure output initialized, not an accepted solve");
      Assert (not Refinement_Kernel.Evaluate ([1.0, 1.0, -1.0], [0.0, 0.0], [0.0, 0.0], Stereo_Pose, E),
        "nonpositive trial rejected");
      Assert (E.Norm = 0.0, "failed evaluation not consumed");
      Assert (not Refinement_Kernel.Evaluate ([OpenCV.Float64_Value'Last, 0.0, 1.0E-300],
        [0.0, 0.0], [0.0, 0.0], Stereo_Pose, E), "overflow trial rejected");
      Assert (E.Norm = 0.0, "overflow evaluation not consumed");
   end Refinement_Solver;

   procedure Refinement_Geometry (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant Normalized_Image_Point_Array (4 .. 4) := [[0.4, 0.2]];
      B : Normalized_Image_Point_Array (8 .. 8);
      P, Saved : Relative_Camera_Pose;
      Projection : Refinement_Kernel.Residual_Vector;
   begin
      for Fixture_Index in 1 .. 3 loop
         P := (if Fixture_Index < 3 then Stereo_Pose else (E_Rotation, E_Translation));
         Saved := P;
         Projection := Independent_Projection ([2.0, 1.0, 5.0], P);
         B (8) := [Projection (2), Projection (3)];
         if Fixture_Index > 1 then
            B (8) (0) := B (8) (0) + 0.025;
            B (8) (1) := B (8) (1) - 0.04;
         end if;
         declare
            Initial : constant Triangulated_Point_Array := Triangulate_Normalized (A, B, P);
            Copy : constant Triangulated_Point_Array := Initial;
            Results : constant Point_Refinement_Result_Array := Refine_Triangulated_Points (A, B, P, Initial);
            R : Point_Refinement_Result renames Results (1);
            Before : constant OpenCV.Float64_Value := Independent_RMS (Initial (1).Position_In_First_Camera, P, A (4), B (8));
            After : constant OpenCV.Float64_Value := Independent_RMS (R.Point.Position_In_First_Camera, P, A (4), B (8));
            Previous : OpenCV.Float64_Value := R.Initial_Normalized_RMS;
         begin
            Assert (Initial = Copy and then P = Saved, "input points and pose unchanged");
            Assert (R.Point.Status = Usable and then R.Point.First_Depth > 0.0 and then R.Point.Second_Depth > 0.0,
              "positive usable geometry");
            Assert (R.Residuals_Evaluated and then After <= Before and then R.Final_Normalized_RMS <= R.Initial_Normalized_RMS,
              "independent nonworsening objective");
            if Fixture_Index > 1 then
               Assert (R.Outcome = Improved and then R.Accepted_Steps > 0 and then After < Before, "genuine DLT noisy improvement");
            end if;
            if R.Accepted_Steps = 0 then Assert (R.Point = Initial (1), "no drift without acceptance"); end if;
            Projection := Independent_Projection (R.Point.Position_In_First_Camera, P);
            if R.Outcome = Improved then
               Assert (Near (R.Point.First_Normalized_Error, Math.Sqrt
                 ((Projection (0) - A (4) (0)) ** 2 + (Projection (1) - A (4) (1)) ** 2), 1.0E-12) and then
                 Near (R.Point.Second_Normalized_Error, Math.Sqrt
                 ((Projection (2) - B (8) (0)) ** 2 + (Projection (3) - B (8) (1)) ** 2), 1.0E-12), "fixed-pose independent updated errors");
            end if;
            for Budget in 1 .. 20 loop
               declare
                  Bounded : constant Point_Refinement_Result_Array := Refine_Triangulated_Points
                    (A, B, P, Initial, (Maximum_Iterations => Budget, others => <>));
                  Objective : constant OpenCV.Float64_Value := Bounded (1).Final_Normalized_RMS;
               begin
                  Assert (Bounded (1).Accepted_Steps <= Budget and then Objective <= Previous, "bounded deterministic prefix monotonicity");
                  if Bounded (1).Accepted_Steps > Budget - 1 then
                     Assert (Objective < Previous, "every accepted prefix strictly improves");
                  end if;
                  Previous := Objective;
               end;
            end loop;
            Ada.Text_IO.Put_Line ("Refinement fixture" & Integer'Image (Fixture_Index) & " " & Point_Refinement_Outcome'Image (R.Outcome) &
              " steps=" & Natural'Image (R.Accepted_Steps) & " independent RMS before/after=" &
              OpenCV.Float64_Value'Image (Before) & OpenCV.Float64_Value'Image (After));
         end;
      end loop;
   end Refinement_Geometry;

   procedure Refinement_Batch (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant Normalized_Image_Point_Array (4 .. 10) :=
        [[0.4, 0.2], [0.4, 0.2], [0.0, 0.0], others => [0.0, 0.0]];
      B : constant Normalized_Image_Point_Array (12 .. 18) :=
        [[0.2, 0.2], [0.225, 0.16], [-1.0E-8, 0.0], others => [0.0, 0.0]];
      Seed : constant Triangulated_Point_Array := Triangulate_Normalized (A, B, Stereo_Pose);
      Initial : Triangulated_Point_Array (20 .. 26);
   begin
      Initial (20 .. 22) := Seed (1 .. 3);
      Initial (23) := (Status => At_Infinity);
      Initial (24) := (Status => Unrepresentable_Point);
      Initial (25) := (Status => Non_Positive_Depth);
      Initial (26) := (Status => Undefined_Reprojection);
      declare
         Copy : constant Triangulated_Point_Array := Initial;
         Results : constant Point_Refinement_Result_Array := Refine_Triangulated_Points
           (A, B, Stereo_Pose, Initial, (Minimum_Acute_Parallax_Radians => 1.0E-6, others => <>));
         Default : constant Point_Refinement_Result_Array := Refine_Triangulated_Points (A, B, Stereo_Pose, Initial);
      begin
         Assert (Results'First = 1 and then Results'Length = 7 and then Initial = Copy, "mixed bounds ordering and immutable input");
         Assert (Results (2).Outcome = Improved, "noisy point improves independently of difficult entries");
         Assert (Seed (3).Status = Usable and then Results (3).Outcome = Skipped_Low_Parallax and then
           Results (3).Point = Initial (22) and then Results (3).Accepted_Steps = 0 and then
           not Results (3).Residuals_Evaluated, "caller low-parallax screening preserves usable value");
         Assert (Default (3).Outcome /= Skipped_Low_Parallax, "zero minimum no hidden rejection");
         for I in 4 .. 7 loop
            Assert (Results (I).Outcome = Skipped_Unusable and then Results (I).Point = Initial (19 + I) and then
              Results (I).Accepted_Steps = 0 and then not Results (I).Residuals_Evaluated and then
              Results (I).Initial_Normalized_RMS = 0.0 and then Results (I).Final_Normalized_RMS = 0.0,
              "nonusable exact preservation and no absent-field access");
         end loop;
         Ada.Text_IO.Put_Line ("Refinement low-parallax screened/default=" & Point_Refinement_Outcome'Image (Results (3).Outcome) &
           " / " & Point_Refinement_Outcome'Image (Default (3).Outcome));
      end;
   end Refinement_Batch;

   procedure Point_Refinement_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check); --  Deliberately inject IEEE nonfinite arguments.
      Empty : constant Normalized_Image_Point_Array (1 .. 0) := [];
      A : constant Normalized_Image_Point_Array := [[0.4, 0.2]];
      B : constant Normalized_Image_Point_Array := [[0.2, 0.2]];
      Good : constant Triangulated_Point_Array := Triangulate_Normalized (A, B, Stereo_Pose);
      P : Relative_Camera_Pose;
      Bad : Triangulated_Point_Array := Good;
      Obs : Normalized_Image_Point_Array := A;
      procedure Reject (First, Second : Normalized_Image_Point_Array;
                        Pose : Relative_Camera_Pose; Points : Triangulated_Point_Array;
                        Options : Triangulation_Refinement_Options := (others => <>)) is
      begin
         declare
            R : constant Point_Refinement_Result_Array := Refine_Triangulated_Points (First, Second, Pose, Points, Options);
         begin
            Assert (R'Length = Natural'Last, "expected malformed refinement rejection");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Reject;
   begin
      Assert (Refine_Triangulated_Points (Empty, Empty, Stereo_Pose, [1 .. 0 => <>])'Length = 0, "validated empty refinement");
      declare
         Exact : constant Triangulated_Point_Array := [(Usable, [2.0, 1.0, 5.0], 9.0, 8.0, 7.0, 6.0)];
         R : constant Point_Refinement_Result_Array := Refine_Triangulated_Points (A, B, Stereo_Pose, Exact);
      begin
         Assert (R (1).Outcome = No_Improving_Step and then R (1).Point = Exact (1) and then
           R (1).Accepted_Steps = 0 and then R (1).Residuals_Evaluated and then
           R (1).Initial_Normalized_RMS = 0.0 and then R (1).Final_Normalized_RMS = 0.0,
           "exact zero stops without drift; reported diagnostics are not objective");
      end;
      Reject (A, Empty, Stereo_Pose, Good);
      Reject (A, B, Stereo_Pose, [1 .. 0 => <>]);
      Reject (Empty, Empty, Stereo_Pose, [1 .. 0 => <>], (Maximum_Iterations => 101, others => <>));
      Reject (A, B, Stereo_Pose, Good, (Minimum_Acute_Parallax_Radians => -0.01, others => <>));
      Reject (A, B, Stereo_Pose, Good, (Minimum_Acute_Parallax_Radians => 2.0, others => <>));
      for Kind in 0 .. 2 loop
         declare
            V : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Nonfinite (Interfaces.Integer_32 (Kind)));
         begin
            Reject (A, B, Stereo_Pose, Good, (Minimum_Acute_Parallax_Radians => V, others => <>));
            Obs := A; Obs (1) (0) := V; Reject (Obs, B, Stereo_Pose, Good);
            Obs := B; Obs (1) (1) := V; Reject (A, Obs, Stereo_Pose, Good);
            P := Stereo_Pose; P.Translation_Direction (0) := V; Reject (Empty, Empty, P, [1 .. 0 => <>]);
            P := Stereo_Pose; P.Rotation_First_To_Second (0, 0) := V; Reject (A, B, P, Good);
            Bad := Good; Bad (1).Position_In_First_Camera (0) := V; Reject (A, B, Stereo_Pose, Bad);
            Bad := Good; Bad (1).First_Depth := V; Reject (A, B, Stereo_Pose, Bad);
            Bad := Good; Bad (1).Second_Depth := V; Reject (A, B, Stereo_Pose, Bad);
            Bad := Good; Bad (1).First_Normalized_Error := V; Reject (A, B, Stereo_Pose, Bad);
            Bad := Good; Bad (1).Second_Normalized_Error := V; Reject (A, B, Stereo_Pose, Bad);
         end;
      end loop;
      P := Stereo_Pose; P.Translation_Direction := [others => 0.0]; Reject (Empty, Empty, P, [1 .. 0 => <>]);
      P := Stereo_Pose; P.Rotation_First_To_Second (0, 0) := 1.01; Reject (A, B, P, Good);
      P := Stereo_Pose; P.Rotation_First_To_Second (0, 0) := -1.0; Reject (Empty, Empty, P, [1 .. 0 => <>]);
      Bad := Good; Bad (1).Position_In_First_Camera (2) := -1.0; Reject (A, B, Stereo_Pose, Bad);
      P := Stereo_Pose; P.Rotation_First_To_Second := [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]];
      Reject (A, B, P, Good);
      Bad := Good; Bad (1).First_Depth := 0.0; Reject (A, B, Stereo_Pose, Bad);
      Bad := Good; Bad (1).Second_Depth := -1.0; Reject (A, B, Stereo_Pose, Bad);
      Bad := Good; Bad (1).First_Normalized_Error := -1.0; Reject (A, B, Stereo_Pose, Bad);
      Bad := Good; Bad (1).Second_Normalized_Error := -1.0; Reject (A, B, Stereo_Pose, Bad);
   end Point_Refinement_Invalid;

   procedure Refinement_Extremes (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant Normalized_Image_Point_Array := [[0.0, 0.0]];
      Initial : Triangulated_Point_Array := [(Usable, [0.0, 0.0, 1.0], 1.0, 1.0, 0.0, 0.0)];
      Tiny : OpenCV.Float64_Value := OpenCV.Float64_Value'Model_Small;
   begin
      Tiny := Tiny / 100.0;
      for Fixture_Index in 1 .. 3 loop
         Initial (1).Position_In_First_Camera :=
           (case Fixture_Index is
             when 1 => [OpenCV.Float64_Value'Last, 0.0, 1.0E-300],
             when 2 => [0.0, 0.0, Tiny],
             when others => [0.0, 0.0, OpenCV.Float64_Value'Last]);
         declare
            R : constant Point_Refinement_Result_Array := Refine_Triangulated_Points (A, A, Stereo_Pose, Initial);
         begin
            Ada.Text_IO.Put_Line ("Refinement extreme" & Integer'Image (Fixture_Index) & " " & Point_Refinement_Outcome'Image (R (1).Outcome));
            Assert (R (1).Outcome /= Skipped_Low_Parallax and then R (1).Outcome /= Skipped_Unusable,
              "difficult finite geometry attempted without constraint error or coordinate clamps");
            if R (1).Outcome = Improved then
               Assert (R (1).Final_Normalized_RMS < R (1).Initial_Normalized_RMS, "extreme improvement strictly decreases");
            else
               Assert (R (1).Point = Initial (1) and then R (1).Accepted_Steps = 0, "extreme original preserved");
            end if;
         end;
      end loop;
   end Refinement_Extremes;

   procedure Quality_Boundaries (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant Normalized_Image_Point_Array := [5 => [0.4, 0.0]];
      Second : constant Normalized_Image_Point_Array := [9 => [0.2, 0.0]];
      Points : constant Triangulated_Point_Array (12 .. 12) :=
        [12 => (Usable, [2.0, 0.0, 5.0], 5.0, 5.0, 0.01, 0.02)];
      Alpha : constant OpenCV.Float64_Value :=
        Measure_Stereo_Parallax (First, Second, Stereo_Pose) (1).Acute_Line_Angle_Radians;
   begin
      for Minimum of Reprojection_Error_Array'[0.0, Alpha / 2.0, Alpha, Alpha * 2.0] loop
         for Maximum of Reprojection_Error_Array'[0.01, 0.02, 0.03] loop
            declare
               Q : constant Triangulation_Quality_Array := Assess_Triangulation
                 (First, Second, Stereo_Pose, Points, (Minimum, Maximum));
            begin
               Assert (Q'First = 1 and then Q (1).Original_Status = Usable and then Points (12).Status = Usable,
                 "quality preserves status and iteration bounds");
               Assert (Q (1).Passes_Parallax_Limit = (Minimum <= Alpha) and then
                 Q (1).Passes_Reprojection_Limit = (Maximum >= 0.02) and then
                 Q (1).Accepted = (Minimum <= Alpha and then Maximum >= 0.02), "inclusive combined policy flags");
            end;
         end loop;
      end loop;
      declare
         Q : constant Triangulation_Quality_Array := Assess_Triangulation
           (First, Second, Stereo_Pose, [7 => (Status => Non_Positive_Depth)]);
      begin
         Assert (not Q (1).Accepted and then not Q (1).Passes_Reprojection_Limit and then
           Q (1).Parallax.Acute_Line_Angle_Radians > 0.0, "nonusable keeps parallax");
      end;
      Ada.Text_IO.Put_Line ("Quality below/equal/above angle and residual boundaries, all combined flags PASS");
   end Quality_Boundaries;

   procedure Quality_Low_Parallax (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant Normalized_Image_Point_Array := [[0.0, 0.0]];
      Second : constant Normalized_Image_Point_Array := [[-1.0E-8, 0.0]];
      Points : constant Triangulated_Point_Array := Triangulate_Normalized (First, Second, Stereo_Pose);
      Q : constant Triangulation_Quality_Array := Assess_Triangulation
        (First, Second, Stereo_Pose, Points, (Minimum_Acute_Parallax_Radians => 1.0E-6, others => <>));
   begin
      Assert (Points (1).Status = Usable, "finite small disparity reconstruction usable");
      Assert (not Q (1).Accepted and then not Q (1).Passes_Parallax_Limit and then
        Q (1).Passes_Reprojection_Limit, "usable is not well conditioned");
      Assert (Near (Q (1).Parallax.Acute_Line_Angle_Radians, Math.Arctan (1.0E-8), 1.0E-23),
        "independent disparity angle");
      Ada.Text_IO.Put_Line ("Low-parallax native Usable / quality rejected angle=" &
        OpenCV.Float64_Value'Image (Q (1).Parallax.Acute_Line_Angle_Radians));
   end Quality_Low_Parallax;

   procedure Quality_Noisy (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant Normalized_Image_Point_Array := [[0.4, 0.2]];
      Second : constant Normalized_Image_Point_Array := [[0.2, 0.23]];
      Points : constant Triangulated_Point_Array := Triangulate_Normalized (First, Second, Stereo_Pose);
      Maximum : constant OpenCV.Float64_Value := OpenCV.Float64_Value'Max
        (Points (1).First_Normalized_Error, Points (1).Second_Normalized_Error);
   begin
      for Limit of Reprojection_Error_Array'[Maximum / 2.0, Maximum, Maximum * 2.0] loop
         declare
            Q : constant Triangulation_Quality_Array := Assess_Triangulation
              (First, Second, Stereo_Pose, Points, (Maximum_Normalized_Reprojection_Error => Limit, others => <>));
         begin
            Assert (Points (1).Status = Usable and then Q (1).Accepted = (Limit >= Maximum),
              "native noisy residual inclusive threshold / unchanged usable");
         end;
      end loop;
      Ada.Text_IO.Put_Line ("Noisy quality below/equal/above maximum=" & OpenCV.Float64_Value'Image (Maximum));
   end Quality_Noisy;

   procedure Quality_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      --  Allow deliberately malformed IEEE fixtures to reach API validation.
      pragma Suppress (Validity_Check);
      Empty : constant Normalized_Image_Point_Array := [1 .. 0 => <>];
      One : constant Normalized_Image_Point_Array := [[0.0, 0.0]];
      Good : constant Triangulated_Point := (Usable, [0.0, 0.0, 1.0], 1.0, 1.0, 0.0, 0.0);
      procedure Reject (P : Relative_Camera_Pose; A, B : Normalized_Image_Point_Array;
                        Points : Triangulated_Point_Array; O : Triangulation_Quality_Options := (others => <>)) is
      begin
         declare
            Q : constant Triangulation_Quality_Array := Assess_Triangulation (A, B, P, Points, O);
         begin
            Assert (False, "invalid assessment accepted" & Natural'Image (Q'Length));
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Reject;
      P : Relative_Camera_Pose;
      Bad : Triangulated_Point := Good;
      A : Normalized_Image_Point_Array := One;
   begin
      Assert (Measure_Stereo_Parallax (Empty, Empty, Stereo_Pose)'Length = 0 and then
        Assess_Triangulation (Empty, Empty, Stereo_Pose, [1 .. 0 => <>])'Length = 0, "validated empty diagnostics");
      Reject (Stereo_Pose, One, Empty, [Good]);
      Reject (Stereo_Pose, One, One, [1 .. 0 => <>]);
      P := Stereo_Pose; P.Rotation_First_To_Second (0, 0) := 2.0;
      Reject (P, Empty, Empty, [1 .. 0 => <>]);
      P := Stereo_Pose; P.Rotation_First_To_Second (0, 0) := -1.0;
      Reject (P, Empty, Empty, [1 .. 0 => <>]);
      P := Stereo_Pose; P.Translation_Direction := [0.0, 0.0, 0.0];
      Reject (P, Empty, Empty, [1 .. 0 => <>]);
      for Kind in 0 .. 2 loop
         declare
            V : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Nonfinite (Interfaces.Integer_32 (Kind)));
         begin
            P := Stereo_Pose; P.Translation_Direction (0) := V; Reject (P, One, One, [Good]);
            P := Stereo_Pose; P.Rotation_First_To_Second (0, 0) := V; Reject (P, One, One, [Good]);
            A := One; A (1) (0) := V; Reject (Stereo_Pose, A, One, [Good]);
            Reject (Stereo_Pose, One, A, [Good]);
            Reject (Stereo_Pose, Empty, Empty, [1 .. 0 => <>], (V, 0.0));
            Reject (Stereo_Pose, Empty, Empty, [1 .. 0 => <>], (0.0, V));
            Bad := Good; Bad.Position_In_First_Camera (1) := V; Reject (Stereo_Pose, One, One, [Bad]);
            Bad := Good; Bad.First_Depth := V; Reject (Stereo_Pose, One, One, [Bad]);
            Bad := Good; Bad.Second_Depth := V; Reject (Stereo_Pose, One, One, [Bad]);
            Bad := Good; Bad.First_Normalized_Error := V; Reject (Stereo_Pose, One, One, [Bad]);
            Bad := Good; Bad.Second_Normalized_Error := V; Reject (Stereo_Pose, One, One, [Bad]);
         end;
      end loop;
      Reject (Stereo_Pose, Empty, Empty, [1 .. 0 => <>], (-1.0, 0.0));
      Reject (Stereo_Pose, Empty, Empty, [1 .. 0 => <>], (2.0, 0.0));
      Reject (Stereo_Pose, Empty, Empty, [1 .. 0 => <>], (0.0, -1.0));
      for V of Reprojection_Error_Array'[0.0, -1.0] loop
         Bad := Good; Bad.First_Depth := V; Reject (Stereo_Pose, One, One, [Bad]);
         Bad := Good; Bad.Second_Depth := V; Reject (Stereo_Pose, One, One, [Bad]);
      end loop;
      Bad := Good; Bad.First_Normalized_Error := -1.0; Reject (Stereo_Pose, One, One, [Bad]);
      Bad := Good; Bad.Second_Normalized_Error := -1.0; Reject (Stereo_Pose, One, One, [Bad]);
   end Quality_Invalid;

   procedure Parallax_Large (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant Stereo_Parallax_Array := Measure_Stereo_Parallax
        ([[OpenCV.Float64_Value'Last, OpenCV.Float64_Value'Last]],
         [[-OpenCV.Float64_Value'Last, OpenCV.Float64_Value'Last]], Stereo_Pose);
   begin
      Assert (Near (A (1).Forward_Ray_Angle_Radians, OpenCV.Float64_Value (Ada.Numerics.Pi) / 2.0, 1.0E-15),
        "extreme finite bearings do not square unscaled coordinates");
   end Parallax_Large;

   function Visibility_Candidate (Normal : Camera_Direction := [0.0, 0.0, 1.0])
     return Planar_Motion_Hypothesis is
     ([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
      [0.1, 0.0, 0.0], Normal, False);

   function Visibility_Oracle
     (V : Planar_Motion_Hypothesis; A, B : Normalized_Image_Point_Array;
      Selected : Inlier_Index_Array) return Boolean
   is
      RN : Camera_Direction := [others => 0.0];
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            RN (I) := RN (I) + V.Rotation_First_To_Second (I, J) * V.Plane_Normal_In_First (J);
         end loop;
      end loop;
      for Index of Selected loop
         declare
            P : constant Normalized_Image_Point := A (A'First + Index - 1);
            Q : constant Normalized_Image_Point := B (B'First + Index - 1);
            X1 : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Interfaces.C.C_float (P (0)));
            Y1 : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Interfaces.C.C_float (P (1)));
            X2 : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Interfaces.C.C_float (Q (0)));
            Y2 : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Interfaces.C.C_float (Q (1)));
            First : constant OpenCV.Float64_Value := V.Plane_Normal_In_First (0) * X1 +
              V.Plane_Normal_In_First (1) * Y1 + V.Plane_Normal_In_First (2);
            Second : constant OpenCV.Float64_Value := RN (0) * X2 + RN (1) * Y2 + RN (2);
         begin
            if First <= 0.0 or else Second <= 0.0 then
               return False;
            end if;
         end;
      end loop;
      return True;
   end Visibility_Oracle;

   procedure Visibility_Signs (T : in out Fixture) is
      pragma Unreferenced (T);
      Positive_Normal : constant Planar_Motion_Hypothesis := Visibility_Candidate;
      Negative_Normal : Planar_Motion_Hypothesis := Visibility_Candidate ([0.0, 0.0, -1.0]);
   begin
      Negative_Normal.Translation_Over_Plane_Distance := [-0.1, 0.0, 0.0];
      declare
         Results : constant Planar_Hypothesis_Visibility_Array := Assess_Planar_Visibility
           ([Positive_Normal, Negative_Normal], [[0.2, 0.1]], [[0.25, 0.1]]);
      begin
         Assert (Results (1).Disposition = Passes_Visibility and then
           Results (2).Disposition = Rejected_By_Visibility, "independent +/-1 normal signs");
         Assert (Visibility_Passing_Hypothesis_Indices (Results) = [1], "passing positions only");
         Ada.Text_IO.Put_Line ("Visibility normal-sign oracle: (+1,+1) passes; (-1,-1) rejects; survivors [1]");
      end;
   end Visibility_Signs;

   procedure Visibility_Zero (T : in out Fixture) is
      pragma Unreferenced (T);
      V : constant Planar_Motion_Hypothesis := Visibility_Candidate ([1.0, 0.0, 0.0]);
   begin
      for X of Translation_Vector'([0.0, 0.25, -0.25]) loop
         declare
            Results : constant Planar_Hypothesis_Visibility_Array :=
              Assess_Planar_Visibility ([V], [[X, 0.0]], [[0.25, 0.0]]);
         begin
            Assert ((Results (1).Disposition = Passes_Visibility) = (X > 0.0), "strict >0, no epsilon");
         end;
      end loop;
      Ada.Text_IO.Put_Line ("Visibility strict boundary: first=0/.25/-.25 second=.25; rejected/passes/rejected");
   end Visibility_Zero;

   procedure Visibility_Frame (T : in out Fixture) is
      pragma Unreferenced (T);
      V : Planar_Motion_Hypothesis := Visibility_Candidate ([0.6, 0.0, 0.8]);
      C : constant OpenCV.Float64_Value := Math.Cos (0.2);
      S : constant OpenCV.Float64_Value := Math.Sin (0.2);
      A : constant Normalized_Image_Point_Array := [[0.0, 0.0]];
      B : constant Normalized_Image_Point_Array := [[-1.2, 0.0]];
      Correct : constant OpenCV.Float64_Value := (C * 0.6 + S * 0.8) * (-1.2) - S * 0.6 + C * 0.8;
      Inverted : constant OpenCV.Float64_Value := (C * 0.6 - S * 0.8) * (-1.2) + S * 0.6 + C * 0.8;
   begin
      V.Rotation_First_To_Second := [[C, 0.0, S], [0.0, 1.0, 0.0], [-S, 0.0, C]];
      Assert (Correct < 0.0 and then Inverted > 0.0, "frame fixture discriminates transpose");
      declare
         Results : constant Planar_Hypothesis_Visibility_Array := Assess_Planar_Visibility ([V], A, B);
      begin
         Assert (Results (1).Disposition = Rejected_By_Visibility and then
           not Visibility_Oracle (V, A, B, [1]), "native uses R*n not R^T*n");
      end;
      Ada.Text_IO.Put_Line ("Visibility frame: first=.8 R*n second=" & OpenCV.Float64_Value'Image (Correct) &
        " R^T*n second=" & OpenCV.Float64_Value'Image (Inverted));
   end Visibility_Frame;

   procedure Visibility_Composition (T : in out Fixture) is
      pragma Unreferenced (T);
      C : constant OpenCV.Float64_Value := Math.Cos (0.08);
      S : constant OpenCV.Float64_Value := Math.Sin (0.08);
      H : constant Homography_Matrix := [[C, 0.0, S - 0.12], [0.0, 1.0, 0.04], [-S, 0.0, C + 0.08]];
      V : constant Planar_Motion_Hypothesis_Array := Decompose_Calibrated_Homography (H, (1.0, 1.0, 0.0, 0.0));
      A, B : Normalized_Image_Point_Array (1 .. 12);
      Selected : Inlier_Index_Array (1 .. 12);
      Known : Boolean := False;
   begin
      Assert (V'Length = 4, "four general decomposition candidates");
      for I in A'Range loop
         A (I) := [OpenCV.Float64_Value (I mod 4) * 0.1, OpenCV.Float64_Value (I / 4) * 0.1];
         --  Independent plane Z=1, X_second=R*X_first+t/d.
         declare
            Z : constant OpenCV.Float64_Value := -S * A (I) (0) + C + 0.08;
         begin
            B (I) := [(C * A (I) (0) + S - 0.12) / Z, (A (I) (1) + 0.04) / Z];
         end;
         Selected (I) := I;
      end loop;
      declare
         Results : constant Planar_Hypothesis_Visibility_Array := Assess_Planar_Visibility (V, A, B);
      begin
         Assert (Results'Length = V'Length, "preserve all hypotheses");
         for I in V'Range loop
            Assert ((Results (I).Disposition = Passes_Visibility) = Visibility_Oracle (V (I), A, B, Selected),
              "decomposition composition Float32 independent signs");
            if abs (V (I).Plane_Normal_In_First (2) - 1.0) < 1.0E-8 then
               Known := True;
               Assert (Results (I).Disposition = Passes_Visibility, "known consistent candidate passes");
            elsif abs (V (I).Plane_Normal_In_First (2) + 1.0) < 1.0E-8 then
               Assert (Results (I).Disposition = Rejected_By_Visibility, "sign opposite rejects");
            end if;
            Ada.Text_IO.Put_Line ("Visibility composition candidate" & Positive'Image (I) & " " &
              Planar_Visibility_Disposition'Image (Results (I).Disposition));
         end loop;
         Assert (Known and then Visibility_Passing_Hypothesis_Indices (Results)'Length > 1,
           "known candidate, remaining multiple-motion ambiguity");
      end;
   end Visibility_Composition;

   procedure Visibility_Selected (T : in out Fixture) is
      pragma Unreferenced (T);
      A, B : Image_Point_Array (1 .. 24);
      N1, N2 : Normalized_Image_Point_Array (1 .. 24);
      V : constant Planar_Motion_Hypothesis := Visibility_Candidate ([1.0, 0.0, 0.0]);
      C : constant OpenCV.Float64_Value := Math.Cos (0.08);
      S : constant OpenCV.Float64_Value := Math.Sin (0.08);
      H : constant Homography_Matrix := [[C, 0.0, S - 0.12], [0.0, 1.0, 0.04], [-S, 0.0, C + 0.08]];
      Hypotheses : constant Planar_Motion_Hypothesis_Array :=
        Decompose_Calibrated_Homography (H, (1.0, 1.0, 0.0, 0.0));
   begin
      for I in A'Range loop
         A (I) := [OpenCV.Float64_Value (I mod 6) * 40.0 + 50.0,
                   OpenCV.Float64_Value (I / 6) * 30.0 + 10.0];
         --  Same Z=1, known R/t/d/n fixture as decomposition composition.
         declare
            X : constant OpenCV.Float64_Value := A (I) (0) / 800.0;
            Y : constant OpenCV.Float64_Value := A (I) (1) / 820.0;
            Z : constant OpenCV.Float64_Value := -S * X + C + 0.08;
         begin
            B (I) := [800.0 * (C * X + S - 0.12) / Z, 820.0 * (Y + 0.04) / Z];
         end;
      end loop;
      A (2) := [-400.0, 100.0]; B (2) := [-4_000.0, 5_000.0];
      B (7) := [5_000.0, -4_000.0]; B (15) := [-6_000.0, 3_000.0];
      N1 := Undistort_To_Normalized (A, (800.0, 820.0, 0.0, 0.0));
      N2 := Undistort_To_Normalized (B, (800.0, 820.0, 0.0, 0.0));
      declare
         Estimate : constant Homography_Estimate := Estimate_Homography_RANSAC (A, B, (2_000, 0.1, 0.999));
         Selected : constant Inlier_Index_Array := Inliers (Estimate);
         All_Results : constant Planar_Hypothesis_Visibility_Array := Assess_Planar_Visibility ([V], N1, N2);
         Subset : constant Planar_Hypothesis_Visibility_Array := Assess_Planar_Visibility ([V], N1, N2, Selected);
         Composed : constant Planar_Hypothesis_Visibility_Array :=
           Assess_Planar_Visibility (Hypotheses, N1, N2, Selected);
      begin
         Assert (Found (Estimate) and then Selected'Length = 21, "robust homography 21 selected inliers");
         for Index of Selected loop
            Assert (Index /= 2 and then Index /= 7 and then Index /= 15, "outliers excluded");
         end loop;
         Assert (All_Results (1).Disposition = Rejected_By_Visibility and then
           Subset (1).Disposition = Passes_Visibility and then Visibility_Oracle (V, N1, N2, Selected),
           "selection excludes contradictory reference");
         for I in Hypotheses'Range loop
            Assert ((Composed (I).Disposition = Passes_Visibility) =
              Visibility_Oracle (Hypotheses (I), N1, N2, Selected),
              "same planar decomposition / robust homography selected signs");
         end loop;
         Ada.Text_IO.Put_Line ("Visibility selected-inlier composition: supplied=24 selected=21; all rejected, selected passes");
      end;
   end Visibility_Selected;

   procedure Visibility_Pure (T : in out Fixture) is
      pragma Unreferenced (T);
   begin
      for Perturbation of Image_Point'([0.0, 0.0001]) loop
         declare
            V : constant Planar_Motion_Hypothesis_Array := Decompose_Calibrated_Homography
              ([[1.0, Perturbation, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], (1.0, 1.0, 0.0, 0.0));
            Empty : constant Planar_Hypothesis_Visibility_Array := Assess_Planar_Visibility (V, [], []);
            Full : constant Planar_Hypothesis_Visibility_Array := Assess_Planar_Visibility (V, [[0.2, 0.1]], [[0.25, 0.1]]);
         begin
            Assert (V'Length = 1 and then V (1).Pure_Rotation, "native pure/near rotation");
            Assert (Empty (1).Disposition = Not_Applicable_Pure_Rotation and then
              Full (1).Disposition = Not_Applicable_Pure_Rotation and then
              Visibility_Passing_Hypothesis_Indices (Full)'Length = 0, "pure not physically rejected/passing");
         end;
      end loop;
      declare
         C : constant OpenCV.Float64_Value := Math.Cos (0.12);
         S : constant OpenCV.Float64_Value := Math.Sin (0.12);
         V : constant Planar_Motion_Hypothesis_Array := Decompose_Calibrated_Homography
           ([[C, -S, 0.0], [S, C, 0.0], [0.0, 0.0, 1.0]], (1.0, 1.0, 0.0, 0.0));
      begin
         Assert (V'Length = 1 and then V (1).Pure_Rotation and then
           Assess_Planar_Visibility (V, [[0.2, 0.1]], [[0.18, 0.12]]) (1).Disposition =
             Not_Applicable_Pure_Rotation, "known nonidentity pure rotation");
      end;
   end Visibility_Pure;

   procedure Visibility_Empty (T : in out Fixture) is
      pragma Unreferenced (T);
      H : Planar_Motion_Hypothesis_Array (7 .. 7) := [7 => Visibility_Candidate];
      A : constant Normalized_Image_Point_Array (5 .. 5) := [5 => [0.2, 0.1]];
      B : constant Normalized_Image_Point_Array (9 .. 9) := [9 => [0.25, 0.1]];
   begin
      declare
         E : constant Planar_Hypothesis_Visibility_Array := Assess_Planar_Visibility ([], A, B);
         N : constant Planar_Hypothesis_Visibility_Array := Assess_Planar_Visibility (H, [], []);
         S : constant Planar_Hypothesis_Visibility_Array := Assess_Planar_Visibility (H, A, B, []);
         F : constant Planar_Hypothesis_Visibility_Array := Assess_Planar_Visibility (H, A, B, [8 => 1]);
      begin
         Assert (E'First = 1 and then E'Length = 0 and then N (1).Disposition = No_Selected_References and then
           S (1).Disposition = No_Selected_References and then F'First = 1 and then
           F (1).Disposition = Passes_Visibility, "empty/no-selected/nondefault bounds");
      end;
      H (7).Plane_Normal_In_First := [0.0, 0.0, -1.0];
      Assert (Visibility_Passing_Hypothesis_Indices (Assess_Planar_Visibility (H, A, B))'Length = 0,
        "zero native survivors is successful");
      declare
         Results : constant Planar_Hypothesis_Visibility_Array (8 .. 10) :=
           [8 => (Disposition => Passes_Visibility),
            9 => (Disposition => Not_Applicable_Pure_Rotation),
            10 => (Disposition => No_Selected_References)];
      begin
         Assert (Visibility_Passing_Hypothesis_Indices (Results) = [1], "helper positions independent of bounds");
      end;
   end Visibility_Empty;

   procedure Visibility_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check); -- Deliberate IEEE nonfinite inputs.
      V : Planar_Motion_Hypothesis;
      A : Normalized_Image_Point_Array (1 .. 2);
      procedure Reject (H : Planar_Motion_Hypothesis_Array;
                        P, Q : Normalized_Image_Point_Array; S : Inlier_Index_Array) is
      begin
         declare
            Results : constant Planar_Hypothesis_Visibility_Array := Assess_Planar_Visibility (H, P, Q, S);
            pragma Unreferenced (Results);
         begin
            Assert (False, "invalid visibility accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Reject;
   begin
      A := [others => [0.2, 0.1]];
      Reject ([Visibility_Candidate], A, A, [1, 1]);
      Reject ([Visibility_Candidate], A, A, [2, 1]);
      Reject ([Visibility_Candidate], A, A, [3]);
      Reject ([Visibility_Candidate], A, A (1 .. 1), []);
      Reject ([1 .. 5 => Visibility_Candidate], A, A, []);
      for Mode in 1 .. 9 loop
         V := Visibility_Candidate;
         case Mode is
            when 1 => V.Rotation_First_To_Second (0, 0) := 2.0;
            when 2 => V.Translation_Over_Plane_Distance := [others => 0.0];
            when 3 => V.Translation_Over_Plane_Distance (0) := OpenCV.Float64_Value (Nonfinite (0));
            when 4 => V.Plane_Normal_In_First := [0.0, 0.0, 2.0];
            when 5 => V.Plane_Normal_In_First (0) := OpenCV.Float64_Value (Nonfinite (1));
            when 6 => V.Pure_Rotation := True;
            when 7 => V.Plane_Normal_In_First := [others => 0.0];
            when 8 => V.Rotation_First_To_Second (0, 0) := -1.0;
            when 9 => V.Rotation_First_To_Second (0, 0) := OpenCV.Float64_Value (Nonfinite (0));
         end case;
         Reject ([V], A, A, []);
      end loop;
      A (2) (0) := OpenCV.Float64_Value (Nonfinite (0));
      Reject ([], A, A, []);
      Reject ([Visibility_Candidate], A, A, [1]);
   end Visibility_Invalid;

   procedure Visibility_Float32 (T : in out Fixture) is
      pragma Unreferenced (T);
   begin
      for X of Image_Point'([1.0E100, 1.0E-100]) loop
         begin
            declare
               Results : constant Planar_Hypothesis_Visibility_Array :=
                 Assess_Planar_Visibility ([Visibility_Candidate], [[X, 0.0]], [[0.25, 0.0]]);
               pragma Unreferenced (Results);
            begin
               Assert (False, "Float32 unrepresentable accepted");
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end loop;
      declare
         Results : constant Planar_Hypothesis_Visibility_Array := Assess_Planar_Visibility
           ([Visibility_Candidate], [[1.0E-40, 0.0]], [[3.0E38, 0.0]]);
      begin
         Assert (Results (1).Disposition = Passes_Visibility, "finite Float32 subnormal/large accepted");
      end;
      declare
         V : constant Planar_Motion_Hypothesis :=
           Visibility_Candidate ([Math.Sqrt (0.5), 0.0, -Math.Sqrt (0.5)]);
         Results : constant Planar_Hypothesis_Visibility_Array := Assess_Planar_Visibility
           ([V], [[1.0 + 2.0**(-30), 0.0]], [[2.0, 0.0]]);
      begin
         Assert (V.Plane_Normal_In_First (0) * (1.0 + 2.0**(-30)) +
           V.Plane_Normal_In_First (2) > 0.0, "Float64 boundary expression positive");
         Assert (Results (1).Disposition = Rejected_By_Visibility,
           "native Float32 rounds first X to 1, expression exactly zero");
         Ada.Text_IO.Put_Line ("Visibility Float32 sign boundary: Float64 positive, native rounded zero rejects");
      end;
   end Visibility_Float32;

   procedure Visibility_Layout (T : in out Fixture) is
      pragma Unreferenced (T);
      function Layout (Field : Interfaces.Integer_32) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_visibility_layout";
      procedure Fill (V : access ABI.C_Planar_Visibility)
        with Import, Convention => C, External_Name => "calib3d_test_fill_visibility";
      V : aliased ABI.C_Planar_Visibility;
      use type Interfaces.Unsigned_8;
   begin
      Ada.Text_IO.Put_Line ("Visibility layout observed Ada size/type-alignment=" & Natural'Image (V'Size) &
        "/" & Natural'Image (ABI.C_Planar_Visibility'Alignment) & " C=" & Interfaces.Integer_32'Image (Layout (0)) &
        "/" & Interfaces.Integer_32'Image (Layout (1)) & " positions=" & Natural'Image (V.Count'Position) &
        "/" & Natural'Image (V.Accepted'Position));
      Assert (V'Size = Natural (Layout (0)) * System.Storage_Unit and then
        ABI.C_Planar_Visibility'Alignment = Natural (Layout (1)) and then V.Count'Position = Natural (Layout (2)) and then
        V.Accepted'Position = Natural (Layout (3)), "compiler visibility layout");
      for I in 1 .. 4 loop
         Assert (V.Accepted'Position + I - 1 = Natural (Layout (Interfaces.Integer_32 (I + 3))), "flag offset");
      end loop;
      Fill (V'Access);
      Assert (V.Count = 4 and then V.Accepted (1) = 1 and then V.Accepted (2) = 0 and then
        V.Accepted (3) = 1 and then V.Accepted (4) = 0, "C-written/Ada-read full visibility result");
      Ada.Text_IO.Put_Line ("Visibility compiler size/alignment=" & Interfaces.Integer_32'Image (Layout (0)) &
        "/" & Interfaces.Integer_32'Image (Layout (1)) & " count/array/four offsets interchange PASS");
   end Visibility_Layout;

   procedure Planar_Layout (T : in out Fixture) is
      pragma Unreferenced (T);
      function Layout (Field : Interfaces.Integer_32) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_planar_layout";
      procedure Fill (V : access ABI.C_Planar_Decomposition)
        with Import, Convention => C, External_Name => "calib3d_test_fill_planar";
      V : aliased ABI.C_Planar_Decomposition;
      X : ABI.C_Planar_Motion;
      Offsets : constant array (Natural range 0 .. 14) of Natural :=
        [X.R00'Position, X.R01'Position, X.R02'Position, X.R10'Position, X.R11'Position, X.R12'Position, X.R20'Position, X.R21'Position, X.R22'Position, X.TX'Position, X.TY'Position, X.TZ'Position, X.NX'Position, X.NY'Position, X.NZ'Position];
   begin
      Assert (X'Size = Natural (Layout (0)) * System.Storage_Unit and then
        X'Alignment = Natural (Layout (1)), "candidate compiler layout");
      for I in Offsets'Range loop
         Assert (Offsets (I) = Natural (Layout (Interfaces.Integer_32 (I + 2))), "candidate field offset");
      end loop;
      Assert (V'Size = Natural (Layout (17)) * System.Storage_Unit and then
        V'Alignment = Natural (Layout (18)), "aggregate compiler layout");
      Assert (V.Count'Position = Natural (Layout (19)) and then
        V.Candidates'Position = Natural (Layout (20)), "aggregate offsets");
      Fill (V'Access);
      Assert (V.Count = 4, "C-written aggregate count");
      for I in 1 .. 4 loop
         Assert (V.Candidates'Position + (I - 1) * X'Size / System.Storage_Unit =
           Natural (Layout (Interfaces.Integer_32 (21 + I))), "four element locations");
         Assert (X'Size / System.Storage_Unit = Natural (Layout (21)), "candidate stride");
         X := V.Candidates (I);
         declare
            Values : constant array (Positive range 1 .. 15) of Interfaces.C.double :=
              [X.R00, X.R01, X.R02, X.R10, X.R11, X.R12, X.R20, X.R21, X.R22, X.TX, X.TY, X.TZ, X.NX, X.NY, X.NZ];
         begin
            for J in Values'Range loop
               Assert (Values (J) = Interfaces.C.double (J + (I - 1) * 100), "full-record interchange");
            end loop;
         end;
      end loop;
      Ada.Text_IO.Put_Line ("Planar layout candidate size/alignment=" & Interfaces.Integer_32'Image (Layout (0)) &
        "/" & Interfaces.Integer_32'Image (Layout (1)) & " aggregate=" & Interfaces.Integer_32'Image (Layout (17)) &
        "/" & Interfaces.Integer_32'Image (Layout (18)) & "; 15 offsets, stride, four locations/interchange PASS");
   end Planar_Layout;

   procedure Planar_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      Calibration : constant Camera_Intrinsics := (800.0, 820.0, 640.0, 360.0);
      type Matrix is array (Natural range 0 .. 2, Natural range 0 .. 2) of OpenCV.Float64_Value;
      Camera : constant Matrix := [[800.0, 0.0, 640.0], [0.0, 820.0, 360.0], [0.0, 0.0, 1.0]];
      Inverse : constant Matrix := [[1.0 / 800.0, 0.0, -640.0 / 800.0],
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
      Translation : constant Translation_Vector := [-0.12, 0.04, 0.08];
   begin
      for Pure in Boolean loop
         declare
            Angle : constant OpenCV.Float64_Value := (if Pure then 0.12 else 0.08);
            C : constant OpenCV.Float64_Value := Math.Cos (Angle);
            S : constant OpenCV.Float64_Value := Math.Sin (Angle);
            Rotation : constant Matrix := (if Pure then
              [[C, -S, 0.0], [S, C, 0.0], [0.0, 0.0, 1.0]] else
              [[C, 0.0, S], [0.0, 1.0, 0.0], [-S, 0.0, C]]);
            Normalized : Matrix := Rotation;
            H : Homography_Matrix;
         begin
            if not Pure then
               for I in 0 .. 2 loop
                  Normalized (I, 2) := Normalized (I, 2) + Translation (I);
               end loop;
            end if;
            H := Homography_Matrix (Product (Product (Camera, Normalized), Inverse));
            for Scale of Reprojection_Error_Array'[1.0, 7.0, -7.0, 1.0E-6, 1.0E6] loop
               declare
                  Scaled : Homography_Matrix := H;
                  Known : Boolean := False;
                  Maximum : OpenCV.Float64_Value := 0.0;
               begin
                  for I in 0 .. 2 loop
                     for J in 0 .. 2 loop
                        Scaled (I, J) := H (I, J) * Scale;
                     end loop;
                  end loop;
                  declare
                     Candidates : constant Planar_Motion_Hypothesis_Array :=
                       Decompose_Calibrated_Homography (Scaled, Calibration);
                  begin
                     Assert (Candidates'First = 1 and then Candidates'Length = (if Pure then 1 else 4),
                       "native planar count / one based");
                     for V of Candidates loop
                        declare
                           Implied : Matrix;
                           Dot, Norm, Factor, Error, Motion_Error : OpenCV.Float64_Value := 0.0;
                           Paired : Boolean := Pure;
                        begin
                           Assert (V.Pure_Rotation = Pure, "pure flag");
                           for I in 0 .. 2 loop
                              for J in 0 .. 2 loop
                                 Implied (I, J) := V.Rotation_First_To_Second (I, J) +
                                   V.Translation_Over_Plane_Distance (I) * V.Plane_Normal_In_First (J);
                                 Dot := Dot + Implied (I, J) * Normalized (I, J);
                                 Norm := Norm + Implied (I, J)**2;
                                 Motion_Error := OpenCV.Float64_Value'Max (Motion_Error,
                                   abs (V.Rotation_First_To_Second (I, J) - Rotation (I, J)));
                              end loop;
                           end loop;
                           Factor := Dot / Norm;
                           for I in 0 .. 2 loop
                              for J in 0 .. 2 loop
                                 Error := OpenCV.Float64_Value'Max (Error,
                                   abs (Factor * Implied (I, J) - Normalized (I, J)));
                              end loop;
                           end loop;
                           Maximum := OpenCV.Float64_Value'Max (Maximum, Error);
                           Assert (Error < 1.0E-10, "projective reconstruction");
                           if Pure then
                              Assert (V.Translation_Over_Plane_Distance = Translation_Vector'[0.0, 0.0, 0.0]
                                and then V.Plane_Normal_In_First = Camera_Direction'[0.0, 0.0, 0.0], "pure zeros");
                              Known := Motion_Error < 1.0E-10;
                           else
                              Assert (Near (Math.Sqrt (V.Translation_Over_Plane_Distance (0)**2 +
                                V.Translation_Over_Plane_Distance (1)**2 +
                                V.Translation_Over_Plane_Distance (2)**2), Math.Sqrt (0.0224), 1.0E-10),
                                "translation magnitude not normalized");
                              Known := Known or else (Motion_Error < 1.0E-10 and then
                                abs (V.Translation_Over_Plane_Distance (0) * V.Plane_Normal_In_First (2) + 0.12) < 1.0E-10);
                              for W of Candidates loop
                                 declare
                                    Difference : OpenCV.Float64_Value := 0.0;
                                 begin
                                    for I in 0 .. 2 loop
                                       Difference := Difference + abs (V.Translation_Over_Plane_Distance (I) +
                                         W.Translation_Over_Plane_Distance (I)) +
                                         abs (V.Plane_Normal_In_First (I) + W.Plane_Normal_In_First (I));
                                       for J in 0 .. 2 loop
                                          Difference := Difference + abs (V.Rotation_First_To_Second (I, J) -
                                            W.Rotation_First_To_Second (I, J));
                                       end loop;
                                    end loop;
                                    Paired := Paired or else Difference < 1.0E-10;
                                 end;
                              end loop;
                           end if;
                           Assert (Paired, "simultaneous t/n sign pair");
                        end;
                     end loop;
                     Assert (Known, "known first-to-second motion represented");
                     Ada.Text_IO.Put_Line ("Planar pure=" & Boolean'Image (Pure) & " scale=" &
                       OpenCV.Float64_Value'Image (Scale) & " reconstruction=" & OpenCV.Float64_Value'Image (Maximum));
                  end;
               end;
            end loop;
         end;
      end loop;
   end Planar_Oracle;

   procedure Planar_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check);
      Identity : constant Homography_Matrix := [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
      procedure Reject (H : Homography_Matrix; Calibration : Camera_Intrinsics := K) is
      begin
         declare
            V : constant Planar_Motion_Hypothesis_Array := Decompose_Calibrated_Homography (H, Calibration);
         begin
            Assert (False, "invalid homography accepted" & Natural'Image (V'Length));
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Reject;
      H : Homography_Matrix := Identity;
   begin
      declare
         V : constant Planar_Motion_Hypothesis_Array := Decompose_Calibrated_Homography
           ([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]], (1.0, 1.0, 0.0, 0.0));
      begin
         Assert (V'Length = 1 and then V (1).Pure_Rotation, "valid zero h22 is not rejected");
      end;
      Reject ([others => [others => 0.0]]);
      H (2, 2) := 0.0;
      Reject (H);
      H := Identity;
      H (0, 0) := OpenCV.Float64_Value (Nonfinite (0));
      Reject (H);
      Reject (Identity, (0.0, 820.0, 640.0, 360.0));
      Reject (Identity, (800.0, -1.0, 640.0, 360.0));
      Reject (Identity, (800.0, 820.0, OpenCV.Float64_Value (Nonfinite (1)), 360.0));
      H := Identity;
      H (0, 0) := OpenCV.Float64_Value'Last;
      H (1, 1) := 1.0E-300;
      Reject (H);
   end Planar_Invalid;

   procedure AP3P_Known_Pose (T : in out Fixture) is
      pragma Unreferenced (T);
      function Rotation_Y (Angle : OpenCV.Float64_Value) return Rotation_Matrix is
         C : constant OpenCV.Float64_Value := Math.Cos (Angle);
         S : constant OpenCV.Float64_Value := Math.Sin (Angle);
      begin
         return [[C, 0.0, S], [0.0, 1.0, 0.0], [-S, 0.0, C]];
      end Rotation_Y;
      function Rotation_Z (Angle : OpenCV.Float64_Value) return Rotation_Matrix is
         C : constant OpenCV.Float64_Value := Math.Cos (Angle);
         S : constant OpenCV.Float64_Value := Math.Sin (Angle);
      begin
         return [[C, -S, 0.0], [S, C, 0.0], [0.0, 0.0, 1.0]];
      end Rotation_Z;
      function Multiply (Left, Right : Rotation_Matrix) return Rotation_Matrix is
         Result : Rotation_Matrix := [others => [others => 0.0]];
      begin
         for Row in 0 .. 2 loop
            for Column in 0 .. 2 loop
               for K in 0 .. 2 loop
                  Result (Row, Column) := Result (Row, Column) + Left (Row, K) * Right (K, Column);
               end loop;
            end loop;
         end loop;
         return Result;
      end Multiply;
      function Apply (Matrix : Rotation_Matrix; Point : Object_Point) return Object_Point is
         ([Matrix (0, 0) * Point (0) + Matrix (0, 1) * Point (1) + Matrix (0, 2) * Point (2),
          Matrix (1, 0) * Point (0) + Matrix (1, 1) * Point (1) + Matrix (1, 2) * Point (2),
          Matrix (2, 0) * Point (0) + Matrix (2, 1) * Point (1) + Matrix (2, 2) * Point (2)]);
      function Transpose (Matrix : Rotation_Matrix) return Rotation_Matrix is
         ([[Matrix (0, 0), Matrix (1, 0), Matrix (2, 0)],
           [Matrix (0, 1), Matrix (1, 1), Matrix (2, 1)],
           [Matrix (0, 2), Matrix (1, 2), Matrix (2, 2)]]);
      Truth_R : constant Rotation_Matrix := Multiply (Rotation_Z (0.22), Rotation_Y (0.35));
      Truth_T : constant Translation_Vector := [0.2, -0.3, 0.8];
      Calibration : constant Camera_Intrinsics := (800.0, 820.0, 640.0, 360.0);
      Landmarks : constant Object_Point_Array := [[0.3, -0.2, 3.5], [1.4, 0.1, 4.0], [-0.5, 1.2, 5.3]];
      Fourth : constant Object_Point := [0.7, -0.8, 6.1];
      function Project (Point : Object_Point; Distortion : Distortion_Coefficients) return Image_Point is
         Camera : constant Object_Point :=
           [Apply (Truth_R, Point) (0) + Truth_T (0), Apply (Truth_R, Point) (1) + Truth_T (1),
            Apply (Truth_R, Point) (2) + Truth_T (2)];
         X : constant OpenCV.Float64_Value := Camera (0) / Camera (2);
         Y : constant OpenCV.Float64_Value := Camera (1) / Camera (2);
         R2 : constant OpenCV.Float64_Value := X * X + Y * Y;
         Radial : constant OpenCV.Float64_Value :=
           1.0 + Distortion.K1 * R2 + Distortion.K2 * R2 * R2 + Distortion.K3 * R2 * R2 * R2;
      begin
         return [Calibration.Focal_X * (X * Radial + 2.0 * Distortion.P1 * X * Y +
                   Distortion.P2 * (R2 + 2.0 * X * X)) + Calibration.Center_X,
                 Calibration.Focal_Y * (Y * Radial + Distortion.P1 * (R2 + 2.0 * Y * Y) +
                   2.0 * Distortion.P2 * X * Y) + Calibration.Center_Y];
      end Project;
      procedure Qualify (Distortion : Distortion_Coefficients; Label : String) is
         Pixels : Image_Point_Array (1 .. 3);
         Fourth_Pixel : constant Image_Point := Project (Fourth, Distortion);
         Count : Natural := 0;
         Matched : Boolean := False;
         Best_Angle : OpenCV.Float64_Value := OpenCV.Float64_Value'Last;
         Best_Translation, Best_Center, Best_Pixels, Best_Fourth : OpenCV.Float64_Value := 0.0;
      begin
         for I in Landmarks'Range loop
            Pixels (I) := Project (Landmarks (I), Distortion);
            Assert (Apply (Truth_R, Landmarks (I)) (2) + Truth_T (2) > 0.0, "positive known depth");
         end loop;
         declare
            Found : constant AP3P_Pose_Hypothesis_Array :=
              Solve_AP3P_Hypotheses (Landmarks, Pixels, Calibration, Distortion);
         begin
            Count := Found'Length;
            Assert (Found'First = 1 and then Count in 1 .. 4, "AP3P one-based bounded hypotheses");
            for Candidate of Found loop
               declare
                  R : constant Rotation_Matrix := Rotation_Matrix_Of (Candidate.Pose);
                  Trace : OpenCV.Float64_Value := 0.0;
                  Center : constant Object_Point := Camera_Center (Candidate.Pose);
                  Expected : constant Object_Point :=
                    [-Apply (Transpose (Truth_R), Truth_T) (0), -Apply (Transpose (Truth_R), Truth_T) (1),
                     -Apply (Transpose (Truth_R), Truth_T) (2)];
                  Angle, Translation_Error, Center_Error, Pixel_Error, Fourth_Error : OpenCV.Float64_Value;
                  function Residual (Observed : Image_Point; Point : Object_Point) return OpenCV.Float64_Value is
                     Camera : constant Object_Point :=
                       [Apply (R, Point) (0) + Candidate.Pose.Translation (0),
                        Apply (R, Point) (1) + Candidate.Pose.Translation (1),
                        Apply (R, Point) (2) + Candidate.Pose.Translation (2)];
                     X : constant OpenCV.Float64_Value := Camera (0) / Camera (2);
                     Y : constant OpenCV.Float64_Value := Camera (1) / Camera (2);
                     R2 : constant OpenCV.Float64_Value := X * X + Y * Y;
                     Radial : constant OpenCV.Float64_Value :=
                       1.0 + Distortion.K1 * R2 + Distortion.K2 * R2 * R2 + Distortion.K3 * R2 * R2 * R2;
                     Predicted : constant Image_Point :=
                       [Calibration.Focal_X * (X * Radial + 2.0 * Distortion.P1 * X * Y +
                          Distortion.P2 * (R2 + 2.0 * X * X)) + Calibration.Center_X,
                        Calibration.Focal_Y * (Y * Radial + Distortion.P1 * (R2 + 2.0 * Y * Y) +
                          2.0 * Distortion.P2 * X * Y) + Calibration.Center_Y];
                  begin
                     return Math.Sqrt ((Predicted (0) - Observed (0)) ** 2 + (Predicted (1) - Observed (1)) ** 2);
                  end Residual;
               begin
                  for I in 0 .. 2 loop
                     Trace := Trace + R (0, I) * Truth_R (0, I) + R (1, I) * Truth_R (1, I) + R (2, I) * Truth_R (2, I);
                  end loop;
                  Angle := Math.Arccos (OpenCV.Float64_Value'Max (-1.0, OpenCV.Float64_Value'Min (1.0, (Trace - 1.0) / 2.0)));
                  Translation_Error := Math.Sqrt ((Candidate.Pose.Translation (0) - Truth_T (0)) ** 2 +
                    (Candidate.Pose.Translation (1) - Truth_T (1)) ** 2 + (Candidate.Pose.Translation (2) - Truth_T (2)) ** 2);
                  Center_Error := Math.Sqrt ((Center (0) - Expected (0)) ** 2 + (Center (1) - Expected (1)) ** 2 +
                    (Center (2) - Expected (2)) ** 2);
                  Pixel_Error := 0.0;
                  for I in Landmarks'Range loop
                     Pixel_Error := OpenCV.Float64_Value'Max (Pixel_Error, Residual (Pixels (I), Landmarks (I)));
                  end loop;
                  Fourth_Error := Residual (Fourth_Pixel, Fourth);
                  if Angle < Best_Angle then
                     Best_Angle := Angle; Best_Translation := Translation_Error; Best_Center := Center_Error;
                     Best_Pixels := Pixel_Error; Best_Fourth := Fourth_Error;
                  end if;
                  Matched := Matched or else (Angle < 1.0E-4 and then Translation_Error < 1.0E-4 and then
                    Center_Error < 1.0E-4 and then Pixel_Error < 1.0E-3 and then Fourth_Error < 1.0E-3);
               end;
            end loop;
         end;
         Assert (Matched, "known AP3P pose represented");
         Ada.Text_IO.Put_Line (Label & " hypotheses=" & Natural'Image (Count) & " angle=" &
           OpenCV.Float64_Value'Image (Best_Angle) & " translation=" & OpenCV.Float64_Value'Image (Best_Translation) &
           " center=" & OpenCV.Float64_Value'Image (Best_Center) & " pixels=" & OpenCV.Float64_Value'Image (Best_Pixels) &
           " fourth=" & OpenCV.Float64_Value'Image (Best_Fourth));
      end Qualify;
      Distortion : constant Distortion_Coefficients := (-0.12, 0.02, 0.001, -0.0015, 0.01);
   begin
      Qualify (No_Distortion, "AP3P zero distortion");
      Qualify (Distortion, "AP3P five-coefficient distortion");
   end AP3P_Known_Pose;

   procedure AP3P_Multiple (T : in out Fixture) is
      pragma Unreferenced (T);
      Calibration : constant Camera_Intrinsics := (800.0, 820.0, 640.0, 360.0);
      World : constant Object_Point_Array := [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
      --  Identity pose, t=(0,0,5): u=fx*X/Z+cx. Front-parallel square admits multiple AP3P roots.
      Pixels : constant Image_Point_Array := [[640.0, 360.0], [800.0, 360.0], [640.0, 524.0]];
      Found : constant AP3P_Pose_Hypothesis_Array := Solve_AP3P_Hypotheses (World, Pixels, Calibration);
   begin
      Assert (Found'First = 1 and then Found'Length > 1 and then Found'Length <= 4,
              "multiple native AP3P hypotheses retained without selection");
      for Candidate of Found loop
         declare
            R : constant Rotation_Matrix := Rotation_Matrix_Of (Candidate.Pose);
         begin
            Assert ((for all Value of R => Finite (Value)), "finite multiple-solution rotation");
            Assert ((for all Value of Candidate.Pose.Translation => Finite (Value)), "finite translation");
         end;
      end loop;
      Ada.Text_IO.Put_Line ("AP3P multiple hypotheses=" & Natural'Image (Found'Length));
   end AP3P_Multiple;

   procedure AP3P_Reject (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check);
      Calibration : constant Camera_Intrinsics := (800.0, 820.0, 640.0, 360.0);
      World : constant Object_Point_Array := [[0.3, -0.2, 3.5], [1.4, 0.1, 4.0], [-0.5, 1.2, 5.3]];
      Image : constant Image_Point_Array := [[700.0, 300.0], [900.0, 400.0], [500.0, 550.0]];
      procedure Reject (Objects : Object_Point_Array; Pixels : Image_Point_Array;
                        Camera : Camera_Intrinsics := Calibration;
                        Distortion : Distortion_Coefficients := No_Distortion) is
      begin
         declare
            Found : constant AP3P_Pose_Hypothesis_Array :=
              Solve_AP3P_Hypotheses (Objects, Pixels, Camera, Distortion);
         begin
            Assert (False, "invalid AP3P accepted" & Natural'Image (Found'Length));
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Reject;
   begin
      Reject ([1 .. 0 => [0.0, 0.0, 0.0]], [1 .. 0 => [0.0, 0.0]]);
      Reject (World (1 .. 1), Image (1 .. 1));
      Reject (World (1 .. 2), Image (1 .. 2));
      Reject ([World (1), World (2), World (3), [0.1, 0.2, 6.0]],
              [Image (1), Image (2), Image (3), [1.0, 2.0]]);
      Reject ([World (1), World (2), World (3), [0.1, 0.2, 6.0], [0.2, 0.3, 7.0]],
              [Image (1), Image (2), Image (3), [1.0, 2.0], [3.0, 4.0]]);
      Reject (World, Image (1 .. 2));
      Reject ([World (1), World (1), World (3)], Image);
      Reject ([[0.0, 0.0, 1.0], [1.0, 0.0, 2.0], [2.0, 0.0, 3.0]], Image);
      Reject (World, [Image (1), Image (1), Image (3)]);
      Reject (World, [[640.0, 360.0], [700.0, 360.0], [500.0, 360.0]]);
      Reject ([[1.0E200, 0.0, 1.0E200], [0.0, 1.0E200, 1.0E200], [-1.0E200, -1.0E200, 2.0E200]], Image);
      Reject (World, Image, (0.0, 820.0, 640.0, 360.0));
      Reject (World, Image, (800.0, -1.0, 640.0, 360.0));
      Reject (World, Image, (800.0, 820.0, OpenCV.Float64_Value (Nonfinite (0)), 360.0));
      Reject (World, Image, Calibration, (OpenCV.Float64_Value (Nonfinite (1)), 0.0, 0.0, 0.0, 0.0));
      declare
         Bad_World : Object_Point_Array := World;
         Bad_Image : Image_Point_Array := Image;
      begin
         Bad_World (1) (0) := OpenCV.Float64_Value (Nonfinite (0));
         Reject (Bad_World, Image);
         Bad_Image (2) (1) := OpenCV.Float64_Value (Nonfinite (1));
         Reject (World, Bad_Image);
      end;
   end AP3P_Reject;

   procedure AP3P_Zero (T : in out Fixture) is
      pragma Unreferenced (T);
      Found : constant AP3P_Pose_Hypothesis_Array := Solve_AP3P_Hypotheses
        ([[-0.39081598340561996, 0.32617277568950875, 4.0131477606213553],
          [-1.6067503815809268, 0.73284013236088175, 2.4253925955828199],
          [1.9483108946655348, -1.1457818430811084, 7.0007856567437514]],
         [[118.67887557148487, 129.79777044444734],
          [452.33705169300674, 427.73531562095332],
          [690.96005298599528, 241.045602436595]],
         (800.0, 820.0, 640.0, 360.0));
   begin
      Assert (Found'First = 1 and then Found'Last = 0, "valid native zero is an empty array");
   end AP3P_Zero;

   procedure AP3P_Frame (T : in out Fixture) is
      pragma Unreferenced (T);
      Calibration : constant Camera_Intrinsics := (800.0, 820.0, 640.0, 360.0);
      World : constant Object_Point_Array := [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
      Pixels : constant Image_Point_Array := [[640.0, 360.0], [800.0, 360.0], [640.0, 524.0]];
      Found : constant AP3P_Pose_Hypothesis_Array := Solve_AP3P_Hypotheses (World, Pixels, Calibration);
      Matched : Boolean := False;
   begin
      for Candidate of Found loop
         declare
            R : constant Rotation_Matrix := Rotation_Matrix_Of (Candidate.Pose);
            Camera : constant Object_Point :=
              [R (0, 0) * World (1) (0) + R (0, 1) * World (1) (1) + R (0, 2) * World (1) (2) + Candidate.Pose.Translation (0),
               R (1, 0) * World (1) (0) + R (1, 1) * World (1) (1) + R (1, 2) * World (1) (2) + Candidate.Pose.Translation (1),
               R (2, 0) * World (1) (0) + R (2, 1) * World (1) (1) + R (2, 2) * World (1) (2) + Candidate.Pose.Translation (2)];
            Center : constant Object_Point := Camera_Center (Candidate.Pose);
            Depth : constant OpenCV.Float64_Value := Camera (2);
         begin
            if abs (Camera (0) - Candidate.Pose.Translation (0)) < 1.0E-9 and then
              abs (Camera (1) - Candidate.Pose.Translation (1)) < 1.0E-9 and then Depth > 0.0 and then
              abs (Calibration.Focal_X * Camera (0) / Depth + Calibration.Center_X - Pixels (1) (0)) < 1.0E-6 and then
              abs (Center (0) + (R (0, 0) * Candidate.Pose.Translation (0) + R (1, 0) * Candidate.Pose.Translation (1) +
                   R (2, 0) * Candidate.Pose.Translation (2))) < 1.0E-8
            then
               Matched := True;
            end if;
         end;
      end loop;
      Assert (Matched, "world-to-camera frame and camera center");
   end AP3P_Frame;

   procedure AP3P_Layout (T : in out Fixture) is
      pragma Unreferenced (T);
      function Field (Index : Interfaces.Integer_32) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_ap3p_layout";
      procedure Fill (Value : access ABI.C_AP3P_Hypotheses)
        with Import, Convention => C, External_Name => "calib3d_test_fill_ap3p";
      Native : aliased ABI.C_AP3P_Hypotheses;
   begin
      Assert (ABI.C_AP3P_Hypotheses'Size = Natural (Field (0)) * System.Storage_Unit, "AP3P size");
      Assert (ABI.C_AP3P_Hypotheses'Alignment = Natural (Field (1)), "AP3P alignment");
      Assert (Native.Count'Position = Natural (Field (2)), "AP3P count position");
      Assert (Native.Candidates'Position = Natural (Field (3)), "AP3P candidates position");
      Assert (ABI.C_Pose'Size = Natural (Field (4)) * System.Storage_Unit, "reused pose stride");
      Assert (Native.Candidates (1).RX'Position = 0, "candidate-relative first pose");
      Assert (Field (5) = Field (3), "first C candidate location");
      Assert (Field (6) - Field (5) = 48, "second C candidate stride");
      Assert (Field (7) - Field (6) = 48, "third C candidate stride");
      Assert (Field (8) - Field (7) = 48, "fourth C candidate stride");
      Fill (Native'Access);
      Assert (Native.Count = 4, "AP3P interchange count");
      for I in 1 .. 4 loop
         Assert (Native.Candidates (I).RX = Interfaces.C.double (I) and then
                 Native.Candidates (I).RY = Interfaces.C.double (I + 1) and then
                 Native.Candidates (I).RZ = Interfaces.C.double (I + 2) and then
                 Native.Candidates (I).TX = Interfaces.C.double (I + 3) and then
                 Native.Candidates (I).TY = Interfaces.C.double (I + 4) and then
                 Native.Candidates (I).TZ = Interfaces.C.double (I + 5), "six-field interchange");
      end loop;
   end AP3P_Layout;

    procedure AP3P_Reference_Known (T : in out Fixture) is
       pragma Unreferenced (T);
       function Rotation_Y (Angle : OpenCV.Float64_Value) return Rotation_Matrix is
          C : constant OpenCV.Float64_Value := Math.Cos (Angle);
          S : constant OpenCV.Float64_Value := Math.Sin (Angle);
       begin
          return [[C, 0.0, S], [0.0, 1.0, 0.0], [-S, 0.0, C]];
       end Rotation_Y;
       function Rotation_Z (Angle : OpenCV.Float64_Value) return Rotation_Matrix is
          C : constant OpenCV.Float64_Value := Math.Cos (Angle);
          S : constant OpenCV.Float64_Value := Math.Sin (Angle);
       begin
          return [[C, -S, 0.0], [S, C, 0.0], [0.0, 0.0, 1.0]];
       end Rotation_Z;
       function Multiply (Left, Right : Rotation_Matrix) return Rotation_Matrix is
          Result : Rotation_Matrix := [others => [others => 0.0]];
       begin
          for Row in 0 .. 2 loop
             for Column in 0 .. 2 loop
                for Axis in 0 .. 2 loop
                   Result (Row, Column) := Result (Row, Column) + Left (Row, Axis) * Right (Axis, Column);
                end loop;
             end loop;
          end loop;
          return Result;
       end Multiply;
       function Apply (Matrix : Rotation_Matrix; Point : Object_Point) return Object_Point is
          ([Matrix (0, 0) * Point (0) + Matrix (0, 1) * Point (1) + Matrix (0, 2) * Point (2),
           Matrix (1, 0) * Point (0) + Matrix (1, 1) * Point (1) + Matrix (1, 2) * Point (2),
           Matrix (2, 0) * Point (0) + Matrix (2, 1) * Point (1) + Matrix (2, 2) * Point (2)]);
       Truth_R : constant Rotation_Matrix := Multiply (Rotation_Z (0.22), Rotation_Y (0.35));
       Truth_T : constant Translation_Vector := [0.2, -0.3, 0.8];
       Cam : constant Camera_Intrinsics := (800.0, 820.0, 640.0, 360.0);
       Seeds : constant Object_Point_Array := [[0.3, -0.2, 3.5], [1.4, 0.1, 4.0], [-0.5, 1.2, 5.3]];
       Fourth : constant Object_Point := [0.7, -0.8, 6.1];
       Depth : constant OpenCV.Float64_Value := Apply (Truth_R, Fourth) (2) + Truth_T (2);
       function Angle_Of (Candidate : Rotation_Matrix) return OpenCV.Float64_Value is
          Trace : OpenCV.Float64_Value := 0.0;
       begin
          for Axis in 0 .. 2 loop
             Trace := Trace + Candidate (0, Axis) * Truth_R (0, Axis)
               + Candidate (1, Axis) * Truth_R (1, Axis)
               + Candidate (2, Axis) * Truth_R (2, Axis);
          end loop;
          return Math.Arccos (OpenCV.Float64_Value'Max (-1.0,
            OpenCV.Float64_Value'Min (1.0, (Trace - 1.0) / 2.0)));
       end Angle_Of;
       function Project (Point : Object_Point) return Image_Point is
          Camera : constant Object_Point :=
            [Apply (Truth_R, Point) (0) + Truth_T (0),
             Apply (Truth_R, Point) (1) + Truth_T (1),
             Apply (Truth_R, Point) (2) + Truth_T (2)];
       begin
          return [Cam.Focal_X * Camera (0) / Camera (2) + Cam.Center_X,
                  Cam.Focal_Y * Camera (1) / Camera (2) + Cam.Center_Y];
       end Project;
       Pixels : Image_Point_Array (1 .. 3);
       World_Ref : constant Object_Point_Array (4 .. 4) := [Fourth];
       Pixel_Ref : Image_Point_Array (9 .. 9);
    begin
       Assert (Depth > 0.0, "analytic fourth depth");
       for I in Seeds'Range loop
          Pixels (I) := Project (Seeds (I));
       end loop;
       Pixel_Ref (9) := Project (Fourth);
       declare
          Found : constant AP3P_Pose_Hypothesis_Array :=
            Solve_AP3P_Hypotheses (Seeds, Pixels, Cam);
          Before : constant AP3P_Pose_Hypothesis_Array := Found;
          Saved_World : constant Object_Point_Array := World_Ref;
          Saved_Pixels : constant Image_Point_Array := Pixel_Ref;
          Values : constant AP3P_Reference_Result_Matrix :=
            Evaluate_AP3P_References (Found, World_Ref, Pixel_Ref, Cam);
          Summaries : constant AP3P_Hypothesis_Reference_Summary_Array :=
            Summarize_AP3P_References (Values);
          Matched, Alternate : Boolean := False;
          Best : OpenCV.Float64_Value := 0.0;
       begin
          Assert (Found = Before and then World_Ref = Saved_World
            and then Pixel_Ref = Saved_Pixels, "inputs unchanged");
          Assert (Values'First (1) = 1 and then Values'Last (1) = Found'Length
            and then Values'First (2) = 1 and then Values'Last (2) = 1,
            "one-based fourth-reference matrix");
          for I in Found'Range loop
             declare
                Angle : constant OpenCV.Float64_Value :=
                  Angle_Of (Rotation_Matrix_Of (Found (I).Pose));
                Shift : constant OpenCV.Float64_Value := Math.Sqrt
                  ((Found (I).Pose.Translation (0) - Truth_T (0)) ** 2
                   + (Found (I).Pose.Translation (1) - Truth_T (1)) ** 2
                   + (Found (I).Pose.Translation (2) - Truth_T (2)) ** 2);
                Item : constant AP3P_Reference_Result := Values (I, 1);
             begin
                Assert (Item.Disposition = Projected_Positive_Depth, "projected");
                Assert (Summaries (I).Total_References = 1
                  and then Summaries (I).Projected_References = 1
                  and then Summaries (I).Reprojection.Count = 1
                  and then Summaries (I).Entire_Set_Evaluable, "one-reference summary");
                if Angle < 1.0E-4 and then Shift < 1.0E-4 then
                   Matched := True;
                   Best := Item.Error_Pixels;
                   Assert (abs (Item.Camera_Depth - Depth) < 1.0E-8, "known depth");
                   Assert (Item.Error_Pixels < 1.0E-3, "known fourth error");
                elsif Item.Error_Pixels > 1.0 then
                   Alternate := True;
                end if;
             end;
          end loop;
          Assert (Matched and then (Found'Length = 1 or else Alternate),
            "known pose kept with alternatives and no selection");
          Ada.Text_IO.Put_Line ("AP3P fourth-reference hypotheses=" & Natural'Image (Found'Length)
            & " known-error=" & OpenCV.Float64_Value'Image (Best)
            & " depth=" & OpenCV.Float64_Value'Image (Depth));
       end;
    end AP3P_Reference_Known;

    procedure AP3P_Reference_Many (T : in out Fixture) is
       pragma Unreferenced (T);
       Cam : constant Camera_Intrinsics := (800.0, 820.0, 640.0, 360.0);
       Seeds : constant Object_Point_Array := [[0.3, -0.2, 3.5], [1.4, 0.1, 4.0], [-0.5, 1.2, 5.3]];
       Refs : constant Object_Point_Array (10 .. 14) :=
         [[0.7, -0.8, 6.1], [1.8, 0.4, 5.2], [-1.1, 0.9, 7.4], [0.2, -1.6, 4.8], [-0.4, 0.3, 9.0]];
       Pixels : Image_Point_Array (1 .. 3);
       Observed : Image_Point_Array (21 .. 25);
       function RY (Angle : OpenCV.Float64_Value) return Rotation_Matrix is
          C : constant OpenCV.Float64_Value := Math.Cos (Angle);
          S : constant OpenCV.Float64_Value := Math.Sin (Angle);
       begin
          return [[C, 0.0, S], [0.0, 1.0, 0.0], [-S, 0.0, C]];
       end RY;
       function RZ (Angle : OpenCV.Float64_Value) return Rotation_Matrix is
          C : constant OpenCV.Float64_Value := Math.Cos (Angle);
          S : constant OpenCV.Float64_Value := Math.Sin (Angle);
       begin
          return [[C, -S, 0.0], [S, C, 0.0], [0.0, 0.0, 1.0]];
       end RZ;
       function Pose_Matrix return Rotation_Matrix is
          A : constant Rotation_Matrix := RZ (0.22);
          B : constant Rotation_Matrix := RY (0.35);
          Result : Rotation_Matrix := [others => [others => 0.0]];
       begin
          for Row in 0 .. 2 loop
             for Column in 0 .. 2 loop
                for Axis in 0 .. 2 loop
                   Result (Row, Column) := Result (Row, Column) + A (Row, Axis) * B (Axis, Column);
                end loop;
             end loop;
          end loop;
          return Result;
       end Pose_Matrix;
       Truth : constant Rotation_Matrix := Pose_Matrix;
       Shift : constant Translation_Vector := [0.2, -0.3, 0.8];
       function Project (Point : Object_Point) return Image_Point is
          X : constant OpenCV.Float64_Value :=
            Truth (0, 0) * Point (0) + Truth (0, 1) * Point (1) + Truth (0, 2) * Point (2) + Shift (0);
          Y : constant OpenCV.Float64_Value :=
            Truth (1, 0) * Point (0) + Truth (1, 1) * Point (1) + Truth (1, 2) * Point (2) + Shift (1);
          Z : constant OpenCV.Float64_Value :=
            Truth (2, 0) * Point (0) + Truth (2, 1) * Point (1) + Truth (2, 2) * Point (2) + Shift (2);
       begin
          return [Cam.Focal_X * X / Z + Cam.Center_X, Cam.Focal_Y * Y / Z + Cam.Center_Y];
       end Project;
       Known : Natural := 0;
       Separated : Boolean := False;
    begin
       for I in Seeds'Range loop
          Pixels (I) := Project (Seeds (I));
       end loop;
       for Offset in 0 .. Refs'Length - 1 loop
          Observed (Observed'First + Offset) := Project (Refs (Refs'First + Offset));
       end loop;
       declare
          Found : constant AP3P_Pose_Hypothesis_Array :=
            Solve_AP3P_Hypotheses (Seeds, Pixels, Cam);
          Values : constant AP3P_Reference_Result_Matrix :=
            Evaluate_AP3P_References (Found, Refs, Observed, Cam);
          Summaries : constant AP3P_Hypothesis_Reference_Summary_Array :=
            Summarize_AP3P_References (Values);
       begin
       Assert (Values'First (1) = 1 and then Values'Last (1) = Found'Length
         and then Values'First (2) = 1 and then Values'Last (2) = 5, "position pairing");
       for Row in Values'Range (1) loop
          declare
             Errors : Reprojection_Error_Array (1 .. 5);
             Direct : constant Reprojection_Error_Array :=
               Reprojection_Errors (Refs, Observed, Cam, No_Distortion, Found (Row).Pose);
             Maximum : OpenCV.Float64_Value := 0.0;
          begin
             for Column in Values'Range (2) loop
                Assert (Values (Row, Column).Disposition = Projected_Positive_Depth, "all projected");
                Assert (abs (Values (Row, Column).Error_Pixels - Direct (Column)) <= 1.0E-9
                  or else abs (Values (Row, Column).Error_Pixels - Direct (Column))
                    <= 1.0E-9 * OpenCV.Float64_Value'Max (1.0, Direct (Column)),
                  "Reprojection_Errors agreement");
                Errors (Column) := Values (Row, Column).Error_Pixels;
                Maximum := OpenCV.Float64_Value'Max (Maximum, Errors (Column));
             end loop;
             declare
                Expected : constant Reprojection_Summary := Summarize_Reprojection (Errors);
             begin
                Assert (Summaries (Row).Projected_References = 5
                  and then Summaries (Row).Total_References = 5
                  and then Summaries (Row).Entire_Set_Evaluable
                  and then Summaries (Row).Reprojection.Count = 5
                  and then Summaries (Row).Reprojection.RMS_Error_Pixels = Expected.RMS_Error_Pixels
                  and then Summaries (Row).Reprojection.Maximum_Error_Pixels = Expected.Maximum_Error_Pixels,
                  "summary reuses Summarize_Reprojection");
             end;
             if Maximum < 1.0E-3 then
                Known := Known + 1;
             elsif Maximum > 1.0 then
                Separated := True;
             end if;
          end;
       end loop;
       Assert (Known = 1 and then (Found'Length = 1 or else Separated),
         "independent references distinguish one known candidate");
       Ada.Text_IO.Put_Line ("AP3P many references=" & Natural'Image (Refs'Length)
         & " hypotheses=" & Natural'Image (Found'Length));
       end;
    end AP3P_Reference_Many;

    procedure AP3P_Reference_Bounds (T : in out Fixture) is
       pragma Unreferenced (T);
       Cam : constant Camera_Intrinsics := (100.0, 110.0, 20.0, 30.0);
       Pose_A : constant World_To_Camera_Pose :=
         (Rotation => [others => 0.0], Translation => [0.0, 0.0, 2.0]);
       Pose_B : constant World_To_Camera_Pose :=
         (Rotation => [others => 0.0], Translation => [0.1, -0.2, 1.0]);
       Hypotheses : constant AP3P_Pose_Hypothesis_Array (4 .. 5) :=
         [(Pose => Pose_A), (Pose => Pose_B)];
       World_Ref : constant Object_Point_Array (10 .. 14) :=
         [[1.0, 0.0, 5.0], [-1.0, 2.0, 6.0], [0.5, -0.5, 8.0], [2.0, 1.0, 3.0], [0.0, 0.0, 9.0]];
       Pixels : constant Image_Point_Array (21 .. 25) :=
         Project_Points (World_Ref, Cam, No_Distortion, Pose_A);
       Before_H : constant AP3P_Pose_Hypothesis_Array := Hypotheses;
       Before_W : constant Object_Point_Array := World_Ref;
       Before_P : constant Image_Point_Array := Pixels;
       Values : constant AP3P_Reference_Result_Matrix :=
         Evaluate_AP3P_References (Hypotheses, World_Ref, Pixels, Cam);
       Summaries : constant AP3P_Hypothesis_Reference_Summary_Array :=
         Summarize_AP3P_References (Values);
    begin
       Assert (Hypotheses = Before_H and then World_Ref = Before_W and then Pixels = Before_P,
         "lower-bound inputs immutable");
       Assert (Values'First (1) = 1 and then Values'Last (1) = 2
         and then Values'First (2) = 1 and then Values'Last (2) = 5, "one-based output");
       Assert (Summaries'First = 1 and then Summaries'Last = 2, "one-based summaries");
       for Column in 1 .. 5 loop
          Assert (Values (1, Column).Disposition = Projected_Positive_Depth
            and then Values (1, Column).Error_Pixels < 1.0E-6, "first candidate order");
       end loop;
       Assert (Summaries (1).Entire_Set_Evaluable and then Summaries (2).Total_References = 5,
         "summary candidate order");
    end AP3P_Reference_Bounds;

    procedure AP3P_Reference_Depth (T : in out Fixture) is
       pragma Unreferenced (T);
       Cam : constant Camera_Intrinsics := (100.0, 120.0, 10.0, 20.0);
       Identity : constant World_To_Camera_Pose :=
         (Rotation => [others => 0.0], Translation => [others => 0.0]);
       Shifted : constant World_To_Camera_Pose :=
         (Rotation => [others => 0.0], Translation => [1.0, -2.0, 3.0]);
       Quarter : constant World_To_Camera_Pose :=
         (Rotation => [0.0, 0.0, 1.5707963267948966], Translation => [0.0, 0.0, 1.0]);
       Points : constant Object_Point_Array :=
         [[0.2, -0.3, 5.0], [1.0, 1.0, 0.0], [0.4, -0.2, -2.0]];
       Pixels : constant Image_Point_Array := [[11.0, 12.0], [13.0, 14.0], [15.0, 16.0]];
       Values : constant AP3P_Reference_Result_Matrix :=
         Evaluate_AP3P_References ([(Pose => Identity)], Points, Pixels, Cam);
       Summary : constant AP3P_Hypothesis_Reference_Summary_Array :=
         Summarize_AP3P_References (Values);
    begin
       Assert (Values (1, 1).Disposition = Projected_Positive_Depth
         and then abs (Values (1, 1).Camera_Depth - 5.0) < 1.0E-12, "strict positive");
       Assert (Values (1, 2).Disposition = Non_Positive_Depth
         and then Values (1, 2).Camera_Depth = 0.0
         and then Values (1, 2).Error_Pixels = 0.0, "exact zero not projected");
       Assert (Values (1, 3).Disposition = Non_Positive_Depth
         and then abs (Values (1, 3).Camera_Depth + 2.0) < 1.0E-12, "negative depth");
       Assert (Summary (1).Total_References = 3 and then Summary (1).Projected_References = 1
         and then Summary (1).Non_Positive_Depth_References = 2
         and then Summary (1).Numerically_Unavailable_References = 0
         and then not Summary (1).Entire_Set_Evaluable
         and then Summary (1).Reprojection.Count = 1, "depth summary");
       declare
          Moved : constant Object_Point_Array :=
            [[-0.8, 1.7, 2.0], [0.0, 3.0, -3.0], [-0.6, 1.8, -5.0]];
          Shifted_Values : constant AP3P_Reference_Result_Matrix :=
            Evaluate_AP3P_References ([(Pose => Shifted)], Moved, Pixels, Cam);
          Turned : constant Object_Point_Array := [[-0.2, 0.3, 4.0]];
          Turned_Values : constant AP3P_Reference_Result_Matrix :=
            Evaluate_AP3P_References ([(Pose => Quarter)], Turned, Pixels (1 .. 1), Cam);
       begin
          Assert (abs (Shifted_Values (1, 1).Camera_Depth - 5.0) < 1.0E-12
            and then Shifted_Values (1, 2).Camera_Depth = 0.0
            and then abs (Shifted_Values (1, 3).Camera_Depth + 2.0) < 1.0E-12,
            "added translation is t");
          Assert (abs (Turned_Values (1, 1).Camera_Depth - 5.0) < 1.0E-9,
            "Rz depth uses R, which would be 5 either way; X distinguishes transpose");
          declare
             Predicted : constant Image_Point_Array :=
               Project_Points (Turned, Cam, No_Distortion, Quarter);
          begin
             Assert (abs (Predicted (1) (0) - (Cam.Focal_X * (-0.3) / 5.0 + Cam.Center_X)) < 1.0E-6
               and then abs (Predicted (1) (1) - (Cam.Focal_Y * (-0.2) / 5.0 + Cam.Center_Y)) < 1.0E-6
               and then abs (Turned_Values (1, 1).Error_Pixels
                 - Math.Sqrt ((Predicted (1) (0) - Pixels (1) (0)) ** 2
                              + (Predicted (1) (1) - Pixels (1) (1)) ** 2)) < 1.0E-6,
               "nonidentity rotation is not transposed");
          end;
       end;
    end AP3P_Reference_Depth;

    procedure AP3P_Reference_Distortion (T : in out Fixture) is
       pragma Unreferenced (T);
       Cam : constant Camera_Intrinsics := (800.0, 820.0, 640.0, 360.0);
       Lens : constant Distortion_Coefficients := (-0.12, 0.02, 0.001, -0.0015, 0.01);
       Pose : constant World_To_Camera_Pose :=
         (Rotation => [0.10, -0.05, 0.08], Translation => [0.20, -0.30, 4.00]);
       Points : constant Object_Point_Array :=
         [[1.5, -1.2, 2.0], [-1.4, 1.1, 2.4], [1.2, 0.9, 2.2]];
       function Brown (Point : Object_Point) return Image_Point is
          Camera : constant Object_Point := World_To_Camera_Point (Pose, Point);
          X : constant OpenCV.Float64_Value := Camera (0) / Camera (2);
          Y : constant OpenCV.Float64_Value := Camera (1) / Camera (2);
          R2 : constant OpenCV.Float64_Value := X * X + Y * Y;
          Radial : constant OpenCV.Float64_Value :=
            1.0 + Lens.K1 * R2 + Lens.K2 * R2 * R2 + Lens.K3 * R2 * R2 * R2;
       begin
          return [Cam.Focal_X * (X * Radial + 2.0 * Lens.P1 * X * Y + Lens.P2 * (R2 + 2.0 * X * X))
                    + Cam.Center_X,
                  Cam.Focal_Y * (Y * Radial + Lens.P1 * (R2 + 2.0 * Y * Y) + 2.0 * Lens.P2 * X * Y)
                    + Cam.Center_Y];
       end Brown;
       Distorted : Image_Point_Array (Points'Range);
       Plain : constant Image_Point_Array := Project_Points (Points, Cam, No_Distortion, Pose);
       Native : constant Image_Point_Array := Project_Points (Points, Cam, Lens, Pose);
       Hypothesis : constant AP3P_Pose_Hypothesis_Array := [(Pose => Pose)];
    begin
       for I in Points'Range loop
          Distorted (I) := Brown (Points (I));
          Assert (abs (Native (I) (0) - Distorted (I) (0)) < 1.0E-6
            and then abs (Native (I) (1) - Distorted (I) (1)) < 1.0E-6, "scalar Brown matches native");
          Assert (abs (Native (I) (0) - Plain (I) (0)) > 0.01
            or else abs (Native (I) (1) - Plain (I) (1)) > 0.01, "off-axis distortion is exercised");
       end loop;
       declare
          Values : constant AP3P_Reference_Result_Matrix :=
            Evaluate_AP3P_References (Hypothesis, Points, Distorted, Cam, Lens);
          Zero : constant AP3P_Reference_Result_Matrix :=
            Evaluate_AP3P_References (Hypothesis, Points, Plain, Cam);
          Direct : constant Reprojection_Error_Array :=
            Reprojection_Errors (Points, Distorted, Cam, Lens, Pose);
       begin
          for I in 1 .. 3 loop
             Assert (Values (1, I).Error_Pixels < 1.0E-6, "distorted diagnostic error");
             Assert (abs (Values (1, I).Error_Pixels - Direct (I)) < 1.0E-9, "error API agreement");
             Assert (Zero (1, I).Error_Pixels < 1.0E-6, "zero-distortion control");
          end loop;
       end;
    end AP3P_Reference_Distortion;

    procedure AP3P_Reference_Numbers (T : in out Fixture) is
       pragma Unreferenced (T);
       Cam : constant Camera_Intrinsics := (100.0, 100.0, 0.0, 0.0);
       Identity : constant World_To_Camera_Pose :=
         (Rotation => [others => 0.0], Translation => [others => 0.0]);
       Finite_Point : constant Object_Point_Array := [[0.25, -0.5, 2.0]];
       Predicted : constant Image_Point_Array :=
         Project_Points (Finite_Point, Cam, No_Distortion, Identity);
       Observed : constant Image_Point_Array := [[-1.0E308, 0.0]];
       Residual : constant AP3P_Reference_Result_Matrix :=
         Evaluate_AP3P_References ([(Pose => Identity)], Finite_Point, Observed, Cam);
       Turn : constant World_To_Camera_Pose :=
         (Rotation => [0.0, 0.0, Ada.Numerics.Pi / 4.0], Translation => [0.0, 0.0, 0.0]);
       Batch : Object_Point_Array := [[0.0, 0.0, 4.0], [1.0E308, -1.0E308, 4.0]];
       Batch_Pixels : constant Image_Point_Array := [[0.0, 0.0], [0.0, 0.0]];
    begin
       Batch (2) := [1.0E308 * 1.5, -(1.0E308 * 1.5), 4.0];
       declare
          Mixed : constant AP3P_Reference_Result_Matrix :=
            Evaluate_AP3P_References ([(Pose => Turn)], Batch, Batch_Pixels, Cam);
          Mixed_Summary : constant AP3P_Hypothesis_Reference_Summary_Array :=
            Summarize_AP3P_References (Mixed);
       begin
          Assert (Mixed (1, 1).Disposition = Projected_Positive_Depth
            and then Mixed (1, 1).Camera_Depth > 0.0, "ordinary reference preserved");
          Assert (Mixed (1, 2).Disposition = Numerically_Unavailable
            and then Mixed (1, 2).Camera_Depth = 0.0
            and then Mixed (1, 2).Error_Pixels = 0.0, "rotated opposing coordinates overflow");
          Assert (Mixed_Summary (1).Projected_References = 1
            and then Mixed_Summary (1).Numerically_Unavailable_References = 1
            and then Mixed_Summary (1).Non_Positive_Depth_References = 0
            and then Mixed_Summary (1).Total_References = 2
            and then not Mixed_Summary (1).Entire_Set_Evaluable
            and then Mixed_Summary (1).Reprojection.Count = 1, "summary excludes unavailable reference");
       end;
       Assert ((for all Value of Predicted (1) => Finite (Value)), "native prediction finite");
       Assert ((for all Value of Observed (1) => Finite (Value)), "observed pixel finite");
       Assert (Residual (1, 1).Disposition = Projected_Positive_Depth
         and then Finite (Residual (1, 1).Error_Pixels),
         "representable extreme residual remains projected");
    end AP3P_Reference_Numbers;

    procedure AP3P_Reference_Summary (T : in out Fixture) is
       pragma Unreferenced (T);
       function Item (Depth, Error : OpenCV.Float64_Value) return AP3P_Reference_Result is
          ((Projected_Positive_Depth, Depth, Error));
       Behind : constant AP3P_Reference_Result := (Non_Positive_Depth, -1.0, 0.0);
       Missing : constant AP3P_Reference_Result := (Numerically_Unavailable, 0.0, 0.0);
       Pair : constant AP3P_Reference_Result_Matrix :=
         [[Item (2.0, 5.0), Item (3.0, 12.0)]];
       Expected : constant Reprojection_Summary := Summarize_Reprojection ([5.0, 12.0]);
       Mixed_Depth : constant AP3P_Reference_Result_Matrix := [[Item (2.0, 5.0), Behind]];
       Mixed_Number : constant AP3P_Reference_Result_Matrix := [[Item (2.0, 5.0), Missing]];
       All_Behind : constant AP3P_Reference_Result_Matrix := [[Behind, (Non_Positive_Depth, 0.0, 0.0)]];
       Empty_Columns : constant AP3P_Reference_Result_Matrix (1 .. 1, 1 .. 0) :=
         [others => [others => Missing]];
       procedure Reject (Values : AP3P_Reference_Result_Matrix) is
       begin
          declare
             Summary : constant AP3P_Hypothesis_Reference_Summary_Array :=
               Summarize_AP3P_References (Values);
             pragma Unreferenced (Summary);
          begin
             Assert (False, "malformed reference summary accepted");
          end;
       exception
          when OpenCV.OpenCV_Error => null;
       end Reject;
       function RMS (Values : AP3P_Reference_Result_Matrix) return AP3P_Hypothesis_Reference_Summary is
          (Summarize_AP3P_References (Values) (1));
    begin
       Assert (abs (Expected.RMS_Error_Pixels - Math.Sqrt ((25.0 + 144.0) / 2.0)) < 1.0E-12
         and then Expected.Maximum_Error_Pixels = 12.0, "analytic RMS");
       Assert (RMS (Pair).Reprojection = Expected and then RMS (Pair).Entire_Set_Evaluable,
         "projected pair uses Summarize_Reprojection");
       Assert (RMS (Mixed_Depth).Projected_References = 1
         and then RMS (Mixed_Depth).Non_Positive_Depth_References = 1
         and then not RMS (Mixed_Depth).Entire_Set_Evaluable
         and then RMS (Mixed_Depth).Reprojection.RMS_Error_Pixels = 5.0, "subset RMS");
       Assert (RMS (Mixed_Number).Numerically_Unavailable_References = 1
         and then RMS (Mixed_Number).Reprojection.Count = 1, "unavailable excluded");
       Assert (RMS (All_Behind).Projected_References = 0
         and then RMS (All_Behind).Reprojection.Count = 0
         and then RMS (All_Behind).Reprojection.RMS_Error_Pixels = 0.0
         and then not RMS (All_Behind).Entire_Set_Evaluable, "no projected evidence");
       Assert (RMS (Empty_Columns).Total_References = 0 and then not RMS (Empty_Columns).Entire_Set_Evaluable,
         "empty references are not success");
       declare
          Tiny : constant AP3P_Reference_Result_Matrix :=
            [[Item (1.0, 1.0E-200), Item (1.0, 1.0E-200)]];
          Large : constant AP3P_Reference_Result_Matrix :=
            [[Item (1.0, OpenCV.Float64_Value'Last), Item (1.0, OpenCV.Float64_Value'Last)]];
       begin
          Assert (RMS (Tiny).Reprojection = Summarize_Reprojection ([1.0E-200, 1.0E-200])
            and then RMS (Large).Reprojection.Maximum_Error_Pixels = OpenCV.Float64_Value'Last,
            "scaled summary extremes");
       end;
       Reject ([[(Projected_Positive_Depth, 0.0, 1.0)]]);
       Reject ([[(Projected_Positive_Depth, 1.0, -1.0)]]);
       Reject ([[(Non_Positive_Depth, 1.0, 0.0)]]);
       Reject ([[(Non_Positive_Depth, -1.0, 2.0)]]);
       Reject ([[(Numerically_Unavailable, 1.0, 0.0)]]);
       Reject ([[(Numerically_Unavailable, 0.0, 1.0)]]);
    end AP3P_Reference_Summary;

    procedure AP3P_Reference_Empty (T : in out Fixture) is
       pragma Unreferenced (T);
       pragma Suppress (Validity_Check);
       Cam : constant Camera_Intrinsics := (800.0, 820.0, 640.0, 360.0);
       Pose : constant World_To_Camera_Pose :=
         (Rotation => [others => 0.0], Translation => [0.0, 0.0, 1.0]);
       None_H : constant AP3P_Pose_Hypothesis_Array (1 .. 0) := [others => <>];
       None_W : constant Object_Point_Array (1 .. 0) := [others => [others => 0.0]];
       None_P : constant Image_Point_Array (1 .. 0) := [others => [others => 0.0]];
       Some_W : constant Object_Point_Array := [[0.1, 0.2, 3.0]];
       Some_P : constant Image_Point_Array := [[1.0, 2.0]];
       Some_H : constant AP3P_Pose_Hypothesis_Array := [(Pose => Pose)];
       Native_Zero : constant AP3P_Pose_Hypothesis_Array := Solve_AP3P_Hypotheses
         ([[-0.39081598340561996, 0.32617277568950875, 4.0131477606213553],
           [-1.6067503815809268, 0.73284013236088175, 2.4253925955828199],
           [1.9483108946655348, -1.1457818430811084, 7.0007856567437514]],
          [[118.67887557148487, 129.79777044444734],
           [452.33705169300674, 427.73531562095332],
           [690.96005298599528, 241.045602436595]], Cam);
       procedure Reject (Hypotheses : AP3P_Pose_Hypothesis_Array;
                         World_Ref : Object_Point_Array; Pixels : Image_Point_Array;
                         Camera : Camera_Intrinsics := Cam) is
       begin
          declare
             Values : constant AP3P_Reference_Result_Matrix :=
               Evaluate_AP3P_References (Hypotheses, World_Ref, Pixels, Camera);
             pragma Unreferenced (Values);
          begin
             Assert (False, "invalid AP3P reference input accepted");
          end;
       exception
          when OpenCV.OpenCV_Error => null;
       end Reject;
    begin
       declare
          Empty : constant AP3P_Reference_Result_Matrix :=
            Evaluate_AP3P_References (None_H, None_W, None_P, Cam);
          Rows : constant AP3P_Reference_Result_Matrix :=
            Evaluate_AP3P_References (None_H, Some_W, Some_P, Cam);
          Columns : constant AP3P_Reference_Result_Matrix :=
            Evaluate_AP3P_References (Some_H, None_W, None_P, Cam);
          Native : constant AP3P_Reference_Result_Matrix :=
            Evaluate_AP3P_References (Native_Zero, Some_W, Some_P, Cam);
       begin
          Assert (Empty'Length (1) = 0 and then Empty'Length (2) = 0, "0x0");
          Assert (Rows'Length (1) = 0 and then Rows'Length (2) = 1, "0xN");
          Assert (Columns'Length (1) = 1 and then Columns'Length (2) = 0, "Mx0");
          Assert (Native_Zero'Length = 0 and then Native'Length (1) = 0
            and then Native'Length (2) = 1, "real native zero remains empty");
          Assert (Summarize_AP3P_References (Rows)'Length = 0, "empty summary");
       end;
       Reject ([(Pose => (Rotation => [OpenCV.Float64_Value (Nonfinite (0)), 0.0, 0.0],
                          Translation => [others => 0.0]))], Some_W, Some_P);
       Reject (Some_H, [[OpenCV.Float64_Value (Nonfinite (1)), 0.0, 1.0]], Some_P);
       Reject (Some_H, Some_W, [[0.0, OpenCV.Float64_Value (Nonfinite (0))]]);
       Reject (Some_H, Some_W, Some_P, (0.0, 820.0, 640.0, 360.0));
       Reject (Some_H, Some_W, Some_P, (800.0, 820.0, OpenCV.Float64_Value (Nonfinite (0)), 360.0));
       Reject (Some_H, Some_W (1 .. 1) & Some_W (1 .. 1), Some_P);
       begin
          declare
             Values : constant AP3P_Reference_Result_Matrix :=
               Evaluate_AP3P_References (Some_H, Some_W, Some_P, Cam,
                 (OpenCV.Float64_Value (Nonfinite (0)), 0.0, 0.0, 0.0, 0.0));
             pragma Unreferenced (Values);
          begin
             Assert (False, "nonfinite distortion accepted");
          end;
       exception
          when OpenCV.OpenCV_Error => null;
       end;
       declare
          Five : constant AP3P_Pose_Hypothesis_Array (1 .. 5) :=
            [others => (Pose => Pose)];
       begin
          Reject (Five, Some_W, Some_P);
       end;
    end AP3P_Reference_Empty;



   procedure AP3P_Select_Known (T : in out Fixture) is
      pragma Unreferenced (T);
      function Rotation_Y (Angle : OpenCV.Float64_Value) return Rotation_Matrix is
         C : constant OpenCV.Float64_Value := Math.Cos (Angle);
         S : constant OpenCV.Float64_Value := Math.Sin (Angle);
      begin
         return [[C, 0.0, S], [0.0, 1.0, 0.0], [-S, 0.0, C]];
      end Rotation_Y;
      function Rotation_Z (Angle : OpenCV.Float64_Value) return Rotation_Matrix is
         C : constant OpenCV.Float64_Value := Math.Cos (Angle);
         S : constant OpenCV.Float64_Value := Math.Sin (Angle);
      begin
         return [[C, -S, 0.0], [S, C, 0.0], [0.0, 0.0, 1.0]];
      end Rotation_Z;
      function Multiply (Left, Right : Rotation_Matrix) return Rotation_Matrix is
         Result : Rotation_Matrix := [others => [others => 0.0]];
      begin
         for Row in 0 .. 2 loop
            for Column in 0 .. 2 loop
               for K in 0 .. 2 loop
                  Result (Row, Column) :=
                    Result (Row, Column) + Left (Row, K) * Right (K, Column);
               end loop;
            end loop;
         end loop;
         return Result;
      end Multiply;
      function Apply (Matrix : Rotation_Matrix; Point : Object_Point) return Object_Point is
        ([Matrix (0, 0) * Point (0) + Matrix (0, 1) * Point (1) + Matrix (0, 2) * Point (2),
         Matrix (1, 0) * Point (0) + Matrix (1, 1) * Point (1) + Matrix (1, 2) * Point (2),
         Matrix (2, 0) * Point (0) + Matrix (2, 1) * Point (1) + Matrix (2, 2) * Point (2)]);
      function Transpose (Matrix : Rotation_Matrix) return Rotation_Matrix is
        ([[Matrix (0, 0), Matrix (1, 0), Matrix (2, 0)],
          [Matrix (0, 1), Matrix (1, 1), Matrix (2, 1)],
          [Matrix (0, 2), Matrix (1, 2), Matrix (2, 2)]]);
      Truth_R : constant Rotation_Matrix := Multiply (Rotation_Z (0.22), Rotation_Y (0.35));
      Truth_T : constant Translation_Vector := [0.2, -0.3, 0.8];
      Calibration : constant Camera_Intrinsics := (800.0, 820.0, 640.0, 360.0);
      Seeds : constant Object_Point_Array :=
        [[0.3, -0.2, 3.5], [1.4, 0.1, 4.0], [-0.5, 1.2, 5.3]];
      Reference : constant Object_Point := [0.7, -0.8, 6.1];

      function Project (Point : Object_Point) return Image_Point is
         Camera : constant Object_Point :=
           [Apply (Truth_R, Point) (0) + Truth_T (0),
            Apply (Truth_R, Point) (1) + Truth_T (1),
            Apply (Truth_R, Point) (2) + Truth_T (2)];
      begin
         return [Calibration.Focal_X * (Camera (0) / Camera (2)) + Calibration.Center_X,
                 Calibration.Focal_Y * (Camera (1) / Camera (2)) + Calibration.Center_Y];
      end Project;
      Seed_Pixels : Image_Point_Array (1 .. 3);
      Reference_Pixels : constant Image_Point_Array := [Project (Reference)];
      Policy : constant AP3P_Selection_Options := (1.0E-6, 1.0E-6, 1.0);
   begin
      for I in Seeds'Range loop
         Seed_Pixels (I) := Project (Seeds (I));
      end loop;
      declare
         Found : constant AP3P_Pose_Hypothesis_Array :=
           Solve_AP3P_Hypotheses (Seeds, Seed_Pixels, Calibration);
         Evaluated : constant AP3P_Reference_Result_Matrix :=
           Evaluate_AP3P_References (Found, [Reference], Reference_Pixels, Calibration);
         Chosen : constant AP3P_Selection_Result :=
           Select_AP3P_Hypothesis (Evaluated, Policy);
         Direct : constant AP3P_Pose_Selection :=
           Select_AP3P_Pose
             (Seeds, Seed_Pixels, [Reference], Reference_Pixels, Calibration,
              No_Distortion, Policy);
         Index : constant Positive := Selected_Hypothesis_Index (Chosen);
         Pose : constant World_To_Camera_Pose := Selected_Pose (Direct);
         Matrix_Pose : constant World_To_Camera_Pose :=
           Selected_Pose (Chosen, Found, Index);
         Recovered : constant Rotation_Matrix := Rotation_Matrix_Of (Pose);
         Expected_Center : constant Object_Point :=
           [-Apply (Transpose (Truth_R), Truth_T) (0),
            -Apply (Transpose (Truth_R), Truth_T) (1),
            -Apply (Transpose (Truth_R), Truth_T) (2)];
         Center : constant Object_Point := Camera_Center (Pose);
         Trace : OpenCV.Float64_Value := 0.0;
      begin
         Assert (Selected (Chosen)
           and then Chosen.Disposition = Unique_Selected_Hypothesis,
           "fourth reference uniquely selects");
         Assert (Disposition (Direct) = Unique_Selected_Hypothesis
           and then Selected (Direct),
           "end-to-end selection owns a unique pose");
         Assert (Chosen.Candidates (Index).Eligible
           and then Chosen.Candidates (Index).Rank = 1,
           "selected candidate is rank one");
         for Axis in 0 .. 2 loop
            Trace := Trace + Recovered (Axis, 0) * Truth_R (Axis, 0)
              + Recovered (Axis, 1) * Truth_R (Axis, 1)
              + Recovered (Axis, 2) * Truth_R (Axis, 2);
         end loop;
         Assert (Math.Arccos (OpenCV.Float64_Value'Max
                   (-1.0, OpenCV.Float64_Value'Min (1.0, (Trace - 1.0) / 2.0))) < 1.0E-6,
           "selected rotation matches geometric oracle");
         Assert (Math.Sqrt ((Pose.Translation (0) - Truth_T (0)) ** 2
                   + (Pose.Translation (1) - Truth_T (1)) ** 2
                   + (Pose.Translation (2) - Truth_T (2)) ** 2) < 1.0E-6,
           "selected translation matches geometric oracle");
         Assert (Math.Sqrt ((Center (0) - Expected_Center (0)) ** 2
                   + (Center (1) - Expected_Center (1)) ** 2
                   + (Center (2) - Expected_Center (2)) ** 2) < 1.0E-6,
           "selected camera center is -R^T t");
         declare
            Other : constant Rotation_Matrix := Rotation_Matrix_Of (Matrix_Pose);
            Cross : OpenCV.Float64_Value := 0.0;
         begin
            for Axis in 0 .. 2 loop
               Cross := Cross + Recovered (Axis, 0) * Other (Axis, 0)
                 + Recovered (Axis, 1) * Other (Axis, 1)
                 + Recovered (Axis, 2) * Other (Axis, 2);
            end loop;
            Assert (Math.Arccos (OpenCV.Float64_Value'Max
                      (-1.0, OpenCV.Float64_Value'Min (1.0, (Cross - 1.0) / 2.0))) < 1.0E-5
              and then Math.Sqrt ((Pose.Translation (0) - Matrix_Pose.Translation (0)) ** 2
                + (Pose.Translation (1) - Matrix_Pose.Translation (1)) ** 2
                + (Pose.Translation (2) - Matrix_Pose.Translation (2)) ** 2) < 1.0E-5,
              "owned pose matches the same-evaluation matrix pose geometrically");
         end;
         declare
            Predicted : constant Image_Point_Array :=
              Project_Points ([Reference], Calibration, No_Distortion, Pose);
         begin
            Assert (Math.Sqrt ((Predicted (1) (0) - Reference_Pixels (1) (0)) ** 2
                      + (Predicted (1) (1) - Reference_Pixels (1) (1)) ** 2) < 1.0E-6,
              "selected pose reprojects the independent reference");
         end;
         Ada.Text_IO.Put_Line ("AP3P selection index=" & Positive'Image (Index)
           & " hypotheses=" & Natural'Image (Found'Length)
           & " rms=" & OpenCV.Float64_Value'Image (Chosen.Candidates (Index).RMS_Error_Pixels));
      end;
   end AP3P_Select_Known;

   procedure AP3P_Select_Distortion (T : in out Fixture) is
      pragma Unreferenced (T);
      function Rotation_Y (Angle : OpenCV.Float64_Value) return Rotation_Matrix is
         C : constant OpenCV.Float64_Value := Math.Cos (Angle);
         S : constant OpenCV.Float64_Value := Math.Sin (Angle);
      begin
         return [[C, 0.0, S], [0.0, 1.0, 0.0], [-S, 0.0, C]];
      end Rotation_Y;
      function Rotation_Z (Angle : OpenCV.Float64_Value) return Rotation_Matrix is
         C : constant OpenCV.Float64_Value := Math.Cos (Angle);
         S : constant OpenCV.Float64_Value := Math.Sin (Angle);
      begin
         return [[C, -S, 0.0], [S, C, 0.0], [0.0, 0.0, 1.0]];
      end Rotation_Z;
      function Multiply (Left, Right : Rotation_Matrix) return Rotation_Matrix is
         Result : Rotation_Matrix := [others => [others => 0.0]];
      begin
         for Row in 0 .. 2 loop
            for Column in 0 .. 2 loop
               for K in 0 .. 2 loop
                  Result (Row, Column) :=
                    Result (Row, Column) + Left (Row, K) * Right (K, Column);
               end loop;
            end loop;
         end loop;
         return Result;
      end Multiply;
      function Apply (Matrix : Rotation_Matrix; Point : Object_Point) return Object_Point is
        ([Matrix (0, 0) * Point (0) + Matrix (0, 1) * Point (1) + Matrix (0, 2) * Point (2),
         Matrix (1, 0) * Point (0) + Matrix (1, 1) * Point (1) + Matrix (1, 2) * Point (2),
         Matrix (2, 0) * Point (0) + Matrix (2, 1) * Point (1) + Matrix (2, 2) * Point (2)]);
      Truth_R : constant Rotation_Matrix := Multiply (Rotation_Z (0.22), Rotation_Y (0.35));
      Truth_T : constant Translation_Vector := [0.2, -0.3, 0.8];
      Calibration : constant Camera_Intrinsics := (800.0, 820.0, 640.0, 360.0);
      Lens : constant Distortion_Coefficients := (-0.12, 0.03, 0.001, -0.002, 0.01);
      Seeds : constant Object_Point_Array :=
        [[0.3, -0.2, 3.5], [1.4, 0.1, 4.0], [-0.5, 1.2, 5.3]];
      References : constant Object_Point_Array :=
        [[0.7, -0.8, 6.1], [1.1, 0.4, 5.2], [-0.9, -0.6, 4.4],
         [0.2, 1.3, 7.0], [-1.5, 0.5, 3.8]];
      function Brown (Point : Object_Point) return Image_Point is
         Camera : constant Object_Point :=
           [Apply (Truth_R, Point) (0) + Truth_T (0),
            Apply (Truth_R, Point) (1) + Truth_T (1),
            Apply (Truth_R, Point) (2) + Truth_T (2)];
         X : constant OpenCV.Float64_Value := Camera (0) / Camera (2);
         Y : constant OpenCV.Float64_Value := Camera (1) / Camera (2);
         R2 : constant OpenCV.Float64_Value := X * X + Y * Y;
         Radial : constant OpenCV.Float64_Value :=
           1.0 + Lens.K1 * R2 + Lens.K2 * R2 * R2 + Lens.K3 * R2 * R2 * R2;
      begin
         return [Calibration.Focal_X * (X * Radial + 2.0 * Lens.P1 * X * Y
                   + Lens.P2 * (R2 + 2.0 * X * X)) + Calibration.Center_X,
                 Calibration.Focal_Y * (Y * Radial + Lens.P1 * (R2 + 2.0 * Y * Y)
                   + 2.0 * Lens.P2 * X * Y) + Calibration.Center_Y];
      end Brown;
      Seed_Pixels : Image_Point_Array (1 .. 3);
      Reference_Pixels : Image_Point_Array (1 .. References'Length);
      Policy : constant AP3P_Selection_Options := (1.0E-3, 1.0E-3, 1.0);
   begin
      for I in Seeds'Range loop
         Seed_Pixels (I) := Brown (Seeds (I));
      end loop;
      for I in References'Range loop
         Reference_Pixels (I) := Brown (References (I));
      end loop;
      declare
         Chosen : constant AP3P_Pose_Selection :=
           Select_AP3P_Pose
             (Seeds, Seed_Pixels, References, Reference_Pixels, Calibration, Lens, Policy);
         Pose : constant World_To_Camera_Pose := Selected_Pose (Chosen);
         Qualified : constant AP3P_Candidate_Qualification_Array := Candidates (Chosen);
         Native : constant Image_Point_Array :=
           Project_Points (References, Calibration, Lens, Pose);
         Maximum : OpenCV.Float64_Value := 0.0;
      begin
         Assert (Disposition (Chosen) = Unique_Selected_Hypothesis
           and then Qualified (Candidate_Index (Chosen)).Eligible
           and then Qualified (Candidate_Index (Chosen)).Rank = 1,
           "distorted references uniquely select");
         for I in Native'Range loop
            Maximum := OpenCV.Float64_Value'Max
              (Maximum, Math.Sqrt ((Native (I) (0) - Reference_Pixels (I) (0)) ** 2
                 + (Native (I) (1) - Reference_Pixels (I) (1)) ** 2));
         end loop;
         Assert (Maximum < 1.0E-3, "selected pose matches independent Brown projection");
         Assert ((for some Item of Qualified => not Item.Eligible)
           or else Qualified'Length = 1,
           "an alternate hypothesis is not also the unique winner unless alone");
         Ada.Text_IO.Put_Line ("AP3P distorted selection rms="
           & OpenCV.Float64_Value'Image
             (Qualified (Candidate_Index (Chosen)).RMS_Error_Pixels)
           & " max=" & OpenCV.Float64_Value'Image (Maximum));
      end;
   end AP3P_Select_Distortion;

   procedure AP3P_Select_Ambiguous (T : in out Fixture) is
      pragma Unreferenced (T);
      function Item (Maximum : OpenCV.Float64_Value) return AP3P_Reference_Result is
        (Projected_Positive_Depth, 4.0, Maximum);
      Policy : constant AP3P_Selection_Options := (10.0, 20.0, 1.0);
      Close : constant AP3P_Reference_Result_Matrix :=
        [[Item (1.0)], [Item (1.5)]];
      Tied : constant AP3P_Reference_Result_Matrix :=
        [[Item (2.0)], [Item (2.0)], [Item (9.0)]];
      Exact_Gap : constant AP3P_Reference_Result_Matrix :=
        [[Item (1.0)], [Item (2.0)]];
      Separated : constant AP3P_Reference_Result_Matrix :=
        [[Item (4.0)], [Item (1.0)]];
      function RMS_Only (Value : OpenCV.Float64_Value) return AP3P_Reference_Result_Matrix is
        ([[Item (Value)]]);
   begin
      declare
         Result : constant AP3P_Selection_Result := Select_AP3P_Hypothesis (Close, Policy);
      begin
         Assert (Result.Disposition = Ambiguous_Qualified_Hypotheses
           and then not Selected (Result) and then Result.Candidate_Index = 0
           and then Result.Separation_Pixels = 0.5
           and then Result.Candidates (1).Rank = 1
           and then Result.Candidates (2).Rank = 2,
           "insufficient RMS separation is ambiguous");
      end;
      declare
         Result : constant AP3P_Selection_Result := Select_AP3P_Hypothesis (Tied, Policy);
      begin
         Assert (Result.Disposition = Ambiguous_Qualified_Hypotheses
           and then Result.Separation_Pixels = 0.0
           and then Result.Candidates (1).Rank = 1
           and then Result.Candidates (2).Rank = 2
           and then Result.Candidates (3).Rank = 3,
           "exact RMS ties keep original order and stay ambiguous");
      end;
      declare
         Result : constant AP3P_Selection_Result :=
           Select_AP3P_Hypothesis (Exact_Gap, (10.0, 20.0, 1.0));
      begin
         Assert (Result.Disposition = Ambiguous_Qualified_Hypotheses
           and then Result.Separation_Pixels = 1.0,
           "separation equal to the minimum is not strict");
      end;
      declare
         Result : constant AP3P_Selection_Result :=
           Select_AP3P_Hypothesis (Separated, Policy);
      begin
         Assert (Result.Disposition = Unique_Selected_Hypothesis
           and then Selected_Hypothesis_Index (Result) = 2
           and then Result.Separation_Pixels = 3.0
           and then Result.Candidates (2).Rank = 1
           and then Result.Candidates (1).Rank = 2,
           "strict separation selects the lower RMS regardless of order");
      end;
      declare
         Large : constant AP3P_Selection_Result :=
           Select_AP3P_Hypothesis (RMS_Only (1.0E308), (OpenCV.Float64_Value'Last, OpenCV.Float64_Value'Last, 0.0));
      begin
         Assert (Large.Disposition = Unique_Selected_Hypothesis
           and then Large.Candidates (1).RMS_Error_Pixels = 1.0E308,
           "a huge finite eligible RMS is retained");
      end;
   end AP3P_Select_Ambiguous;

   procedure AP3P_Select_Empty (T : in out Fixture) is
      pragma Unreferenced (T);
      Policy : constant AP3P_Selection_Options := (1.0, 1.0, 0.0);
      None : constant AP3P_Reference_Result_Matrix (1 .. 0, 1 .. 1) :=
        [others => [others => (Numerically_Unavailable, 0.0, 0.0)]];
      No_References : constant AP3P_Reference_Result_Matrix (1 .. 2, 1 .. 0) :=
        [others => [others => (Numerically_Unavailable, 0.0, 0.0)]];
      Rejected : constant AP3P_Reference_Result_Matrix :=
        [[(Projected_Positive_Depth, 3.0, 5.0)],
         [(Non_Positive_Depth, 0.0, 0.0)],
         [(Numerically_Unavailable, 0.0, 0.0)]];
      Seeds : constant Object_Point_Array :=
        [[-0.39081598340561996, 0.32617277568950875, 4.0131477606213553],
         [-1.6067503815809268, 0.73284013236088175, 2.4253925955828199],
         [1.9483108946655348, -1.1457818430811084, 7.0007856567437514]];
      Pixels : constant Image_Point_Array :=
        [[118.67887557148487, 129.79777044444734],
         [452.33705169300674, 427.73531562095332],
         [690.96005298599528, 241.045602436595]];
      Calibration : constant Camera_Intrinsics := (800.0, 820.0, 640.0, 360.0);
   begin
      Assert (Select_AP3P_Hypothesis (None, Policy).Disposition = No_Hypotheses,
        "zero hypothesis rows are No_Hypotheses");
      Assert (Select_AP3P_Hypothesis (No_References, Policy).Disposition
        = No_Independent_References, "zero references are not a selection");
      declare
         Result : constant AP3P_Selection_Result :=
           Select_AP3P_Hypothesis (Rejected, (1.0, 1.0, 0.0));
      begin
         Assert (Result.Disposition = No_Eligible_Hypotheses
           and then (for all Item of Result.Candidates => not Item.Eligible),
           "error, depth and unavailable rows are all ineligible");
      end;
      declare
         Direct : constant AP3P_Pose_Selection :=
           Select_AP3P_Pose
             (Seeds, Pixels, [[0.2, -0.4, 8.0]], [[1.0, 2.0]], Calibration,
              No_Distortion, Policy);
      begin
         Assert (Disposition (Direct) = No_Hypotheses
           and then Candidates (Direct)'Length = 0
           and then not Selected (Direct),
           "native zero hypotheses stay No_Hypotheses");
         declare
            Pose : constant World_To_Camera_Pose := Selected_Pose (Direct);
         begin
            Assert (False, "no-hypothesis selection exposed " & OpenCV.Float64_Value'Image (Pose.Translation (2)));
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
      declare
         Empty_References : constant AP3P_Pose_Selection :=
           Select_AP3P_Pose
             (Seeds, Pixels, [1 .. 0 => [0.0, 0.0, 1.0]], [1 .. 0 => [0.0, 0.0]],
              Calibration, No_Distortion, Policy);
      begin
         Assert (Disposition (Empty_References) = No_Independent_References,
           "an empty reference set is classified before the solver");
         declare
            Pose : constant World_To_Camera_Pose := Selected_Pose (Empty_References);
         begin
            Assert (False, "empty references exposed " & OpenCV.Float64_Value'Image (Pose.Translation (2)));
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
   end AP3P_Select_Empty;

   procedure AP3P_Select_Depth (T : in out Fixture) is
      pragma Unreferenced (T);
      Calibration : constant Camera_Intrinsics := (100.0, 110.0, 20.0, 30.0);
      Good : constant World_To_Camera_Pose :=
        (Rotation => [others => 0.0], Translation => [0.0, 0.0, 2.0]);
      Behind : constant World_To_Camera_Pose :=
        (Rotation => [others => 0.0], Translation => [0.0, 0.0, -4.0]);
      Hypotheses : constant AP3P_Pose_Hypothesis_Array :=
        [(Pose => Good), (Pose => Behind)];
      World_Ref : constant Object_Point_Array := [[0.2, -0.3, 3.0]];
      Pixels : constant Image_Point_Array :=
        Project_Points (World_Ref, Calibration, No_Distortion, Good);
      Policy : constant AP3P_Selection_Options := (1.0E-6, 1.0E-6, 0.0);
      Evaluated : constant AP3P_Reference_Result_Matrix :=
        Evaluate_AP3P_References (Hypotheses, World_Ref, Pixels, Calibration);
      Chosen : constant AP3P_Selection_Result :=
        Select_AP3P_Hypothesis (Evaluated, Policy);
   begin
      Assert (Evaluated (2, 1).Disposition = Non_Positive_Depth, "behind camera classified");
      Assert (Chosen.Disposition = Unique_Selected_Hypothesis
        and then Selected_Hypothesis_Index (Chosen) = 1
        and then not Chosen.Candidates (2).Eligible,
        "nonpositive depth removes a candidate");
      Assert (Selected_Pose (Chosen, Hypotheses, 1) = Good, "front pose retained");
   end AP3P_Select_Depth;

   procedure AP3P_Select_Unavailable (T : in out Fixture) is
      pragma Unreferenced (T);
      Angle : constant OpenCV.Float64_Value := Ada.Numerics.Pi / 4.0;
      Pose : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, Angle], Translation => [others => 0.0]);
      Hypotheses : constant AP3P_Pose_Hypothesis_Array := [(Pose => Pose)];
      World_Ref : constant Object_Point_Array :=
        [[0.0, 0.0, 4.0], [1.5E308, -1.5E308, 4.0]];
      Pixels : constant Image_Point_Array := [[10.0, 20.0], [30.0, 40.0]];
      Evaluated : constant AP3P_Reference_Result_Matrix :=
        Evaluate_AP3P_References (Hypotheses, World_Ref, Pixels, (100.0, 100.0, 0.0, 0.0));
      Chosen : constant AP3P_Selection_Result :=
        Select_AP3P_Hypothesis (Evaluated, (1.0E6, 1.0E6, 0.0));
   begin
      Assert (Evaluated (1, 2).Disposition = Numerically_Unavailable,
        "camera-sum overflow remains unavailable");
      Assert (Chosen.Disposition = No_Eligible_Hypotheses
        and then not Chosen.Candidates (1).Eligible
        and then Chosen.Candidates (1).RMS_Error_Pixels = 0.0,
        "one unavailable reference rejects the whole hypothesis");
   end AP3P_Select_Unavailable;

   procedure AP3P_Select_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check);
      Row : constant AP3P_Reference_Result_Matrix :=
        [[(Projected_Positive_Depth, 2.0, 0.0)]];
      Seeds : constant Object_Point_Array := [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
      Pixels : constant Image_Point_Array := [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]];
      procedure Reject_Options (Value : AP3P_Selection_Options) is
      begin
         declare
            Result : constant AP3P_Selection_Result := Select_AP3P_Hypothesis (Row, Value);
         begin
            Assert (False, "invalid options accepted " & AP3P_Selection_Disposition'Image (Result.Disposition));
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Reject_Options;
      procedure Reject_Pose
        (References : Object_Point_Array; Observed : Image_Point_Array) is
      begin
         declare
            Result : constant AP3P_Pose_Selection :=
              Select_AP3P_Pose (Seeds, Pixels, References, Observed,
                (100.0, 100.0, 0.0, 0.0), No_Distortion, (1.0, 1.0, 0.0));
         begin
            Assert (False, "invalid references accepted " & Natural'Image (Candidates (Result)'Length));
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Reject_Pose;
   begin
      Reject_Options ((-1.0, 1.0, 0.0));
      Reject_Options ((1.0, -0.1, 0.0));
      Reject_Options ((1.0, 1.0, -1.0E-12));
      Reject_Options ((OpenCV.Float64_Value (Nonfinite (0)), 1.0, 0.0));
      Reject_Options ((1.0, OpenCV.Float64_Value (Nonfinite (1)), 0.0));
      Reject_Options ((1.0, 1.0, OpenCV.Float64_Value (Nonfinite (0))));
      Reject_Pose ([Seeds (1)], [[1.0, 2.0]]);
      Reject_Pose ([[0.2, 0.3, 4.0], [0.2, 0.3, 4.0]], [[1.0, 2.0], [3.0, 4.0]]);
      Reject_Pose ([[0.2, 0.3, 4.0]], [[1.0, 2.0], [3.0, 4.0]]);
      Reject_Pose ([[0.2, 0.3, 4.0]], [1 .. 0 => [0.0, 0.0]]);
      declare
         Eligible : constant AP3P_Selection_Result :=
           Select_AP3P_Hypothesis (Row, (0.0, 0.0, 0.0));
         Rejected : constant AP3P_Reference_Result_Matrix :=
           [[(Projected_Positive_Depth, 2.0, 0.1)]];
         Chosen : constant AP3P_Selection_Result :=
           Select_AP3P_Hypothesis (Rejected, (0.0, 0.0, 0.0));
      begin
         Assert (Eligible.Disposition = Unique_Selected_Hypothesis,
           "exact zero error meets an inclusive zero threshold");
         Assert (not Selected (Chosen), "positive error exceeds a zero threshold");
         declare
            Index : constant Positive := Selected_Hypothesis_Index (Chosen);
         begin
            Assert (False, "accessor accepted " & Positive'Image (Index));
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
   end AP3P_Select_Invalid;

   procedure AP3P_Select_Bounds (T : in out Fixture) is
      pragma Unreferenced (T);
      Calibration : constant Camera_Intrinsics := (800.0, 820.0, 640.0, 360.0);
      Truth : constant World_To_Camera_Pose :=
        (Rotation => [0.10, -0.05, 0.08], Translation => [0.20, -0.10, 6.00]);
      Seeds : constant Object_Point_Array (7 .. 9) :=
        [[-1.0, -1.0, 0.0], [1.0, -0.4, 0.6], [0.2, 1.1, 1.4]];
      References : constant Object_Point_Array (20 .. 22) :=
        [[-0.6, 0.4, 2.2], [1.3, 0.8, 1.7], [0.4, -1.2, 2.8]];
      Seed_Pixels : constant Image_Point_Array (30 .. 32) :=
        Project_Points (Seeds, Calibration, No_Distortion, Truth);
      Reference_Pixels : constant Image_Point_Array (40 .. 42) :=
        Project_Points (References, Calibration, No_Distortion, Truth);
      Policy : constant AP3P_Selection_Options := (1.0E-6, 1.0E-6, 1.0E-3);
      Chosen : constant AP3P_Pose_Selection :=
        Select_AP3P_Pose
          (Seeds, Seed_Pixels, References, Reference_Pixels, Calibration,
           No_Distortion, Policy);
      Pose : constant World_To_Camera_Pose := Selected_Pose (Chosen);
      Recovered : constant Rotation_Matrix := Rotation_Matrix_Of (Pose);
      Expected : constant Rotation_Matrix := Rotation_Matrix_Of (Truth);
      Trace : OpenCV.Float64_Value := 0.0;
   begin
      Assert (Seeds'First = 7 and then References'First = 20
        and then Seed_Pixels'First = 30, "fixture lower bounds differ");
      Assert (Disposition (Chosen) = Unique_Selected_Hypothesis, "shifted bounds still select");
      for Axis in 0 .. 2 loop
         Trace := Trace + Recovered (Axis, 0) * Expected (Axis, 0)
           + Recovered (Axis, 1) * Expected (Axis, 1)
           + Recovered (Axis, 2) * Expected (Axis, 2);
      end loop;
      Assert (Math.Arccos (OpenCV.Float64_Value'Max
                (-1.0, OpenCV.Float64_Value'Min (1.0, (Trace - 1.0) / 2.0))) < 1.0E-6,
        "shifted-bound selection matches the known rotation");
      Assert (Math.Sqrt ((Camera_Center (Pose) (0) - Camera_Center (Truth) (0)) ** 2
                + (Camera_Center (Pose) (1) - Camera_Center (Truth) (1)) ** 2
                + (Camera_Center (Pose) (2) - Camera_Center (Truth) (2)) ** 2) < 1.0E-6,
        "shifted-bound selection matches the known camera center");
   end AP3P_Select_Bounds;

   procedure AP3P_Select_Order (T : in out Fixture) is
      pragma Unreferenced (T);
      function Angle (Left, Right : Rotation_Matrix) return OpenCV.Float64_Value is
         Trace : OpenCV.Float64_Value := 0.0;
      begin
         for Axis in 0 .. 2 loop
            Trace := Trace + Left (Axis, 0) * Right (Axis, 0)
              + Left (Axis, 1) * Right (Axis, 1)
              + Left (Axis, 2) * Right (Axis, 2);
         end loop;
         return Math.Arccos (OpenCV.Float64_Value'Max
           (-1.0, OpenCV.Float64_Value'Min (1.0, (Trace - 1.0) / 2.0)));
      end Angle;
      Calibration : constant Camera_Intrinsics := (800.0, 820.0, 640.0, 360.0);
      Seeds : constant Object_Point_Array :=
        [[0.3, -0.2, 3.5], [1.4, 0.1, 4.0], [-0.5, 1.2, 5.3]];
      Reference : constant Object_Point_Array := [[0.7, -0.8, 6.1]];
      Truth : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.35, 0.22], Translation => [0.2, -0.3, 0.8]);
      Seed_Pixels : constant Image_Point_Array :=
        Project_Points (Seeds, Calibration, No_Distortion, Truth);
      Reference_Pixels : constant Image_Point_Array :=
        Project_Points (Reference, Calibration, No_Distortion, Truth);
      Found : constant AP3P_Pose_Hypothesis_Array :=
        Solve_AP3P_Hypotheses (Seeds, Seed_Pixels, Calibration);
      Policy : constant AP3P_Selection_Options := (1.0E-6, 1.0E-6, 1.0);
      Forward : constant AP3P_Reference_Result_Matrix :=
        Evaluate_AP3P_References (Found, Reference, Reference_Pixels, Calibration);
      Reversed : AP3P_Pose_Hypothesis_Array (Found'Range);
      Reversed_Matrix : AP3P_Reference_Result_Matrix (Forward'Range (1), Forward'Range (2));
   begin
      Assert (Found'Length > 1, "order test needs more than one native hypothesis");
      for I in Found'Range loop
         Reversed (Reversed'Last - (I - Found'First)) := Found (I);
         Reversed_Matrix (Forward'Last (1) - (I - Found'First), Forward'First (2)) :=
           Forward (I, Forward'First (2));
      end loop;
      declare
         First : constant AP3P_Selection_Result := Select_AP3P_Hypothesis (Forward, Policy);
         Second : constant AP3P_Selection_Result :=
           Select_AP3P_Hypothesis (Reversed_Matrix, Policy);
         Pose_A : constant World_To_Camera_Pose :=
           Selected_Pose (First, Found, Selected_Hypothesis_Index (First));
         Pose_B : constant World_To_Camera_Pose :=
           Selected_Pose (Second, Reversed, Selected_Hypothesis_Index (Second));
      begin
         Assert (Angle (Rotation_Matrix_Of (Pose_A), Rotation_Matrix_Of (Pose_B)) < 1.0E-12,
           "selected physical rotation is order invariant");
         Assert (Math.Sqrt ((Pose_A.Translation (0) - Pose_B.Translation (0)) ** 2
                   + (Pose_A.Translation (1) - Pose_B.Translation (1)) ** 2
                   + (Pose_A.Translation (2) - Pose_B.Translation (2)) ** 2) < 1.0E-9,
           "selected translation is order invariant");
      end;
   end AP3P_Select_Order;

   procedure AP3P_Select_Upper (T : in out Fixture) is
      pragma Unreferenced (T);
      Calibration : constant Camera_Intrinsics := (50.0, 60.0, 7.0, 9.0);
      Pose : constant World_To_Camera_Pose :=
        (Rotation => [others => 0.0], Translation => [0.0, 0.0, 3.0]);
      References : constant Object_Point_Array (Positive'Last .. Positive'Last) :=
        [[0.5, -0.25, 4.0]];
      Observed : constant Image_Point_Array (Positive'Last .. Positive'Last) :=
        Project_Points (References, Calibration, No_Distortion, Pose);
      Hypotheses : constant AP3P_Pose_Hypothesis_Array (Positive'Last .. Positive'Last) :=
        [(Pose => Pose)];
      Evaluated : constant AP3P_Reference_Result_Matrix :=
        Evaluate_AP3P_References (Hypotheses, References, Observed, Calibration);
      Chosen : constant AP3P_Selection_Result :=
        Select_AP3P_Hypothesis (Evaluated, (1.0E-6, 1.0E-6, 0.0));
   begin
      Assert (References'First = Positive'Last, "Positive'Last one-element bounds");
      Assert (Chosen.Disposition = Unique_Selected_Hypothesis
        and then Selected_Hypothesis_Index (Chosen) = 1,
        "Positive'Last bounds still select position one");
      Assert (Selected_Pose (Chosen, Hypotheses, Positive'Last) = Pose,
        "Positive'Last index maps to the only hypothesis");
   end AP3P_Select_Upper;

   procedure AP3P_Select_Provenance (T : in out Fixture) is
      pragma Unreferenced (T);
      Calibration : constant Camera_Intrinsics := (800.0, 820.0, 640.0, 360.0);
      Truth : constant World_To_Camera_Pose :=
        (Rotation => [0.10, -0.05, 0.08], Translation => [0.20, -0.10, 6.00]);
      Seeds : constant Object_Point_Array :=
        [[-1.0, -1.0, 0.0], [1.0, -0.4, 0.6], [0.2, 1.1, 1.4]];
      References : constant Object_Point_Array :=
        [[-0.6, 0.4, 2.2], [1.3, 0.8, 1.7], [0.4, -1.2, 2.8]];
      Seed_Pixels : constant Image_Point_Array :=
        Project_Points (Seeds, Calibration, No_Distortion, Truth);
      Reference_Pixels : constant Image_Point_Array :=
        Project_Points (References, Calibration, No_Distortion, Truth);
      Policy : constant AP3P_Selection_Options := (1.0E-6, 1.0E-6, 1.0E-3);
      Owned : constant AP3P_Pose_Selection :=
        Select_AP3P_Pose
          (Seeds, Seed_Pixels, References, Reference_Pixels, Calibration,
           No_Distortion, Policy);
      Pose : constant World_To_Camera_Pose := Selected_Pose (Owned);
      procedure Reject (Value : AP3P_Pose_Selection) is
      begin
         declare
            Exposed : constant World_To_Camera_Pose := Selected_Pose (Value);
         begin
            Assert (False, "non-unique selection exposed translation "
              & OpenCV.Float64_Value'Image (Exposed.Translation (2)));
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Reject;
   begin
      Assert (Selected (Owned), "end-to-end result owns the selected pose");
      declare
         Recovered : constant Rotation_Matrix := Rotation_Matrix_Of (Pose);
         Expected : constant Rotation_Matrix := Rotation_Matrix_Of (Truth);
         Trace : OpenCV.Float64_Value := 0.0;
      begin
         for Axis in 0 .. 2 loop
            Trace := Trace + Recovered (Axis, 0) * Expected (Axis, 0)
              + Recovered (Axis, 1) * Expected (Axis, 1)
              + Recovered (Axis, 2) * Expected (Axis, 2);
         end loop;
         Assert (Math.Arccos (OpenCV.Float64_Value'Max
                   (-1.0, OpenCV.Float64_Value'Min (1.0, (Trace - 1.0) / 2.0))) < 1.0E-6,
           "owned rotation matches the oracle without another solve");
         Assert (Math.Sqrt ((Camera_Center (Pose) (0) - Camera_Center (Truth) (0)) ** 2
                   + (Camera_Center (Pose) (1) - Camera_Center (Truth) (1)) ** 2
                   + (Camera_Center (Pose) (2) - Camera_Center (Truth) (2)) ** 2) < 1.0E-6,
           "owned camera center matches -R^T t");
      end;
      declare
         Found : constant AP3P_Pose_Hypothesis_Array :=
           Solve_AP3P_Hypotheses (Seeds, Seed_Pixels, Calibration);
         Evaluated : constant AP3P_Reference_Result_Matrix :=
           Evaluate_AP3P_References (Found, References, Reference_Pixels, Calibration);
         Indexed : constant AP3P_Selection_Result :=
           Select_AP3P_Hypothesis (Evaluated, Policy);
         Slot : constant Positive := Selected_Hypothesis_Index (Indexed);
         Substitute : AP3P_Pose_Hypothesis_Array := Found;
      begin
         Substitute (Substitute'First + Slot - 1) :=
           (Pose => (Rotation => [others => 0.0], Translation => [0.0, 0.0, 1.0]));
         Assert (Selected_Pose (Indexed, Substitute, Substitute'First + Slot - 1).Translation
             = Translation_Vector'(0.0, 0.0, 1.0)
           and then Pose.Translation /= Translation_Vector'(0.0, 0.0, 1.0),
           "an external array is not the owned pose");
      end;
      Reject (Select_AP3P_Pose
        (Seeds, Seed_Pixels, References, Reference_Pixels, Calibration,
         No_Distortion, (0.0, 0.0, 0.0)));
      declare
         Wide : constant AP3P_Pose_Selection :=
           Select_AP3P_Pose
             (Seeds, Seed_Pixels, References, Reference_Pixels, Calibration,
              No_Distortion,
              (OpenCV.Float64_Value'Last, OpenCV.Float64_Value'Last,
               OpenCV.Float64_Value'Last));
      begin
         if Candidates (Wide)'Length > 1 then
            Assert (Disposition (Wide) = Ambiguous_Qualified_Hypotheses,
              "huge separation leaves multiple hypotheses ambiguous");
            Reject (Wide);
         else
            Assert (Selected (Wide), "a sole hypothesis remains owned");
         end if;
      end;
   end AP3P_Select_Provenance;

   --  Task 017. Held-out acceptance is independent of fitting error.
   --  Geometric helpers below are test oracles only.

   function Refine_Rot_Y (Angle : OpenCV.Float64_Value) return Rotation_Matrix is
      C : constant OpenCV.Float64_Value := Math.Cos (Angle);
      S : constant OpenCV.Float64_Value := Math.Sin (Angle);
   begin
      return [[C, 0.0, S], [0.0, 1.0, 0.0], [-S, 0.0, C]];
   end Refine_Rot_Y;

   function Refine_Rot_Z (Angle : OpenCV.Float64_Value) return Rotation_Matrix is
      C : constant OpenCV.Float64_Value := Math.Cos (Angle);
      S : constant OpenCV.Float64_Value := Math.Sin (Angle);
   begin
      return [[C, -S, 0.0], [S, C, 0.0], [0.0, 0.0, 1.0]];
   end Refine_Rot_Z;

   function Refine_Multiply (Left, Right : Rotation_Matrix) return Rotation_Matrix is
      Result : Rotation_Matrix := [others => [others => 0.0]];
   begin
      for Row in 0 .. 2 loop
         for Column in 0 .. 2 loop
            for K in 0 .. 2 loop
               Result (Row, Column) :=
                 Result (Row, Column) + Left (Row, K) * Right (K, Column);
            end loop;
         end loop;
      end loop;
      return Result;
   end Refine_Multiply;

   function Refine_Apply (Matrix : Rotation_Matrix; Point : Object_Point) return Object_Point is
     ([Matrix (0, 0) * Point (0) + Matrix (0, 1) * Point (1) + Matrix (0, 2) * Point (2),
      Matrix (1, 0) * Point (0) + Matrix (1, 1) * Point (1) + Matrix (1, 2) * Point (2),
      Matrix (2, 0) * Point (0) + Matrix (2, 1) * Point (1) + Matrix (2, 2) * Point (2)]);

   function Refine_Transpose (Matrix : Rotation_Matrix) return Rotation_Matrix is
     ([[Matrix (0, 0), Matrix (1, 0), Matrix (2, 0)],
       [Matrix (0, 1), Matrix (1, 1), Matrix (2, 1)],
       [Matrix (0, 2), Matrix (1, 2), Matrix (2, 2)]]);

   Refine_Truth_R : constant Rotation_Matrix :=
     Refine_Multiply (Refine_Rot_Z (0.22), Refine_Rot_Y (0.35));
   Refine_Truth_T : constant Translation_Vector := [0.2, -0.3, 0.8];
   Refine_Calibration : constant Camera_Intrinsics := (800.0, 820.0, 640.0, 360.0);
   Refine_Seeds : constant Object_Point_Array :=
     [[0.3, -0.2, 3.5], [1.4, 0.1, 4.0], [-0.5, 1.2, 5.3]];
   Refine_Selection_World : constant Object_Point_Array :=
     [[0.7, -0.8, 6.1], [1.1, 0.4, 5.2]];
   Refine_Holdout_World : constant Object_Point_Array :=
     [[-0.9, -0.6, 4.4], [0.2, 1.3, 7.0], [-1.5, 0.5, 3.8]];
   Refine_Select_Policy : constant AP3P_Selection_Options := (1.0E-3, 1.0E-3, 1.0E-3);
   Refine_Accept_Policy : constant AP3P_Refinement_Acceptance_Options :=
     (1.0, 2.0, 0.25);

   function Refine_Project (Point : Object_Point) return Image_Point is
      Camera : constant Object_Point :=
        [Refine_Apply (Refine_Truth_R, Point) (0) + Refine_Truth_T (0),
         Refine_Apply (Refine_Truth_R, Point) (1) + Refine_Truth_T (1),
         Refine_Apply (Refine_Truth_R, Point) (2) + Refine_Truth_T (2)];
   begin
      return [Refine_Calibration.Focal_X * (Camera (0) / Camera (2)) + Refine_Calibration.Center_X,
              Refine_Calibration.Focal_Y * (Camera (1) / Camera (2)) + Refine_Calibration.Center_Y];
   end Refine_Project;
   function Refine_Brown
     (Point : Object_Point; Lens : Distortion_Coefficients) return Image_Point is
      Camera : constant Object_Point :=
        [Refine_Apply (Refine_Truth_R, Point) (0) + Refine_Truth_T (0),
         Refine_Apply (Refine_Truth_R, Point) (1) + Refine_Truth_T (1),
         Refine_Apply (Refine_Truth_R, Point) (2) + Refine_Truth_T (2)];
      X : constant OpenCV.Float64_Value := Camera (0) / Camera (2);
      Y : constant OpenCV.Float64_Value := Camera (1) / Camera (2);
      R2 : constant OpenCV.Float64_Value := X * X + Y * Y;
      Radial : constant OpenCV.Float64_Value :=
        1.0 + Lens.K1 * R2 + Lens.K2 * R2 * R2 + Lens.K3 * R2 * R2 * R2;
   begin
      return [Refine_Calibration.Focal_X * (X * Radial + 2.0 * Lens.P1 * X * Y
                + Lens.P2 * (R2 + 2.0 * X * X)) + Refine_Calibration.Center_X,
              Refine_Calibration.Focal_Y * (Y * Radial + Lens.P1 * (R2 + 2.0 * Y * Y)
                + 2.0 * Lens.P2 * X * Y) + Refine_Calibration.Center_Y];
   end Refine_Brown;

   function Pixels_Of
     (Points : Object_Point_Array;
      Lens : Distortion_Coefficients := No_Distortion) return Image_Point_Array is
      Result : Image_Point_Array (1 .. Points'Length);
   begin
      for I in Points'Range loop
         declare
            Slot : constant Positive := 1 + (I - Points'First);
         begin
            if Lens = No_Distortion then
               Result (Slot) := Refine_Project (Points (I));
            else
               Result (Slot) := Refine_Brown (Points (I), Lens);
            end if;
         end;
      end loop;
      return Result;
   end Pixels_Of;

   procedure Assert_Geometric_Pose
     (Pose : World_To_Camera_Pose; Message : String;
      Angle_Limit : OpenCV.Float64_Value := 1.0E-4;
      Translation_Limit : OpenCV.Float64_Value := 1.0E-4) is
      Recovered : constant Rotation_Matrix := Rotation_Matrix_Of (Pose);
      Trace : OpenCV.Float64_Value := 0.0;
      Expected_Center : constant Object_Point :=
        [-Refine_Apply (Refine_Transpose (Refine_Truth_R), Object_Point (Refine_Truth_T)) (0),
         -Refine_Apply (Refine_Transpose (Refine_Truth_R), Object_Point (Refine_Truth_T)) (1),
         -Refine_Apply (Refine_Transpose (Refine_Truth_R), Object_Point (Refine_Truth_T)) (2)];
      Center : constant Object_Point := Camera_Center (Pose);
   begin
      for Axis in 0 .. 2 loop
         Trace := Trace + Recovered (Axis, 0) * Refine_Truth_R (Axis, 0)
           + Recovered (Axis, 1) * Refine_Truth_R (Axis, 1)
           + Recovered (Axis, 2) * Refine_Truth_R (Axis, 2);
      end loop;
      Assert (Math.Arccos (OpenCV.Float64_Value'Max
                (-1.0, OpenCV.Float64_Value'Min (1.0, (Trace - 1.0) / 2.0))) < Angle_Limit,
        Message & " rotation");
      Assert (Math.Sqrt ((Pose.Translation (0) - Refine_Truth_T (0)) ** 2
                + (Pose.Translation (1) - Refine_Truth_T (1)) ** 2
                + (Pose.Translation (2) - Refine_Truth_T (2)) ** 2) < Translation_Limit,
        Message & " translation");
      Assert (Math.Sqrt ((Center (0) - Expected_Center (0)) ** 2
                + (Center (1) - Expected_Center (1)) ** 2
                + (Center (2) - Expected_Center (2)) ** 2) < Translation_Limit,
        Message & " camera center");
   end Assert_Geometric_Pose;

   procedure Reject_Accepted (Value : AP3P_Refined_Pose) is
   begin
      declare
         Exposed : constant World_To_Camera_Pose := Accepted_Pose (Value);
      begin
         Assert (False, "unaccepted pose exposed t.z="
           & OpenCV.Float64_Value'Image (Exposed.Translation (2)));
      end;
   exception
      when OpenCV.OpenCV_Error => null;
   end Reject_Accepted;

   procedure Reject_Refined_Summary (Value : AP3P_Refined_Pose) is
   begin
      declare
         Summary : constant Reprojection_Summary := Refined_Holdout (Value);
      begin
         Assert (False, "invalid refined holdout exposed rms="
           & OpenCV.Float64_Value'Image (Summary.RMS_Error_Pixels));
      end;
   exception
      when OpenCV.OpenCV_Error => null;
   end Reject_Refined_Summary;
   procedure AP3P_Refine_Known (T : in out Fixture) is
      pragma Unreferenced (T);
      Result : constant AP3P_Refined_Pose :=
        Select_Refine_AP3P_Pose
          (Refine_Seeds, Pixels_Of (Refine_Seeds),
           Refine_Selection_World, Pixels_Of (Refine_Selection_World),
           Refine_Holdout_World, Pixels_Of (Refine_Holdout_World),
           Refine_Calibration, No_Distortion,
           Refine_Select_Policy, Refine_Accept_Policy);
      Pose : constant World_To_Camera_Pose := Accepted_Pose (Result);
      Before : constant Reprojection_Summary := Initial_Holdout (Result);
      After : constant Reprojection_Summary := Refined_Holdout (Result);
   begin
      Assert (Selection_Disposition (Result) = Unique_Selected_Hypothesis
        and then Refinement_Disposition (Result) = Accepted
        and then Accepted (Result)
        and then Refinement_Attempted (Result)
        and then Native_Refinement_Succeeded (Result),
        "clean nonidentity pose is selected and accepted");
      Assert_Geometric_Pose (Initial_Pose (Result), "initial selected");
      Assert_Geometric_Pose (Pose, "accepted refined");
      Assert (Before.Count = Refine_Holdout_World'Length
        and then After.Count = Before.Count
        and then Before.RMS_Error_Pixels <= 1.0E-6
        and then After.RMS_Error_Pixels <= 1.0E-6,
        "held-out oracle remains inside the caller policy");
      declare
         Native : constant Image_Point_Array :=
           Project_Points (Refine_Holdout_World, Refine_Calibration, No_Distortion, Pose);
         Expected : constant Image_Point_Array := Pixels_Of (Refine_Holdout_World);
         Worst : OpenCV.Float64_Value := 0.0;
      begin
         for I in Native'Range loop
            Worst := OpenCV.Float64_Value'Max
              (Worst, Math.Sqrt ((Native (I) (0) - Expected (I) (0)) ** 2
                + (Native (I) (1) - Expected (I) (1)) ** 2));
         end loop;
         Assert (Worst < 1.0E-4, "accepted pose matches independent pinhole projection");
      end;
   end AP3P_Refine_Known;
   procedure AP3P_Refine_Noisy (T : in out Fixture) is
      pragma Unreferenced (T);
      Seeds : constant Object_Point_Array :=
        [[-1.0, -1.0, 0.0], [1.2, -0.4, 0.5], [0.1, 1.1, 1.2]];
      Selection_Points : constant Object_Point_Array :=
        [[-0.4, 0.6, 2.0], [1.1, 0.7, 1.6], [0.5, -1.1, 2.4], [-1.3, 0.2, 1.8]];
      Holdout_Points : constant Object_Point_Array :=
        [[0.8, -0.2, 2.7], [-0.7, -0.9, 3.1], [1.4, 1.0, 2.2]];
      Truth : constant World_To_Camera_Pose :=
        (Rotation => [0.12, -0.07, 0.05], Translation => [0.15, -0.25, 5.5]);
      --  Deterministic sub-pixel noise on the fitting observations only.
      Noise : constant array (1 .. 7) of Image_Point :=
        [[0.35, -0.20], [-0.15, 0.40], [0.25, 0.10],
         [-0.30, -0.25], [0.20, 0.15], [-0.10, 0.30], [0.40, -0.15]];
      function Truth_Pixels (Points : Object_Point_Array) return Image_Point_Array is
        (Project_Points (Points, Refine_Calibration, No_Distortion, Truth));
      Seed_Pixels : Image_Point_Array := Truth_Pixels (Seeds);
      Selection_Pixels : Image_Point_Array := Truth_Pixels (Selection_Points);
      Holdout_Pixels : constant Image_Point_Array := Truth_Pixels (Holdout_Points);
   begin
      for I in Seed_Pixels'Range loop
         Seed_Pixels (I) (0) := Seed_Pixels (I) (0) + Noise (I) (0);
         Seed_Pixels (I) (1) := Seed_Pixels (I) (1) + Noise (I) (1);
      end loop;
      for I in Selection_Pixels'Range loop
         Selection_Pixels (I) (0) :=
           Selection_Pixels (I) (0) + Noise (3 + (I - Selection_Pixels'First) + 1) (0);
         Selection_Pixels (I) (1) :=
           Selection_Pixels (I) (1) + Noise (3 + (I - Selection_Pixels'First) + 1) (1);
      end loop;
      declare
         Result : constant AP3P_Refined_Pose :=
           Select_Refine_AP3P_Pose
             (Seeds, Seed_Pixels, Selection_Points, Selection_Pixels,
              Holdout_Points, Holdout_Pixels, Refine_Calibration, No_Distortion,
              (2.0, 4.0, 0.05), (1.5, 3.0, 0.5));
         Before : constant Reprojection_Summary := Initial_Holdout (Result);
         After : constant Reprojection_Summary := Refined_Holdout (Result);
         Pose : constant World_To_Camera_Pose := Accepted_Pose (Result);
         Expected : constant Rotation_Matrix := Rotation_Matrix_Of (Truth);
         Recovered : constant Rotation_Matrix := Rotation_Matrix_Of (Pose);
         Trace : OpenCV.Float64_Value := 0.0;
      begin
         Assert (Refinement_Disposition (Result) = Accepted
           and then Native_Refinement_Succeeded (Result),
           "noisy fitting observations refine and pass held-out policy");
         Assert (After.RMS_Error_Pixels <= Before.RMS_Error_Pixels + 0.5
           and then After.RMS_Error_Pixels <= 1.5
           and then After.Maximum_Error_Pixels <= 3.0,
           "held-out summary obeys the caller thresholds");
         for Axis in 0 .. 2 loop
            Trace := Trace + Recovered (Axis, 0) * Expected (Axis, 0)
              + Recovered (Axis, 1) * Expected (Axis, 1)
              + Recovered (Axis, 2) * Expected (Axis, 2);
         end loop;
         Assert (Math.Arccos (OpenCV.Float64_Value'Max
                   (-1.0, OpenCV.Float64_Value'Min (1.0, (Trace - 1.0) / 2.0))) < 5.0E-3,
           "refined rotation stays near the independent truth");
         Assert (abs (Pose.Translation (2) - Truth.Translation (2)) < 5.0E-2,
           "refined translation stays near the independent truth");
      end;
   end AP3P_Refine_Noisy;
   procedure AP3P_Refine_Distortion (T : in out Fixture) is
      pragma Unreferenced (T);
      Lens : constant Distortion_Coefficients := (-0.12, 0.03, 0.001, -0.002, 0.01);
      Result : constant AP3P_Refined_Pose :=
        Select_Refine_AP3P_Pose
          (Refine_Seeds, Pixels_Of (Refine_Seeds, Lens),
           Refine_Selection_World, Pixels_Of (Refine_Selection_World, Lens),
           Refine_Holdout_World, Pixels_Of (Refine_Holdout_World, Lens),
           Refine_Calibration, Lens, (1.0E-2, 1.0E-2, 1.0E-3), (1.0E-2, 1.0E-2, 1.0E-3));
      Pose : constant World_To_Camera_Pose := Accepted_Pose (Result);
      Native : constant Image_Point_Array :=
        Project_Points (Refine_Holdout_World, Refine_Calibration, Lens, Pose);
      Expected : constant Image_Point_Array := Pixels_Of (Refine_Holdout_World, Lens);
      Worst : OpenCV.Float64_Value := 0.0;
   begin
      Assert (Refinement_Disposition (Result) = Accepted, "five-coefficient refinement accepted");
      Assert_Geometric_Pose (Pose, "distorted accepted", 2.0E-3, 2.0E-3);
      for I in Native'Range loop
         Worst := OpenCV.Float64_Value'Max
           (Worst, Math.Sqrt ((Native (I) (0) - Expected (I) (0)) ** 2
             + (Native (I) (1) - Expected (I) (1)) ** 2));
      end loop;
      Assert (Worst < 1.0E-2
        and then Refined_Holdout (Result).Maximum_Error_Pixels <= 1.0E-2,
        "held-out Brown projection stays inside the caller maximum");
      Assert (Lens.K1 /= 0.0 and then Lens.K2 /= 0.0 and then Lens.P1 /= 0.0
        and then Lens.P2 /= 0.0 and then Lens.K3 /= 0.0,
        "all five Brown coefficients are nonzero");
   end AP3P_Refine_Distortion;

   procedure AP3P_Refine_Worsens (T : in out Fixture) is
      pragma Unreferenced (T);
      --  Fitting pixels are shifted together. Held-out pixels stay on the
      --  geometric truth, so a successful fit can increase held-out RMS.
      --  Seeds stay on the geometric truth, so AP3P can recover it.
      --  Selection pixels are biased; refinement may follow that bias and
      --  increase error on the unbiased held-out set.
      Shift : constant Image_Point := [12.0, -9.0];
      Seed_Pixels : constant Image_Point_Array := Pixels_Of (Refine_Seeds);
      Selection_Pixels : Image_Point_Array := Pixels_Of (Refine_Selection_World);
      Holdout_Pixels : constant Image_Point_Array := Pixels_Of (Refine_Holdout_World);
   begin
      for Pixel of Selection_Pixels loop
         Pixel (0) := Pixel (0) + Shift (0);
         Pixel (1) := Pixel (1) + Shift (1);
      end loop;
      declare
         Value : constant AP3P_Refined_Pose :=
           Select_Refine_AP3P_Pose
             (Refine_Seeds, Seed_Pixels, Refine_Selection_World, Selection_Pixels,
              Refine_Holdout_World, Holdout_Pixels, Refine_Calibration,
              No_Distortion, (80.0, 120.0, 0.0), (1.0E6, 1.0E6, 0.05));
         Before : constant Reprojection_Summary := Initial_Holdout (Value);
         After : constant Reprojection_Summary := Refined_Holdout (Value);
      begin
         Ada.Text_IO.Put_Line
           ("AP3P held-out worsening initial="
            & OpenCV.Float64_Value'Image (Before.RMS_Error_Pixels)
            & " refined=" & OpenCV.Float64_Value'Image (After.RMS_Error_Pixels)
            & " " & AP3P_Refinement_Disposition'Image (Refinement_Disposition (Value)));
         Assert (Selection_Disposition (Value) = Unique_Selected_Hypothesis
           and then Native_Refinement_Succeeded (Value)
           and then Refinement_Disposition (Value) = Rejected_By_Policy
           and then not Accepted (Value)
           and then After.RMS_Error_Pixels > Before.RMS_Error_Pixels + 0.05,
           "held-out worsening beyond the allowance is rejected");
         Reject_Accepted (Value);
      end;
   end AP3P_Refine_Worsens;

   procedure AP3P_Refine_Threshold (T : in out Fixture) is
      pragma Unreferenced (T);
      Holdout_Pixels : Image_Point_Array := Pixels_Of (Refine_Holdout_World);
   begin
      Holdout_Pixels (Holdout_Pixels'First) (0) :=
        Holdout_Pixels (Holdout_Pixels'First) (0) + 1.0;
      declare
         Value : constant AP3P_Refined_Pose :=
           Select_Refine_AP3P_Pose
             (Refine_Seeds, Pixels_Of (Refine_Seeds),
              Refine_Selection_World, Pixels_Of (Refine_Selection_World),
              Refine_Holdout_World, Holdout_Pixels, Refine_Calibration,
              No_Distortion, Refine_Select_Policy, (10.0, 0.0, 10.0));
      begin
         Assert (Refinement_Disposition (Value) = Rejected_By_Policy
           and then Native_Refinement_Succeeded (Value)
           and then Refined_Holdout (Value).Maximum_Error_Pixels > 0.0
           and then not Accepted (Value),
           "a caller maximum of zero rejects a positive held-out error");
         Reject_Accepted (Value);
      end;
      declare
         Clean_Pixels : constant Image_Point_Array := Pixels_Of (Refine_Holdout_World);
         Clean : constant AP3P_Refined_Pose :=
           Select_Refine_AP3P_Pose
             (Refine_Seeds, Pixels_Of (Refine_Seeds),
              Refine_Selection_World, Pixels_Of (Refine_Selection_World),
              Refine_Holdout_World, Clean_Pixels, Refine_Calibration,
              No_Distortion, Refine_Select_Policy, (1.0E-6, 1.0E-6, 1.0));
      begin
         Assert (Refinement_Disposition (Clean) = Accepted
           and then Refined_Holdout (Clean).Maximum_Error_Pixels <= 1.0E-6,
           "the unperturbed twin still meets a tight positive threshold");
      end;
   end AP3P_Refine_Threshold;
   procedure AP3P_Refine_Depth (T : in out Fixture) is
      pragma Unreferenced (T);
      --  The selected camera looks along +Z. A held-out world point with
      --  large negative Z is behind that camera.
      Seeds : constant Object_Point_Array :=
        [[-0.4, -0.2, 3.0], [0.6, -0.3, 3.4], [0.1, 0.5, 4.2]];
      Selection_Points : constant Object_Point_Array := [[0.8, 0.2, 5.0]];
      Holdout_Points : constant Object_Point_Array := [[0.0, 0.0, -6.0]];
      Pose : constant World_To_Camera_Pose :=
        (Rotation => [others => 0.0], Translation => [others => 0.0]);
      Result : constant AP3P_Refined_Pose :=
        Select_Refine_AP3P_Pose
          (Seeds, Project_Points (Seeds, Refine_Calibration, No_Distortion, Pose),
           Selection_Points,
           Project_Points (Selection_Points, Refine_Calibration, No_Distortion, Pose),
           Holdout_Points, [[100.0, 120.0]], Refine_Calibration, No_Distortion,
           (1.0E-4, 1.0E-4, 0.0), (1.0, 1.0, 1.0));
   begin
      Assert (Selection_Disposition (Result) = Unique_Selected_Hypothesis
        and then Refinement_Disposition (Result) = Initial_Holdout_Unusable
        and then not Refinement_Attempted (Result)
        and then not Native_Refinement_Succeeded (Result)
        and then not Initial_Holdout_Valid (Result)
        and then not Accepted (Result),
        "nonpositive held-out depth blocks refinement");
      Reject_Accepted (Result);
      Reject_Refined_Summary (Result);
   end AP3P_Refine_Depth;

   procedure AP3P_Refine_Unavailable (T : in out Fixture) is
      pragma Unreferenced (T);
      Angle : constant OpenCV.Float64_Value := Ada.Numerics.Pi / 4.0;
      Pose : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, Angle], Translation => [others => 0.0]);
      Seeds : constant Object_Point_Array :=
        [[-0.2, 0.1, 3.0], [0.4, -0.3, 3.5], [0.1, 0.6, 4.0]];
      Selection_Points : constant Object_Point_Array := [[0.7, 0.2, 5.0]];
      --  The second held-out sum overflows Float64 after the 45-degree rotation.
      Holdout_Points : constant Object_Point_Array :=
        [[0.0, 0.0, 4.0], [1.5E308, -1.5E308, 4.0]];
      Camera : constant Camera_Intrinsics := (100.0, 100.0, 0.0, 0.0);
      Result : constant AP3P_Refined_Pose :=
        Select_Refine_AP3P_Pose
          (Seeds, Project_Points (Seeds, Camera, No_Distortion, Pose),
           Selection_Points, Project_Points (Selection_Points, Camera, No_Distortion, Pose),
           Holdout_Points, [[10.0, 20.0], [30.0, 40.0]], Camera, No_Distortion,
           (1.0E-3, 1.0E-3, 0.0), (1.0, 1.0, 1.0));
   begin
      Assert (Refinement_Disposition (Result) = Initial_Holdout_Unusable
        and then not Refinement_Attempted (Result)
        and then not Accepted (Result),
        "numerically unavailable held-out input blocks refinement");
      Reject_Refined_Summary (Result);
   end AP3P_Refine_Unavailable;

   procedure AP3P_Refine_No_Selection (T : in out Fixture) is
      pragma Unreferenced (T);
      Empty : constant AP3P_Refined_Pose :=
        Select_Refine_AP3P_Pose
          (Refine_Seeds, Pixels_Of (Refine_Seeds),
           Refine_Selection_World (1 .. 0), Image_Point_Array'(1 .. 0 => [others => 0.0]),
           Refine_Holdout_World, Pixels_Of (Refine_Holdout_World),
           Refine_Calibration, No_Distortion,
           Refine_Select_Policy, Refine_Accept_Policy);
      Wide : constant AP3P_Refined_Pose :=
        Select_Refine_AP3P_Pose
          (Refine_Seeds, Pixels_Of (Refine_Seeds),
           Refine_Selection_World, Pixels_Of (Refine_Selection_World),
           Refine_Holdout_World, Pixels_Of (Refine_Holdout_World),
           Refine_Calibration, No_Distortion,
           (OpenCV.Float64_Value'Last, OpenCV.Float64_Value'Last, OpenCV.Float64_Value'Last),
           Refine_Accept_Policy);
   begin
      Assert (Selection_Disposition (Empty) = No_Independent_References
        and then Refinement_Disposition (Empty) = Selection_Only
        and then not Refinement_Attempted (Empty)
        and then not Accepted (Empty),
        "missing selection references do not invent a pose");
      begin
         declare
            Pose : constant World_To_Camera_Pose := Initial_Pose (Empty);
         begin
            Assert (False, "missing selection exposed t.z="
              & OpenCV.Float64_Value'Image (Pose.Translation (2)));
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
      if Selection_Disposition (Wide) = Ambiguous_Qualified_Hypotheses then
         Assert (Refinement_Disposition (Wide) = Selection_Only
           and then not Refinement_Attempted (Wide),
           "an ambiguous selection is not refined");
         Reject_Accepted (Wide);
      else
         Assert (Selection_Disposition (Wide) = Unique_Selected_Hypothesis,
           "only a sole surviving hypothesis may continue");
      end if;
   end AP3P_Refine_No_Selection;
   procedure AP3P_Refine_Duplicates (T : in out Fixture) is
      pragma Unreferenced (T);
      procedure Reject
        (Seeds, Selection_Points, Holdout_Points : Object_Point_Array) is
      begin
         declare
            Value : constant AP3P_Refined_Pose :=
              Select_Refine_AP3P_Pose
                (Seeds, Pixels_Of (Seeds),
                 Selection_Points, Pixels_Of (Selection_Points),
                 Holdout_Points, Pixels_Of (Holdout_Points),
                 Refine_Calibration, No_Distortion,
                 Refine_Select_Policy, Refine_Accept_Policy);
         begin
            Assert (False, "duplicate world points accepted disposition "
              & AP3P_Refinement_Disposition'Image (Refinement_Disposition (Value)));
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Reject;
      Repeated_Seed : Object_Point_Array := Refine_Seeds;
      Repeated_Selection : Object_Point_Array := Refine_Selection_World;
   begin
      Repeated_Seed (Repeated_Seed'Last) := Repeated_Seed (Repeated_Seed'First);
      Reject (Repeated_Seed, Refine_Selection_World, Refine_Holdout_World);
      Repeated_Selection (Repeated_Selection'Last) :=
        Repeated_Selection (Repeated_Selection'First);
      Reject (Refine_Seeds, Repeated_Selection, Refine_Holdout_World);
      Reject (Refine_Seeds, Refine_Selection_World,
              [Refine_Holdout_World (1), Refine_Holdout_World (1), Refine_Holdout_World (3)]);
      Reject (Refine_Seeds, [Refine_Seeds (1)], Refine_Holdout_World);
      Reject (Refine_Seeds, Refine_Selection_World, [Refine_Seeds (3)]);
      Reject (Refine_Seeds, Refine_Selection_World, [Refine_Selection_World (1)]);
   end AP3P_Refine_Duplicates;
   procedure AP3P_Refine_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check);
      procedure Reject_Acceptance (Value : AP3P_Refinement_Acceptance_Options) is
      begin
         declare
            Result : constant AP3P_Refined_Pose :=
              Select_Refine_AP3P_Pose
                (Refine_Seeds, Pixels_Of (Refine_Seeds),
                 Refine_Selection_World, Pixels_Of (Refine_Selection_World),
                 Refine_Holdout_World, Pixels_Of (Refine_Holdout_World),
                 Refine_Calibration, No_Distortion, Refine_Select_Policy, Value);
         begin
            Assert (False, "invalid acceptance accepted "
              & AP3P_Refinement_Disposition'Image (Refinement_Disposition (Result)));
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Reject_Acceptance;
      procedure Reject_Counts
        (Seeds : Object_Point_Array; Seed_Pixels : Image_Point_Array;
         Selection_Points : Object_Point_Array; Selection_Pixels : Image_Point_Array;
         Holdout_Points : Object_Point_Array; Holdout_Pixels : Image_Point_Array) is
      begin
         declare
            Result : constant AP3P_Refined_Pose :=
              Select_Refine_AP3P_Pose
                (Seeds, Seed_Pixels, Selection_Points, Selection_Pixels,
                 Holdout_Points, Holdout_Pixels, Refine_Calibration,
                 No_Distortion, Refine_Select_Policy, Refine_Accept_Policy);
         begin
            Assert (False, "invalid counts accepted "
              & AP3P_Selection_Disposition'Image (Selection_Disposition (Result)));
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Reject_Counts;
   begin
      Reject_Acceptance ((-1.0, 1.0, 1.0));
      Reject_Acceptance ((1.0, -1.0, 1.0));
      Reject_Acceptance ((1.0, 1.0, -1.0));
      Reject_Acceptance ((OpenCV.Float64_Value (Nonfinite (0)), 1.0, 1.0));
      Reject_Acceptance ((1.0, OpenCV.Float64_Value (Nonfinite (1)), 1.0));
      Reject_Acceptance ((1.0, 1.0, OpenCV.Float64_Value (Nonfinite (0))));
      Reject_Counts (Refine_Seeds (1 .. 2), Pixels_Of (Refine_Seeds) (1 .. 2),
                     Refine_Selection_World, Pixels_Of (Refine_Selection_World),
                     Refine_Holdout_World, Pixels_Of (Refine_Holdout_World));
      Reject_Counts (Refine_Seeds, Pixels_Of (Refine_Seeds) (1 .. 2),
                     Refine_Selection_World, Pixels_Of (Refine_Selection_World),
                     Refine_Holdout_World, Pixels_Of (Refine_Holdout_World));
      Reject_Counts (Refine_Seeds, Pixels_Of (Refine_Seeds),
                     Refine_Selection_World, Pixels_Of (Refine_Selection_World) (1 .. 1),
                     Refine_Holdout_World, Pixels_Of (Refine_Holdout_World));
      Reject_Counts (Refine_Seeds, Pixels_Of (Refine_Seeds),
                     Refine_Selection_World, Pixels_Of (Refine_Selection_World),
                     Refine_Holdout_World, Pixels_Of (Refine_Holdout_World) (1 .. 1));
      declare
         Bad_World : Object_Point_Array := Refine_Holdout_World;
         Bad_Pixel : Image_Point_Array := Pixels_Of (Refine_Seeds);
      begin
         Bad_World (Bad_World'First) (2) := OpenCV.Float64_Value (Nonfinite (0));
         Bad_Pixel (Bad_Pixel'First) (0) := OpenCV.Float64_Value (Nonfinite (1));
         Reject_Counts (Refine_Seeds, Pixels_Of (Refine_Seeds),
                        Refine_Selection_World, Pixels_Of (Refine_Selection_World),
                        Bad_World, Pixels_Of (Refine_Holdout_World));
         Reject_Counts (Refine_Seeds, Bad_Pixel,
                        Refine_Selection_World, Pixels_Of (Refine_Selection_World),
                        Refine_Holdout_World, Pixels_Of (Refine_Holdout_World));
      end;
      declare
         Extreme : constant AP3P_Refined_Pose :=
           Select_Refine_AP3P_Pose
             (Refine_Seeds, Pixels_Of (Refine_Seeds),
              Refine_Selection_World, Pixels_Of (Refine_Selection_World),
              Refine_Holdout_World, Pixels_Of (Refine_Holdout_World),
              Refine_Calibration, No_Distortion, Refine_Select_Policy,
              (OpenCV.Float64_Value'Last, OpenCV.Float64_Value'Last, OpenCV.Float64_Value'Last));
      begin
         Assert (Accepted (Extreme), "extreme finite nonnegative thresholds remain valid");
      end;
   end AP3P_Refine_Invalid;
   procedure AP3P_Refine_Bounds (T : in out Fixture) is
      pragma Unreferenced (T);
      subtype High is Positive range 7 .. 9;
      subtype Low is Positive range 4 .. 5;
      subtype Mid is Positive range 20 .. 22;
      Seeds : constant Object_Point_Array (High) :=
        [7 => Refine_Seeds (1), 8 => Refine_Seeds (2), 9 => Refine_Seeds (3)];
      Selection_Points : constant Object_Point_Array (Low) :=
        [4 => Refine_Selection_World (1), 5 => Refine_Selection_World (2)];
      Holdout_Points : constant Object_Point_Array (Mid) :=
        [20 => Refine_Holdout_World (1), 21 => Refine_Holdout_World (2),
         22 => Refine_Holdout_World (3)];
      Result : constant AP3P_Refined_Pose :=
        Select_Refine_AP3P_Pose
          (Seeds, Pixels_Of (Seeds), Selection_Points, Pixels_Of (Selection_Points),
           Holdout_Points, Pixels_Of (Holdout_Points), Refine_Calibration,
           No_Distortion, Refine_Select_Policy, Refine_Accept_Policy);
   begin
      Assert (Seeds'First = 7 and then Selection_Points'Last = 5
        and then Holdout_Points'First = 20, "different lower bounds");
      Assert (Accepted (Result), "positional pairing ignores lower bounds");
      Assert_Geometric_Pose (Accepted_Pose (Result), "bounded accepted");
   end AP3P_Refine_Bounds;

   procedure AP3P_Refine_Upper (T : in out Fixture) is
      pragma Unreferenced (T);
      Selection_Points : constant Object_Point_Array (Positive'Last .. Positive'Last) :=
        [Positive'Last => [0.7, -0.8, 6.1]];
      Holdout_Points : constant Object_Point_Array (Positive'Last .. Positive'Last) :=
        [Positive'Last => [-0.9, -0.6, 4.4]];
      Result : constant AP3P_Refined_Pose :=
        Select_Refine_AP3P_Pose
          (Refine_Seeds, Pixels_Of (Refine_Seeds),
           Selection_Points, Pixels_Of (Selection_Points),
           Holdout_Points, Pixels_Of (Holdout_Points),
           Refine_Calibration, No_Distortion,
           (1.0E-3, 1.0E-3, 0.0), Refine_Accept_Policy);
   begin
      Assert (Selection_Points'First = Positive'Last
        and then Holdout_Points'Last = Positive'Last,
        "Positive'Last one-element bounds");
      Assert (Accepted (Result) and then Initial_Holdout (Result).Count = 1,
        "upper-bound indexing still evaluates the held-out point");
   end AP3P_Refine_Upper;

   procedure AP3P_Refine_Worsening_Arithmetic (T : in out Fixture) is
      pragma Unreferenced (T);
      function Within
        (Original_RMS, Refined_RMS, Allowance : OpenCV.Float64_Value) return Boolean
        renames OpenCV.Calib3D.Internal.Refinement_Policy.Holdout_RMS_Worsening_Within_Allowance;
      Exact : constant OpenCV.Float64_Value := 7.0;
      Just_Above : constant OpenCV.Float64_Value := 7.000000000000002;
      Allowance : constant OpenCV.Float64_Value := 1.6E-15;
      Large_Original : constant OpenCV.Float64_Value := 1.0E307;
      Large_Refined : constant OpenCV.Float64_Value := 1.5E307;
      Near_Limit : constant OpenCV.Float64_Value := OpenCV.Float64_Value'Last;
      Near_Predecessor : constant OpenCV.Float64_Value :=
        OpenCV.Float64_Value'Adjacent (Near_Limit, 0.0);
   begin
      Assert (Within (4.0, 4.0, 0.0), "exact zero worsening is within a zero allowance");
      Assert (Within (5.0, 4.5, 0.0), "genuine improvement is within a zero allowance");
      Assert (Within (2.0, 2.25, 0.25), "worsening exactly equal to the allowance is accepted");
      Assert (not Within (2.0, 2.25, 0.24999999999999997),
        "worsening just above the allowance is rejected");
      Assert (Within (2.0, 2.25, 0.25000000000000006),
        "worsening just below the allowance is accepted");
      Assert (not Within (Exact, Just_Above, Allowance),
        "7.0 to 7.000000000000002 exceeds a 1.6E-15 allowance");
      Assert (Within (Exact, Exact + OpenCV.Float64_Value'Succ (Exact) - Exact, Allowance),
        "one ULP above 7.0 remains within 1.6E-15");
      Assert (Within (Large_Original, Large_Refined, 0.5E307),
        "large finite worsening exactly equal to the allowance is accepted");
      Assert (not Within
                (Large_Original, Large_Refined,
                 OpenCV.Float64_Value'Pred (Large_Refined - Large_Original)),
        "large finite worsening above the adjacent smaller allowance is rejected");
      Assert (not Within (Near_Predecessor, Near_Limit, 0.0),
        "a positive worsening near Float64'Last exceeds a zero allowance");
      Assert (Within (Near_Limit, Near_Predecessor, 0.0),
        "a large finite improvement remains within a zero allowance");
      Assert (Within (0.0, Near_Limit, Near_Limit),
        "worsening from zero to Float64'Last equals that allowance");
      Assert (not Within (0.0, Near_Limit, Near_Predecessor),
        "worsening from zero to Float64'Last exceeds the predecessor allowance");
   end AP3P_Refine_Worsening_Arithmetic;

   procedure AP3P_Refine_Order (T : in out Fixture) is
      pragma Unreferenced (T);
      Forward : constant AP3P_Refined_Pose :=
        Select_Refine_AP3P_Pose
          (Refine_Seeds, Pixels_Of (Refine_Seeds),
           Refine_Selection_World, Pixels_Of (Refine_Selection_World),
           Refine_Holdout_World, Pixels_Of (Refine_Holdout_World),
           Refine_Calibration, No_Distortion,
           Refine_Select_Policy, Refine_Accept_Policy);
      Reversed_Selection : constant Object_Point_Array :=
        [Refine_Selection_World (2), Refine_Selection_World (1)];
      Reversed_Holdout : constant Object_Point_Array :=
        [Refine_Holdout_World (3), Refine_Holdout_World (1), Refine_Holdout_World (2)];
      Backward : constant AP3P_Refined_Pose :=
        Select_Refine_AP3P_Pose
          (Refine_Seeds, Pixels_Of (Refine_Seeds),
           Reversed_Selection, Pixels_Of (Reversed_Selection),
           Reversed_Holdout, Pixels_Of (Reversed_Holdout),
           Refine_Calibration, No_Distortion,
           Refine_Select_Policy, Refine_Accept_Policy);
      First : constant Rotation_Matrix := Rotation_Matrix_Of (Accepted_Pose (Forward));
      Second : constant Rotation_Matrix := Rotation_Matrix_Of (Accepted_Pose (Backward));
      Trace : OpenCV.Float64_Value := 0.0;
   begin
      for Axis in 0 .. 2 loop
         Trace := Trace + First (Axis, 0) * Second (Axis, 0)
           + First (Axis, 1) * Second (Axis, 1)
           + First (Axis, 2) * Second (Axis, 2);
      end loop;
      Assert (Math.Arccos (OpenCV.Float64_Value'Max
                (-1.0, OpenCV.Float64_Value'Min (1.0, (Trace - 1.0) / 2.0))) < 1.0E-5,
        "reversing references does not change the accepted rotation");
      Assert (Math.Sqrt
                ((Accepted_Pose (Forward).Translation (0) - Accepted_Pose (Backward).Translation (0)) ** 2
                 + (Accepted_Pose (Forward).Translation (1) - Accepted_Pose (Backward).Translation (1)) ** 2
                 + (Accepted_Pose (Forward).Translation (2) - Accepted_Pose (Backward).Translation (2)) ** 2)
                < 1.0E-5,
        "reversing references does not change the accepted translation");
      Assert (Initial_Pose (Forward).Translation /= Translation_Vector'(0.0, 0.0, 1.0),
        "the owned initial pose is not a substituted placeholder");
   end AP3P_Refine_Order;

   --  Task 014 independent oracles. Composition equations appear in TEST code only.
   Pi_Value : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Ada.Numerics.Pi);

   function Rot_X (A : OpenCV.Float64_Value) return Rotation_Matrix is
      C : constant OpenCV.Float64_Value := Math.Cos (A);
      S : constant OpenCV.Float64_Value := Math.Sin (A);
   begin
      return [[1.0, 0.0, 0.0], [0.0, C, -S], [0.0, S, C]];
   end Rot_X;

   function Rot_Z (A : OpenCV.Float64_Value) return Rotation_Matrix is
      C : constant OpenCV.Float64_Value := Math.Cos (A);
      S : constant OpenCV.Float64_Value := Math.Sin (A);
   begin
      return [[C, -S, 0.0], [S, C, 0.0], [0.0, 0.0, 1.0]];
   end Rot_Z;

   function Mul (A, B : Rotation_Matrix) return Rotation_Matrix is
   begin
      return Result : Rotation_Matrix := [others => [others => 0.0]] do
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               for K in 0 .. 2 loop
                  Result (I, J) := Result (I, J) + A (I, K) * B (K, J);
               end loop;
            end loop;
         end loop;
      end return;
   end Mul;

   function Apply (M : Rotation_Matrix; V : Object_Point) return Object_Point is
   begin
      return Result : Object_Point := [others => 0.0] do
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               Result (I) := Result (I) + M (I, J) * V (J);
            end loop;
         end loop;
      end return;
   end Apply;

   function Matrix_Difference (A, B : Rotation_Matrix) return OpenCV.Float64_Value is
      Worst : OpenCV.Float64_Value := 0.0;
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            Worst := OpenCV.Float64_Value'Max (Worst, abs (A (I, J) - B (I, J)));
         end loop;
      end loop;
      return Worst;
   end Matrix_Difference;

   function Orthonormal_Error (M : Rotation_Matrix) return OpenCV.Float64_Value is
      Worst : OpenCV.Float64_Value := 0.0;
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            declare
               Dot : OpenCV.Float64_Value := 0.0;
            begin
               for K in 0 .. 2 loop
                  Dot := Dot + M (K, I) * M (K, J);
               end loop;
               Worst := OpenCV.Float64_Value'Max
                 (Worst, abs (Dot - (if I = J then 1.0 else 0.0)));
            end;
         end loop;
      end loop;
      return Worst;
   end Orthonormal_Error;

   Compose_First : constant World_To_Camera_Pose :=
     (Rotation => [0.0, 0.0, Pi_Value / 2.0], Translation => [2.0, -1.0, 0.5]);
   Compose_Second : constant World_To_Camera_Pose :=
     (Rotation => [Pi_Value / 3.0, 0.0, 0.0], Translation => [1.0, 3.0, -2.0]);
   Compose_Points : constant Object_Point_Array :=
     [[0.7, -0.4, 1.3], [-1.1, 0.9, 0.2], [2.5, 1.5, -0.8], [0.0, 0.3, 0.6]];

   function Expected_Rotation return Rotation_Matrix is
     (Mul (Rot_X (Pi_Value / 3.0), Rot_Z (Pi_Value / 2.0)));

   function Expected_Translation return Object_Point is
      Rotated : constant Object_Point :=
        Apply (Rot_X (Pi_Value / 3.0), [2.0, -1.0, 0.5]);
   begin
      return [Rotated (0) + 1.0, Rotated (1) + 3.0, Rotated (2) - 2.0];
   end Expected_Translation;

   procedure Compose_Identity (T : in out Fixture) is
      pragma Unreferenced (T);
      Identity : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, 0.0], Translation => [0.0, 0.0, 0.0]);
      P : constant World_To_Camera_Pose := Compose_First;
      Left : constant World_To_Camera_Pose := Compose_Poses (Identity, P);
      Right : constant World_To_Camera_Pose := Compose_Poses (P, Identity);
      Both : constant World_To_Camera_Pose := Compose_Poses (Identity, Identity);
      type Pose_Pair is array (1 .. 2) of World_To_Camera_Pose;
      Pair : constant Pose_Pair := [Left, Right];
   begin
      for Result of Pair loop
         Assert (Matrix_Difference (Rotation_Matrix_Of (Result), Rotation_Matrix_Of (P)) < 1.0E-13,
                 "identity composition changed rotation");
         for Axis in 0 .. 2 loop
            Assert (Near (Result.Translation (Axis), P.Translation (Axis), 1.0E-13),
                    "identity composition changed translation");
            Assert (Near (Camera_Center (Result) (Axis), Camera_Center (P) (Axis), 1.0E-12),
                    "identity composition moved camera center");
         end loop;
         for Point of Compose_Points loop
            declare
               A : constant Object_Point := World_To_Camera_Point (Result, Point);
               B : constant Object_Point := World_To_Camera_Point (P, Point);
            begin
               Assert (Near (A (0), B (0), 1.0E-12) and then Near (A (1), B (1), 1.0E-12)
                       and then Near (A (2), B (2), 1.0E-12), "identity composition moved a point");
            end;
         end loop;
      end loop;
      Assert (Both = Identity, "identity composed with identity is not exactly zero");
   end Compose_Identity;

   procedure Compose_Order (T : in out Fixture) is
      pragma Unreferenced (T);
      Composed : constant World_To_Camera_Pose := Compose_Poses (Compose_First, Compose_Second);
      Reversed : constant World_To_Camera_Pose := Compose_Poses (Compose_Second, Compose_First);
      Expected_T : constant Object_Point := Expected_Translation;
   begin
      Assert (Matrix_Difference (Rotation_Matrix_Of (Composed), Expected_Rotation) < 1.0E-12,
              "R3 is not R_second * R_first");
      for Axis in 0 .. 2 loop
         Assert (Near (Composed.Translation (Axis), Expected_T (Axis), 1.0E-12),
                 "t3 is not R_second * t_first + t_second");
      end loop;
      Assert (Matrix_Difference (Rotation_Matrix_Of (Reversed), Expected_Rotation) > 0.1,
              "reversed rotation order was not distinguishable");
      Assert (abs (Reversed.Translation (0) - Composed.Translation (0)) > 0.1
              or else abs (Reversed.Translation (1) - Composed.Translation (1)) > 0.1,
              "reversed translation order was not distinguishable");
   end Compose_Order;

   procedure Compose_Sequential (T : in out Fixture) is
      pragma Unreferenced (T);
      Composed : constant World_To_Camera_Pose := Compose_Poses (Compose_First, Compose_Second);
      Direction : constant World_Direction := [0.3, -0.5, 0.8];
      Expected_D : constant Object_Point := Apply (Expected_Rotation, Object_Point (Direction));
      Shifted : constant World_To_Camera_Pose := Compose_Poses
        (Compose_First, (Rotation => Compose_Second.Rotation, Translation => [50.0, -60.0, 70.0]));
      Stepwise : constant Camera_Direction := World_To_Camera_Direction
        (Compose_Second, World_Direction (World_To_Camera_Direction (Compose_First, Direction)));
      Combined : constant Camera_Direction := World_To_Camera_Direction (Composed, Direction);
      Moved : constant Camera_Direction := World_To_Camera_Direction (Shifted, Direction);
   begin
      for Point of Compose_Points loop
         declare
            Sequential : constant Object_Point := World_To_Camera_Point
              (Compose_Second, World_To_Camera_Point (Compose_First, Point));
            Together : constant Object_Point := World_To_Camera_Point (Composed, Point);
         begin
            for Axis in 0 .. 2 loop
               Assert (Near (Together (Axis), Sequential (Axis), 1.0E-12),
                       "composed point differs from sequential transform");
            end loop;
         end;
      end loop;
      for Axis in 0 .. 2 loop
         Assert (Near (Combined (Axis), Stepwise (Axis), 1.0E-12)
                 and then Near (Combined (Axis), Expected_D (Axis), 1.0E-12)
                 and then Near (Moved (Axis), Combined (Axis), 1.0E-12),
                 "direction composition or translation independence differs");
      end loop;
   end Compose_Sequential;

   procedure Compose_Camera_Center (T : in out Fixture) is
      pragma Unreferenced (T);
      Composed : constant World_To_Camera_Pose := Compose_Poses (Compose_First, Compose_Second);
      R : constant Rotation_Matrix := Expected_Rotation;
      Tr : constant Object_Point := Expected_Translation;
      Center : constant Object_Point := Camera_Center (Composed);
   begin
      for Axis in 0 .. 2 loop
         declare
            Expected : OpenCV.Float64_Value := 0.0;
         begin
            for K in 0 .. 2 loop
               Expected := Expected - R (K, Axis) * Tr (K);
            end loop;
            Assert (Near (Center (Axis), Expected, 1.0E-12), "composed center is not -R^T*t");
            Assert (abs (Center (Axis) - Composed.Translation (Axis)) > 1.0E-3
                    or else Axis = 0, "center must not equal the translation term");
         end;
      end loop;
      Assert (abs (Center (1) - Tr (1)) > 0.1 and then abs (Center (2) - Tr (2)) > 0.1,
              "camera center collapsed to translation");
   end Compose_Camera_Center;


   --  Independent scalar pinhole/Brown oracle for camera-frame points (test only).
   function Scalar_Project
     (Camera : Object_Point; Intrinsics : Camera_Intrinsics; D : Distortion_Coefficients)
      return Image_Point
   is
      X : constant OpenCV.Float64_Value := Camera (0) / Camera (2);
      Y : constant OpenCV.Float64_Value := Camera (1) / Camera (2);
      R2 : constant OpenCV.Float64_Value := X * X + Y * Y;
      Radial : constant OpenCV.Float64_Value :=
        1.0 + D.K1 * R2 + D.K2 * R2 * R2 + D.K3 * R2 * R2 * R2;
      XD : constant OpenCV.Float64_Value :=
        X * Radial + 2.0 * D.P1 * X * Y + D.P2 * (R2 + 2.0 * X * X);
      YD : constant OpenCV.Float64_Value :=
        Y * Radial + D.P1 * (R2 + 2.0 * Y * Y) + 2.0 * D.P2 * X * Y;
   begin
      return [Intrinsics.Focal_X * XD + Intrinsics.Center_X,
              Intrinsics.Focal_Y * YD + Intrinsics.Center_Y];
   end Scalar_Project;

   procedure Compose_Projection (T : in out Fixture) is
      pragma Unreferenced (T);
      Camera_A : constant World_To_Camera_Pose :=
        (Rotation => [0.05, -0.04, 0.10], Translation => [0.10, -0.05, 0.30]);
      Camera_B : constant World_To_Camera_Pose :=
        (Rotation => [-0.03, 0.06, 0.02], Translation => [-0.20, 0.10, 5.00]);
      World_Points : constant Object_Point_Array :=
        [[0.4, -0.3, 1.0], [-0.8, 0.5, 0.2], [1.1, 0.9, 0.7], [-0.2, -0.9, 1.4]];
      Intrinsics : constant Camera_Intrinsics := (800.0, 820.0, 320.0, 240.0);
      Brown : constant Distortion_Coefficients :=
        (K1 => -0.12, K2 => 0.03, P1 => 0.001, P2 => -0.0015, K3 => 0.004);
      Composed : constant World_To_Camera_Pose := Compose_Poses (Camera_A, Camera_B);
      Worst : OpenCV.Float64_Value := 0.0;
   begin
      for Model in 1 .. 2 loop
         declare
            D : constant Distortion_Coefficients := (if Model = 1 then No_Distortion else Brown);
            Native : constant Image_Point_Array :=
              Project_Points (World_Points, Intrinsics, D, Composed);
         begin
            for I in World_Points'Range loop
               declare
                  Camera : constant Object_Point := World_To_Camera_Point
                    (Camera_B, World_To_Camera_Point (Camera_A, World_Points (I)));
                  Expected : constant Image_Point := Scalar_Project (Camera, Intrinsics, D);
               begin
                  Worst := OpenCV.Float64_Value'Max
                    (Worst, OpenCV.Float64_Value'Max
                       (abs (Native (I) (0) - Expected (0)), abs (Native (I) (1) - Expected (1))));
               end;
            end loop;
         end;
      end loop;
      Ada.Text_IO.Put_Line ("Compose projection maximum pixel difference:" &
                            OpenCV.Float64_Value'Image (Worst));
      Assert (Worst < 1.0E-8, "composed projection differs from sequential projection");
   end Compose_Projection;
   procedure Compose_Rodrigues (T : in out Fixture) is
      pragma Unreferenced (T);
      Zero : constant World_To_Camera_Pose := (others => <>);
      About_X : constant World_To_Camera_Pose :=
        (Rotation => [0.4, 0.0, 0.0], Translation => [1.0, 0.0, 0.0]);
      About_Y : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.7, 0.0], Translation => [0.0, 2.0, 0.0]);
      Near_Pi : constant World_To_Camera_Pose :=
        (Rotation => [Pi_Value - 1.0E-6, 0.0, 0.0], Translation => [1.0, 2.0, 3.0]);
      Past_Pi : constant World_To_Camera_Pose :=
        (Rotation => [-(Pi_Value - 1.0E-6), 0.0, 0.0], Translation => [-1.0, 0.5, 2.0]);
      type Pair_Array is array (1 .. 7) of World_To_Camera_Pose;
      Firsts : constant Pair_Array :=
        [Zero, Zero, About_X, About_X, About_X, Near_Pi, Near_Pi];
      Seconds : constant Pair_Array :=
        [Zero, About_X, Zero, About_X, About_Y, About_Y, Past_Pi];
   begin
      for I in Pair_Array'Range loop
         declare
            Value : constant World_To_Camera_Pose := Compose_Poses (Firsts (I), Seconds (I));
            M : constant Rotation_Matrix := Rotation_Matrix_Of (Value);
            Expected : constant Rotation_Matrix := Mul
              (Rotation_Matrix_Of (Seconds (I)), Rotation_Matrix_Of (Firsts (I)));
         begin
            Assert (Orthonormal_Error (M) < 1.0E-12, "composed rotation is not orthonormal");
            Assert (Matrix_Difference (M, Expected) < 1.0E-5,
                    "composed physical rotation differs, case" & Integer'Image (I));
         end;
      end loop;
      Assert (Near (Compose_Poses (About_X, About_X).Rotation (0), 0.8, 1.0E-12),
              "same-axis rotation angles did not add");
      --  +(pi-0.01) and -(pi+0.01) about X are one physical rotation.
      declare
         Plus : constant World_To_Camera_Pose := Compose_Poses
           (Zero, (Rotation => [Pi_Value - 0.01, 0.0, 0.0], Translation => [0.0, 0.0, 0.0]));
         Minus : constant World_To_Camera_Pose := Compose_Poses
           (Zero, (Rotation => [-(Pi_Value + 0.01), 0.0, 0.0], Translation => [0.0, 0.0, 0.0]));
      begin
         Assert (Matrix_Difference (Rotation_Matrix_Of (Plus), Rotation_Matrix_Of (Minus)) < 1.0E-12,
                 "physically equivalent Rodrigues vectors yield different matrices");
      end;
   end Compose_Rodrigues;

   procedure Compose_Numbers (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check);
      Max : constant OpenCV.Float64_Value := OpenCV.Float64_Value'Last;
      Mild : constant World_To_Camera_Pose :=
        (Rotation => [0.3, 0.2, 0.1], Translation => [1.0, 2.0, 3.0]);
      Origin : constant Translation_Vector := [0.0, 0.0, 0.0];
      procedure Rejects (A, B : World_To_Camera_Pose; Message : String) is
      begin
         declare
            Value : constant World_To_Camera_Pose := Compose_Poses (A, B);
            pragma Unreferenced (Value);
         begin
            Assert (False, Message);
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Rejects;
   begin
      for Kind in Interfaces.Integer_32 range 0 .. 2 loop
         for Slot in 0 .. 5 loop
            declare
               Bad : World_To_Camera_Pose := Mild;
               Bad_Value : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Nonfinite (Kind));
            begin
               if Slot < 3 then
                  Bad.Rotation (Slot) := Bad_Value;
               else
                  Bad.Translation (Slot - 3) := Bad_Value;
               end if;
               Rejects (Bad, Mild, "nonfinite first accepted");
               Rejects (Mild, Bad, "nonfinite second accepted");
            end;
         end loop;
      end loop;
      Rejects ((Rotation => [1.0E160, 0.0, 0.0], Translation => Origin), Mild,
               "squared-norm overflow first accepted");
      Rejects (Mild, (Rotation => [Max, 0.0, 0.0], Translation => Origin),
               "maximum rotation second accepted");
      Rejects ((Rotation => [1.0E160, 1.0E160, 1.0E160], Translation => Origin), Mild,
               "three-component overflow accepted");
      Rejects ((Rotation => Mild.Rotation, Translation => [Max, Max, Max]),
               (Rotation => Mild.Rotation, Translation => [Max, Max, Max]),
               "unrepresentable composed translation accepted");
      declare
         Cancelled : constant World_To_Camera_Pose := Compose_Poses
           ((Rotation => [0.0, 0.0, 0.0], Translation => [Max, Max, Max]),
            (Rotation => [0.0, 0.0, 0.0], Translation => [-Max, -Max, -Max]));
         Large : constant World_To_Camera_Pose := Compose_Poses
           (Mild, (Rotation => [1.0E10, 0.0, 0.0], Translation => [1.0E300, -1.0E300, 1.0E299]));
      begin
         Assert (Cancelled.Translation = Origin, "maximum translations did not cancel");
         for Axis in 0 .. 2 loop
            Assert (Finite (Large.Translation (Axis)) and then Finite (Large.Rotation (Axis)),
                    "large representable composition returned nonfinite pose");
         end loop;
      end;
   end Compose_Numbers;
   procedure Compose_Layout (T : in out Fixture) is
      pragma Unreferenced (T);
      function Pose_Layout (Field : Interfaces.Integer_32) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_pose_layout";
      First : aliased ABI.C_Pose :=
        (RX => 0.0, RY => 0.0, RZ => 0.25, TX => 2.0, TY => -1.0, TZ => 0.5);
      Second : aliased ABI.C_Pose :=
        (RX => 0.5, RY => 0.0, RZ => 0.0, TX => 1.0, TY => 3.0, TZ => -2.0);
      Result : aliased ABI.C_Pose := (others => 0.0);
      Expected : constant World_To_Camera_Pose := Compose_Poses
        ((Rotation => [0.0, 0.0, 0.25], Translation => [2.0, -1.0, 0.5]),
         (Rotation => [0.5, 0.0, 0.0], Translation => [1.0, 3.0, -2.0]));
   begin
      Assert (ABI.C_Pose'Size = Natural (Pose_Layout (0)) * System.Storage_Unit
              and then ABI.C_Pose'Alignment = Natural (Pose_Layout (1)),
              "compose pose layout differs from compiler-derived C layout");
      Assert (ABI.Compose_RT (First'Access, Second'Access, Result'Access) = 0,
              "raw compose status");
      Assert (First.RZ = 0.25 and then First.TX = 2.0 and then Second.RX = 0.5
              and then Second.TZ = -2.0, "inputs changed across native compose");
      Assert (Result.RX = Interfaces.C.double (Expected.Rotation (0))
              and then Result.RY = Interfaces.C.double (Expected.Rotation (1))
              and then Result.RZ = Interfaces.C.double (Expected.Rotation (2))
              and then Result.TX = Interfaces.C.double (Expected.Translation (0))
              and then Result.TY = Interfaces.C.double (Expected.Translation (1))
              and then Result.TZ = Interfaces.C.double (Expected.Translation (2)),
              "all six compose output fields must interchange");
      Assert (ABI.Compose_RT (First'Access, Second'Access, First'Access) /= 0
              and then First.RZ = 0.25 and then First.TY = -1.0,
              "alias result==first must be rejected without clearing");
      Assert (ABI.Compose_RT (First'Access, Second'Access, Second'Access) /= 0
              and then Second.RX = 0.5 and then Second.TY = 3.0,
              "alias result==second must be rejected without clearing");
   end Compose_Layout;

   ---------------------------------------------------------------------------
   --  Task 018. These analytic matrices and centers are test oracles. They
   --  are not the production inversion, composition, or baseline algorithm.
   ---------------------------------------------------------------------------
   function Transpose (M : Rotation_Matrix) return Rotation_Matrix is
     ([[M (0, 0), M (1, 0), M (2, 0)],
       [M (0, 1), M (1, 1), M (2, 1)],
       [M (0, 2), M (1, 2), M (2, 2)]]);

   function Oracle_Center (R : Rotation_Matrix; T : Object_Point) return Object_Point is
     ([-R (0, 0) * T (0) - R (1, 0) * T (1) - R (2, 0) * T (2),
       -R (0, 1) * T (0) - R (1, 1) * T (1) - R (2, 1) * T (2),
       -R (0, 2) * T (0) - R (1, 2) * T (1) - R (2, 2) * T (2)]);

   function Point_Gap (Left, Right : Object_Point) return OpenCV.Float64_Value is
      Worst : OpenCV.Float64_Value := 0.0;
   begin
      for Axis in 0 .. 2 loop
         Worst := OpenCV.Float64_Value'Max (Worst, abs (Left (Axis) - Right (Axis)));
      end loop;
      return Worst;
   end Point_Gap;

   --  Closed-form axis-angle oracle for moderate test vectors only.
   function Oracle_Rotation (Vector : Rotation_Vector) return Rotation_Matrix is
      Angle : constant OpenCV.Float64_Value :=
        Math.Sqrt (Vector (0) ** 2 + Vector (1) ** 2 + Vector (2) ** 2);
      Axis : constant Object_Point :=
        [Vector (0) / Angle, Vector (1) / Angle, Vector (2) / Angle];
      Cosine : constant OpenCV.Float64_Value := Math.Cos (Angle);
      Sine : constant OpenCV.Float64_Value := Math.Sin (Angle);
      Cross : constant Rotation_Matrix :=
        [[0.0, -Axis (2), Axis (1)], [Axis (2), 0.0, -Axis (0)],
         [-Axis (1), Axis (0), 0.0]];
   begin
      return Result : Rotation_Matrix do
         for Row in 0 .. 2 loop
            for Col in 0 .. 2 loop
               Result (Row, Col) := (1.0 - Cosine) * Axis (Row) * Axis (Col)
                 + Sine * Cross (Row, Col);
               if Row = Col then
                  Result (Row, Col) := Result (Row, Col) + Cosine;
               end if;
            end loop;
         end loop;
      end return;
   end Oracle_Rotation;

   procedure Assert_Rigid
     (Pose : World_To_Camera_Pose; Expected_R : Rotation_Matrix;
      Expected_T : Object_Point; Message : String)
   is
   begin
      Assert (Matrix_Difference (Rotation_Matrix_Of (Pose), Expected_R) < 1.0E-12, Message);
      Assert (Point_Gap (Object_Point (Pose.Translation), Expected_T) < 1.0E-12, Message);
   end Assert_Rigid;

   procedure Motion_Rejects (First, Second : World_To_Camera_Pose; Message : String) is
   begin
      declare
         Value : constant Camera_Motion_Result := Measure_Camera_Motion (First, Second);
         pragma Unreferenced (Value);
      begin
         Assert (False, Message);
      end;
   exception
      when OpenCV.OpenCV_Error => null;
   end Motion_Rejects;

   procedure Motion_Identity (T : in out Fixture) is
      pragma Unreferenced (T);
      Identity : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, 0.0], Translation => [0.0, 0.0, 0.0]);
      Inverse : constant World_To_Camera_Pose := Invert_Pose (Identity);
      Relative : constant World_To_Camera_Pose := Camera_To_Camera_Pose (Identity, Identity);
      Motion : constant Camera_Motion_Result := Measure_Camera_Motion (Identity, Identity);
      Eye : constant Rotation_Matrix :=
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
   begin
      Assert_Rigid (Inverse, Eye, [0.0, 0.0, 0.0], "identity inverse");
      Assert_Rigid (Relative, Eye, [0.0, 0.0, 0.0], "identical relative pose");
      Assert (not Motion.Baseline_Direction_Defined, "identity baseline direction is defined");
      Assert (Motion.Baseline_Distance = 0.0, "identity baseline is not exact zero");
      Assert (Motion.Displacement_World = Object_Point'[0.0, 0.0, 0.0],
              "identity displacement is not exact zero");
      declare
         Pose : constant World_To_Camera_Pose :=
           (Rotation => [0.3, -0.2, 0.5], Translation => [1.2, -0.7, 3.4]);
         Same : constant Camera_Motion_Result := Measure_Camera_Motion (Pose, Pose);
      begin
         Assert (Same.Baseline_Distance = 0.0 and then not Same.Baseline_Direction_Defined,
                 "identical nonidentity poses must have exact zero baseline");
         Assert_Rigid (Same.Relative_Pose, Eye, [0.0, 0.0, 0.0],
                       "identical nonidentity relative pose");
      end;
   end Motion_Identity;

   procedure Motion_Translation (T : in out Fixture) is
      pragma Unreferenced (T);
      --  Identity rotations: analytic centers are exactly -t.
      First : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, 0.0], Translation => [1.0, -2.0, 4.0]);
      Second : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, 0.0], Translation => [5.0, 1.0, -3.0]);
      Relative : constant World_To_Camera_Pose := Camera_To_Camera_Pose (First, Second);
      Motion : constant Camera_Motion_Result := Measure_Camera_Motion (First, Second);
      Eye : constant Rotation_Matrix :=
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
      Expected_T : constant Object_Point := [4.0, 3.0, -7.0];
      Expected_C1 : constant Object_Point := [-1.0, 2.0, -4.0];
      Expected_C2 : constant Object_Point := [-5.0, -1.0, 3.0];
      Expected_D : constant Object_Point := [-4.0, -3.0, 7.0];
   begin
      Assert_Rigid (Relative, Eye, Expected_T, "translation-only relative pose");
      Assert (Point_Gap (Motion.First_Center_World, Expected_C1) < 1.0E-12, "first center");
      Assert (Point_Gap (Motion.Second_Center_World, Expected_C2) < 1.0E-12, "second center");
      Assert (Point_Gap (Motion.Displacement_World, Expected_D) < 1.0E-12, "world displacement");
      Assert (Point_Gap (Motion.Second_Center_In_First, Expected_D) < 1.0E-12,
              "identity-frame center in first camera");
      Assert (Near (Motion.Baseline_Distance, Math.Sqrt (74.0), 1.0E-12), "baseline sqrt(74)");
      Assert (Motion.Baseline_Direction_Defined, "nonzero baseline direction missing");
      Assert (Point_Gap (Motion.Baseline_Direction_World,
                         [Expected_D (0) / Motion.Baseline_Distance,
                          Expected_D (1) / Motion.Baseline_Distance,
                          Expected_D (2) / Motion.Baseline_Distance]) < 1.0E-12,
              "baseline direction");
      Assert (abs (Motion.Relative_Pose.Translation (0) - Motion.Displacement_World (0)) > 1.0,
              "relative translation must not be copied from world displacement");
   end Motion_Translation;

   procedure Motion_Noncommuting (T : in out Fixture) is
      pragma Unreferenced (T);
      --  R1 = Rz(pi/3), t1 = (1, 0, 0); R2 = Rx(pi/4), t2 = (0, 2, 0).
      R1 : constant Rotation_Matrix := Rot_Z (Pi_Value / 3.0);
      R2 : constant Rotation_Matrix := Rot_X (Pi_Value / 4.0);
      T1 : constant Object_Point := [1.0, 0.0, 0.0];
      T2 : constant Object_Point := [0.0, 2.0, 0.0];
      First : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, Pi_Value / 3.0], Translation => Translation_Vector (T1));
      Second : constant World_To_Camera_Pose :=
        (Rotation => [Pi_Value / 4.0, 0.0, 0.0], Translation => Translation_Vector (T2));
      Relative : constant World_To_Camera_Pose :=
        Camera_To_Camera_Pose (First => First, Second => Second);
      Reversed : constant World_To_Camera_Pose :=
        Camera_To_Camera_Pose (First => Second, Second => First);
      Expected_R : constant Rotation_Matrix := Mul (R2, Transpose (R1));
      Expected_T : constant Object_Point :=
        [T2 (0) - (Expected_R (0, 0) * T1 (0) + Expected_R (0, 1) * T1 (1) + Expected_R (0, 2) * T1 (2)),
         T2 (1) - (Expected_R (1, 0) * T1 (0) + Expected_R (1, 1) * T1 (1) + Expected_R (1, 2) * T1 (2)),
         T2 (2) - (Expected_R (2, 0) * T1 (0) + Expected_R (2, 1) * T1 (1) + Expected_R (2, 2) * T1 (2))];
      Swapped_R : constant Rotation_Matrix := Mul (R1, Transpose (R2));
   begin
      Assert_Rigid (Relative, Expected_R, Expected_T, "Rz then Rx relative pose");
      Assert (Matrix_Difference (Rotation_Matrix_Of (Reversed), Swapped_R) < 1.0E-12,
              "reversed cameras do not invert the rotation order");
      Assert (Matrix_Difference (Expected_R, Swapped_R) > 0.1, "rotation order was commutative");
      Assert (Matrix_Difference (Rotation_Matrix_Of (Compose_Poses (Invert_Pose (First), Second)),
                                 Expected_R) < 1.0E-12,
              "relative pose disagrees with Compose_Poses");
   end Motion_Noncommuting;

   procedure Motion_Pure_Rotation (T : in out Fixture) is
      pragma Unreferenced (T);
      --  Build t = -R_native * C for a common mathematical center;
      --  native center evaluation may retain a floating-point residue.
      Center : constant Object_Point := [3.0, -1.0, 2.0];
      First_Rotation : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, Pi_Value / 3.0], Translation => [0.0, 0.0, 0.0]);
      Second_Rotation : constant World_To_Camera_Pose :=
        (Rotation => [Pi_Value / 4.0, 0.0, 0.0], Translation => [0.0, 0.0, 0.0]);
      R1 : constant Rotation_Matrix := Rotation_Matrix_Of (First_Rotation);
      R2 : constant Rotation_Matrix := Rotation_Matrix_Of (Second_Rotation);
      T1 : constant Object_Point :=
        [-(R1 (0, 0) * Center (0) + R1 (0, 1) * Center (1) + R1 (0, 2) * Center (2)),
         -(R1 (1, 0) * Center (0) + R1 (1, 1) * Center (1) + R1 (1, 2) * Center (2)),
         -(R1 (2, 0) * Center (0) + R1 (2, 1) * Center (1) + R1 (2, 2) * Center (2))];
      T2 : constant Object_Point :=
        [-(R2 (0, 0) * Center (0) + R2 (0, 1) * Center (1) + R2 (0, 2) * Center (2)),
         -(R2 (1, 0) * Center (0) + R2 (1, 1) * Center (1) + R2 (1, 2) * Center (2)),
         -(R2 (2, 0) * Center (0) + R2 (2, 1) * Center (1) + R2 (2, 2) * Center (2))];
      First : constant World_To_Camera_Pose :=
        (Rotation => First_Rotation.Rotation, Translation => Translation_Vector (T1));
      Second : constant World_To_Camera_Pose :=
        (Rotation => Second_Rotation.Rotation, Translation => Translation_Vector (T2));
      Motion : constant Camera_Motion_Result := Measure_Camera_Motion (First, Second);
   begin
      Assert (Point_Gap (Motion.First_Center_World, Center) < 1.0E-12, "shared first center");
      Assert (Point_Gap (Motion.Second_Center_World, Center) < 1.0E-12, "shared second center");
      Assert (Point_Gap (Motion.Displacement_World, [0.0, 0.0, 0.0]) < 1.0E-15,
              "pure rotation displacement");
      Assert (Motion.Baseline_Distance < 1.0E-15, "pure rotation baseline");
      Assert (Motion.Baseline_Direction_Defined = (Motion.Baseline_Distance /= 0.0),
              "direction must follow exact zero, not a platform-specific rounding residue");
      declare
         At_Origin : constant Camera_Motion_Result :=
           Measure_Camera_Motion (First_Rotation, Second_Rotation);
      begin
         Assert (At_Origin.Baseline_Distance = 0.0
                 and then not At_Origin.Baseline_Direction_Defined
                 and then At_Origin.Displacement_World = Object_Point'[0.0, 0.0, 0.0],
                 "pure rotation at common origin must have exact zero baseline");
      end;
      Assert (Matrix_Difference (Rotation_Matrix_Of (Motion.Relative_Pose),
                                 Mul (R2, Transpose (R1))) < 1.0E-12,
              "pure rotation relative orientation");
      Assert (Point_Gap (Object_Point (Motion.Relative_Pose.Translation), [0.0, 0.0, 0.0]) < 1.0E-12,
              "pure rotation relative translation");
   end Motion_Pure_Rotation;

   procedure Motion_General (T : in out Fixture) is
      pragma Unreferenced (T);
      T1 : constant Object_Point := [0.2, -0.3, 0.8];
      T2 : constant Object_Point := [-1.5, 0.4, 2.2];
      First : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.35, 0.22], Translation => Translation_Vector (T1));
      Second : constant World_To_Camera_Pose :=
        (Rotation => [0.41, 0.0, -0.17], Translation => Translation_Vector (T2));
      R1 : constant Rotation_Matrix := Oracle_Rotation (First.Rotation);
      R2 : constant Rotation_Matrix := Oracle_Rotation (Second.Rotation);
      C1 : constant Object_Point := Oracle_Center (R1, T1);
      C2 : constant Object_Point := Oracle_Center (R2, T2);
      Displacement : constant Object_Point :=
        [C2 (0) - C1 (0), C2 (1) - C1 (1), C2 (2) - C1 (2)];
      Relative_R : constant Rotation_Matrix := Mul (R2, Transpose (R1));
      Relative_T : constant Object_Point :=
        [T2 (0) - (Relative_R (0, 0) * T1 (0) + Relative_R (0, 1) * T1 (1) + Relative_R (0, 2) * T1 (2)),
         T2 (1) - (Relative_R (1, 0) * T1 (0) + Relative_R (1, 1) * T1 (1) + Relative_R (1, 2) * T1 (2)),
         T2 (2) - (Relative_R (2, 0) * T1 (0) + Relative_R (2, 1) * T1 (1) + Relative_R (2, 2) * T1 (2))];
      In_First : constant Object_Point := Apply (R1, Displacement);
      Motion : constant Camera_Motion_Result := Measure_Camera_Motion (First, Second);
      Point : constant Object_Point := [1.25, -0.5, 3.5];
      Direction : constant World_Direction := [0.2, 0.4, -0.7];
      Direct_Second : constant Object_Point :=
        [R2 (0, 0) * Point (0) + R2 (0, 1) * Point (1) + R2 (0, 2) * Point (2) + T2 (0),
         R2 (1, 0) * Point (0) + R2 (1, 1) * Point (1) + R2 (1, 2) * Point (2) + T2 (1),
         R2 (2, 0) * Point (0) + R2 (2, 1) * Point (1) + R2 (2, 2) * Point (2) + T2 (2)];
   begin
      Assert_Rigid (Motion.Relative_Pose, Relative_R, Relative_T, "general relative pose");
      Assert (Point_Gap (Motion.First_Center_World, C1) < 1.0E-12, "general first center");
      Assert (Point_Gap (Motion.Second_Center_World, C2) < 1.0E-12, "general second center");
      Assert (Point_Gap (Motion.Displacement_World, Displacement) < 1.0E-12, "general displacement");
      Assert (Point_Gap (Motion.Second_Center_In_First, In_First) < 1.0E-12, "center in first frame");
      Assert (Near (Motion.Baseline_Distance,
                    Math.Sqrt (Displacement (0) ** 2 + Displacement (1) ** 2
                               + Displacement (2) ** 2), 1.0E-12), "general baseline");
      Assert (Point_Gap (World_To_Camera_Point (Motion.Relative_Pose,
                                                World_To_Camera_Point (First, Point)),
                         Direct_Second) < 1.0E-11,
              "relative point transform disagrees with the second camera");
      Assert (Point_Gap (Object_Point (World_To_Camera_Direction (Motion.Relative_Pose,
                            World_Direction (World_To_Camera_Direction (First, Direction)))),
                         Apply (R2, Object_Point (Direction))) < 1.0E-12,
              "relative direction transform");
   end Motion_General;

   procedure Motion_Roundtrip (T : in out Fixture) is
      pragma Unreferenced (T);
      Pose : constant World_To_Camera_Pose :=
        (Rotation => [0.3, -0.2, 0.5], Translation => [1.2, -0.7, 3.4]);
      Inverse : constant World_To_Camera_Pose := Invert_Pose (Pose);
      Twice : constant World_To_Camera_Pose := Invert_Pose (Inverse);
      Point : constant Object_Point := [0.4, -1.1, 2.0];
      Direction : constant World_Direction := [0.6, 0.1, -0.3];
      Camera_Point : constant Object_Point := World_To_Camera_Point (Pose, Point);
      Restored : constant Object_Point := World_To_Camera_Point (Inverse, Camera_Point);
      Camera_Direction_Value : constant Camera_Direction :=
        World_To_Camera_Direction (Pose, Direction);
      Restored_Direction : constant World_Direction :=
        World_Direction (World_To_Camera_Direction
          (Inverse, World_Direction (Camera_Direction_Value)));
      First : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, Pi_Value / 3.0], Translation => [1.0, 2.0, 3.0]);
      Second : constant World_To_Camera_Pose :=
        (Rotation => [Pi_Value / 4.0, 0.0, 0.0], Translation => [-2.0, 0.5, 1.5]);
      Forward : constant World_To_Camera_Pose :=
        Camera_To_Camera_Pose (First => First, Second => Second);
      Backward : constant World_To_Camera_Pose :=
        Camera_To_Camera_Pose (First => Second, Second => First);
      Swapped : constant Camera_Motion_Result :=
        Measure_Camera_Motion (First => Second, Second => First);
      Original : constant Camera_Motion_Result :=
        Measure_Camera_Motion (First => First, Second => Second);
   begin
      Assert (Matrix_Difference (Rotation_Matrix_Of (Twice), Rotation_Matrix_Of (Pose)) < 1.0E-12,
              "double inversion changed the rotation");
      Assert (Point_Gap (Object_Point (Twice.Translation), Object_Point (Pose.Translation)) < 1.0E-12,
              "double inversion changed the translation");
      Assert (Point_Gap (Restored, Point) < 1.0E-11, "pose then inverse did not restore the point");
      Assert (Point_Gap (Object_Point (Restored_Direction), Object_Point (Direction)) < 1.0E-12,
              "pose then inverse did not restore the direction");
      Assert (Point_Gap (World_To_Camera_Point (Backward, World_To_Camera_Point (Forward, Point)),
                         Point) < 1.0E-11,
              "forward then backward relative motion");
      Assert (Matrix_Difference (Rotation_Matrix_Of (Backward),
                                 Transpose (Rotation_Matrix_Of (Forward))) < 1.0E-12,
              "swapped cameras are not the inverse rotation");
      for Axis in 0 .. 2 loop
         Assert (Near (Swapped.Displacement_World (Axis), -Original.Displacement_World (Axis), 1.0E-12),
                 "swapped displacement is not negated");
      end loop;
      Assert (Near (Swapped.Baseline_Distance, Original.Baseline_Distance, 1.0E-12),
              "swapped baseline magnitude changed");
   end Motion_Roundtrip;

   procedure Motion_Projection (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant World_To_Camera_Pose :=
        (Rotation => [0.05, -0.04, 0.10], Translation => [0.10, -0.05, 0.30]);
      Second : constant World_To_Camera_Pose :=
        (Rotation => [-0.03, 0.06, 0.02], Translation => [-0.20, 0.10, 5.00]);
      World_Points : constant Object_Point_Array :=
        [[0.4, -0.3, 1.0], [-0.8, 0.5, 0.2], [1.1, 0.9, 0.7], [-0.2, -0.9, 1.4]];
      Intrinsics : constant Camera_Intrinsics := (800.0, 820.0, 320.0, 240.0);
      Brown : constant Distortion_Coefficients :=
        (K1 => -0.12, K2 => 0.03, P1 => 0.001, P2 => -0.0015, K3 => 0.004);
      Relative : constant World_To_Camera_Pose := Camera_To_Camera_Pose (First, Second);
      Worst : OpenCV.Float64_Value := 0.0;
   begin
      for Model in 1 .. 2 loop
         declare
            D : constant Distortion_Coefficients := (if Model = 1 then No_Distortion else Brown);
            Direct : constant Image_Point_Array :=
              Project_Points (World_Points, Intrinsics, D, Second);
         begin
            for I in World_Points'Range loop
               declare
                  In_First : constant Object_Point := World_To_Camera_Point (First, World_Points (I));
                  In_Second : constant Object_Point := World_To_Camera_Point (Relative, In_First);
                  Expected : constant Image_Point := Scalar_Project (In_Second, Intrinsics, D);
               begin
                  Worst := OpenCV.Float64_Value'Max
                    (Worst, OpenCV.Float64_Value'Max
                       (abs (Direct (I) (0) - Expected (0)), abs (Direct (I) (1) - Expected (1))));
               end;
            end loop;
         end;
      end loop;
      Ada.Text_IO.Put_Line ("Relative motion projection maximum pixel difference:" &
                            OpenCV.Float64_Value'Image (Worst));
      Assert (Worst < 1.0E-8, "relative projection disagrees with the second camera");
   end Motion_Projection;

   procedure Motion_Range (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check);
      Max : constant OpenCV.Float64_Value := OpenCV.Float64_Value'Last;
      Mild : constant World_To_Camera_Pose :=
        (Rotation => [0.2, -0.1, 0.3], Translation => [1.0, -2.0, 3.0]);
      Tiny : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, 0.0], Translation => [1.0E-300, -2.0E-300, 4.0E-300]);
      Large : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, 0.0], Translation => [1.0E15, -2.0E15, 2.0E15]);
      Shifted : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, 0.0], Translation => [1.0E15 + 3.0, -2.0E15, 2.0E15]);
      Identity : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, 0.0], Translation => [0.0, 0.0, 0.0]);
      Tiny_Motion : constant Camera_Motion_Result := Measure_Camera_Motion (Identity, Tiny);
      Huge : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, 0.0], Translation => [3.0E300, -4.0E300, 0.0]);
      Huge_Motion : constant Camera_Motion_Result := Measure_Camera_Motion (Identity, Huge);
      Large_Motion : constant Camera_Motion_Result := Measure_Camera_Motion (Large, Shifted);
   begin
      for Kind in Interfaces.Integer_32 range 0 .. 2 loop
         for Slot in 0 .. 5 loop
            declare
               Bad : World_To_Camera_Pose := Mild;
               Bad_Value : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Nonfinite (Kind));
            begin
               if Slot < 3 then
                  Bad.Rotation (Slot) := Bad_Value;
               else
                  Bad.Translation (Slot - 3) := Bad_Value;
               end if;
               Motion_Rejects (Bad, Mild, "nonfinite first motion accepted");
               Motion_Rejects (Mild, Bad, "nonfinite second motion accepted");
            end;
         end loop;
      end loop;
      Motion_Rejects ((Rotation => [1.0E160, 0.0, 0.0], Translation => [0.0, 0.0, 0.0]), Mild,
                      "unrepresentable rotation accepted");
      Motion_Rejects ((Rotation => [0.0, 0.0, 0.0], Translation => [Max, 0.0, 0.0]),
                      (Rotation => [0.0, 0.0, 0.0], Translation => [-Max, 0.0, 0.0]),
                      "unrepresentable center difference accepted");
      declare
         --  World displacement is 1.2*Last along X, but the second camera
         --  rotates it into two representable components (~0.849*Last).
         Extent : constant OpenCV.Float64_Value := 0.6 * Max;
         Component : constant OpenCV.Float64_Value := Extent / Math.Sqrt (2.0);
         First : constant World_To_Camera_Pose :=
           (Rotation => [0.0, 0.0, 0.0], Translation => [Extent, 0.0, 0.0]);
         Second : constant World_To_Camera_Pose :=
           (Rotation => [0.0, 0.0, Pi_Value / 4.0],
            Translation => [-Component, -Component, 0.0]);
         Relative : constant World_To_Camera_Pose := Camera_To_Camera_Pose (First, Second);
      begin
         Assert (Finite (Relative.Translation (0)) and then Finite (Relative.Translation (1)),
                 "displacement-overflow fixture must have representable relative translation");
         Motion_Rejects (First, Second, "world displacement overflow accepted");
      end;
      Motion_Rejects (Identity,
                      (Rotation => [0.0, 0.0, 0.0], Translation => [Max, Max, 0.0]),
                      "finite displacement with overflowing norm accepted");
      Assert (Finite (Tiny_Motion.Baseline_Distance)
              and then abs (Tiny_Motion.Baseline_Distance / 1.0E-300 - Math.Sqrt (21.0)) < 1.0E-12
              and then Tiny_Motion.Baseline_Direction_Defined,
              "tiny finite baseline was lost");
      Assert (Point_Gap (Tiny_Motion.Baseline_Direction_World,
                        [-1.0 / Math.Sqrt (21.0), 2.0 / Math.Sqrt (21.0),
                         -4.0 / Math.Sqrt (21.0)]) < 1.0E-12, "tiny baseline direction");
      Assert (Finite (Huge_Motion.Baseline_Distance)
              and then abs (Huge_Motion.Baseline_Distance / 1.0E300 - 5.0) < 1.0E-12,
              "large 3-4-5 baseline overflowed during squaring");
      Assert (Point_Gap (Huge_Motion.Baseline_Direction_World, [-0.6, 0.8, 0.0]) < 1.0E-12,
              "large baseline direction");
      Assert (Near (Large_Motion.Displacement_World (0), -3.0, 1.0E-6)
              and then Large_Motion.Displacement_World (1) = 0.0
              and then Large_Motion.Displacement_World (2) = 0.0,
              "large finite translation difference");
      Assert (Near (Large_Motion.Baseline_Distance, 3.0, 1.0E-6), "large finite baseline");
   end Motion_Range;

   procedure Motion_Frame_Order (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, Pi_Value / 2.0], Translation => [2.0, 0.0, 0.0]);
      Second : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, 0.0], Translation => [0.0, 3.0, 0.0]);
      Forward : constant Camera_Motion_Result :=
        Measure_Camera_Motion (First => First, Second => Second);
      Backward : constant Camera_Motion_Result :=
        Measure_Camera_Motion (First => Second, Second => First);
      --  Rz(pi/2): C1 = -R^T t = (0, 2, 0); C2 = (0, -3, 0).
      Expected : constant Object_Point := [0.0, -5.0, 0.0];
   begin
      Assert (Point_Gap (Forward.Displacement_World, Expected) < 1.0E-12, "frame-order displacement");
      Assert (Point_Gap (Backward.Displacement_World, [0.0, 5.0, 0.0]) < 1.0E-12,
              "reversed frame displacement sign");
      Assert (Forward.Relative_Pose.Translation (1) > 0.0, "relative translation sign");
      Assert (Forward.Displacement_World (1) < 0.0, "world displacement sign");
      Assert (abs (Forward.Relative_Pose.Translation (1) - Forward.Displacement_World (1)) > 1.0,
              "relative translation was given the world-displacement sign");
   end Motion_Frame_Order;

   ---------------------------------------------------------------------------
   --  Task 015 guarded native epipolar correction. Policy values below are
   --  explicit TEST/EXAMPLE choices justified by the measured fixtures; they
   --  are not library defaults or optimality guarantees.
   ---------------------------------------------------------------------------
   C_Policy : constant Epipolar_Correction_Options :=
     (Minimum_Second_Singular_Ratio       => 1.0E-3,
      Maximum_Third_Singular_Ratio        => 1.0E-9,
      Minimum_Epipolar_Line_Ratio         => 1.0E-12,
      Maximum_Corrected_Error_Pixels      => 1.0E-9,
      Absolute_Displacement_Excess_Pixels => 1.0E-9,
      Relative_Displacement_Excess        => 1.0E-9);
   C_Rect : constant Fundamental_Matrix := [[0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]];
   C_Vert : constant Fundamental_Matrix := [[0.0, 0.0, 1.0], [0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]];

   function Rejected
     (Matrix : Fundamental_Matrix; First, Second : Image_Point_Array;
      Options : Epipolar_Correction_Options) return Boolean is
   begin
      declare
         R : constant Corrected_Epipolar_Matches :=
           Correct_Epipolar_Matches (Matrix, First, Second, Options);
         pragma Unreferenced (R);
      begin
         return False;
      end;
   exception
      when OpenCV.OpenCV_Error => return True;
   end Rejected;

   --  Shared structural checks: one-based, finite diagnostics, Ada-recomputed
   --  residual/displacement agree with published diagnostics, bound satisfied.
   procedure Check_Correction
     (R : Corrected_Epipolar_Matches; F : Fundamental_Matrix;
      First, Second : Image_Point_Array; Options : Epipolar_Correction_Options)
   is
   begin
      Assert (R.Count = First'Length and then R.First_Points'First = 1 and then
              R.Second_Points'First = 1 and then R.Diagnostics'First = 1, "one-based result");
      for I in 1 .. R.Count loop
         declare
            D : Epipolar_Correction_Diagnostic renames R.Diagnostics (I);
            P1 : constant Image_Point := First (First'First + (I - 1));
            P2 : constant Image_Point := Second (Second'First + (I - 1));
            Q1 : constant Image_Point := R.First_Points (I);
            Q2 : constant Image_Point := R.Second_Points (I);
            E : constant Epipolar_Error_Result := Maximum_Epipolar_Error (F, Q1, Q2);
            Displacement : constant OpenCV.Float64_Value := Math.Sqrt
              ((Q1 (0) - P1 (0))**2 + (Q1 (1) - P1 (1))**2 + (Q2 (0) - P2 (0))**2 + (Q2 (1) - P2 (1))**2);
         begin
            Assert (Finite (D.Original_Error_Pixels) and then Finite (D.Corrected_Error_Pixels) and then
                    Finite (D.Total_Displacement_Pixels) and then Finite (D.One_Sided_Bound_Pixels),
                    "finite diagnostics");
            Assert (E.Defined and then E.Maximum_Error_Pixels <= Options.Maximum_Corrected_Error_Pixels,
                    "independent Ada residual within policy");
            Assert (Near (Displacement, D.Total_Displacement_Pixels,
                          1.0E-12 * (1.0 + Displacement)), "displacement diagnostic agrees");
            Assert (D.Total_Displacement_Pixels <= D.One_Sided_Bound_Pixels *
                      (1.0 + Options.Relative_Displacement_Excess) +
                      Options.Absolute_Displacement_Excess_Pixels, "one-sided necessary bound");
         end;
      end loop;
   end Check_Correction;

   procedure Correction_Rectified (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant Image_Point_Array := [[100.0, 20.0], [300.0, 40.0], [-50.0, 7.0]];
      Second : constant Image_Point_Array := [[90.0, 24.0], [250.0, 30.0], [-60.0, -3.0]];
      Saved1 : constant Image_Point_Array := First;
      R : constant Corrected_Epipolar_Matches := Correct_Epipolar_Matches (C_Rect, First, Second, C_Policy);
      Midpoint : constant Reprojection_Error_Array := [22.0, 35.0, 2.0];
   begin
      Check_Correction (R, C_Rect, First, Second, C_Policy);
      for I in 1 .. 3 loop
         --  Independent analytic optimum: equal x, y moved to the midpoint.
         Assert (Near (R.First_Points (I) (0), First (I) (0), 1.0E-12) and then
                 Near (R.First_Points (I) (1), Midpoint (I), 1.0E-12) and then
                 Near (R.Second_Points (I) (0), Second (I) (0), 1.0E-12) and then
                 Near (R.Second_Points (I) (1), Midpoint (I), 1.0E-12), "rectified analytic oracle");
      end loop;
      Assert (Near (R.Diagnostics (1).Total_Displacement_Pixels, Math.Sqrt (8.0), 1.0E-12) and then
              R.Diagnostics (1).One_Sided_Bound_Pixels = 4.0 and then
              R.Diagnostics (1).Original_Error_Pixels = 4.0 and then
              R.Diagnostics (1).Corrected_Error_Pixels = 0.0, "sqrt(8) displacement and 4-pixel bound");
      Assert (First = Saved1, "inputs unchanged");
      Ada.Text_IO.Put_Line ("correction rectified: displacement=" &
        OpenCV.Float64_Value'Image (R.Diagnostics (1).Total_Displacement_Pixels) & " bound=4");
   end Correction_Rectified;

   procedure Correction_Vertical (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant Image_Point_Array := [[20.0, 100.0], [5.0, 5.0]];
      Second : constant Image_Point_Array := [[24.0, 90.0], [9.0, 40.0]];
      R : constant Corrected_Epipolar_Matches := Correct_Epipolar_Matches (C_Vert, First, Second, C_Policy);
   begin
      Check_Correction (R, C_Vert, First, Second, C_Policy);
      Assert (Near (R.First_Points (1) (0), 22.0, 1.0E-12) and then Near (R.First_Points (1) (1), 100.0, 1.0E-12)
              and then Near (R.Second_Points (2) (0), 7.0, 1.0E-12)
              and then Near (R.Second_Points (2) (1), 40.0, 1.0E-12), "vertical analytic oracle");
   end Correction_Vertical;

   procedure Correction_Scale (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant Image_Point_Array := [[100.0, 20.0], [300.0, 40.0]];
      Second : constant Image_Point_Array := [[90.0, 24.0], [250.0, 30.0]];
      Reference : constant Corrected_Epipolar_Matches :=
        Correct_Epipolar_Matches (C_Rect, First, Second, C_Policy);
      F : Fundamental_Matrix;
   begin
      for Scale of Reprojection_Error_Array'[-1.0, 2.0, 0.5, 1.0E-100, 1.0E-40, 1.0E40, 1.0E100,
                                             1.0E-300, 1.0E300]
      loop
         F := C_Rect;
         for V of F loop
            V := V * Scale;
         end loop;
         declare
            R : constant Corrected_Epipolar_Matches := Correct_Epipolar_Matches (F, First, Second, C_Policy);
         begin
            Assert (R = Reference, "F scale/sign does not change the guarded result");
         end;
      end loop;
      --  Coefficient ranges that cannot survive normalization are rejected,
      --  not silently rounded into a different matrix.
      F := [[0.0, 0.0, 0.0], [0.0, 0.0, -1.0E300], [0.0, 1.0E300, 4.0E-300]];
      Assert (Rejected (F, First, Second, C_Policy), "normalization underflow rejected");
   end Correction_Scale;

   --  Research fixtures (rotated singular-value constructions, exact digits).
   C_Rank_One : constant Fundamental_Matrix :=
     [[0.45419899723237056, 0.14050021438031315, 0.59912146691828327],
      [0.38256653778680805, 0.11834169802510941, 0.50463305007126513], [0.0, 0.0, 0.0]];
   C_Near_Rank_One : constant Fundamental_Matrix :=
     [[0.45419917258335846, 0.1404996475182396, 0.59912146691828327],
      [0.38256632960277992, 0.1183423710274762, 0.50463305007126513],
      [-1.1508098899676867E-07, 3.7202555194225961E-07, 0.0]];
   C_General : constant Fundamental_Matrix :=
     [[0.54187449117452635, -0.14293082240506408, 0.59912146691828327],
      [0.27847452372631143, 0.45484288141974499, 0.50463305007126513],
      [-0.057540494498384331, 0.18601277597112981, 0.0]];

   procedure Correction_Conditioning (T : in out Fixture) is
      pragma Unreferenced (T);
      P1 : constant Image_Point_Array := [[100.0, 20.0]];
      P2 : constant Image_Point_Array := [[90.0, 24.0]];
      --  Exact diagonal singular values: boundary comparisons are inclusive.
      D2 : constant Fundamental_Matrix := [[1.0, 0.0, 0.0], [0.0, 1.0E-3, 0.0], [0.0, 0.0, 0.0]];
      D3 : constant Fundamental_Matrix := [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 2.0**(-30)]];
      Full : constant Fundamental_Matrix := [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 10.0]];
      O : Epipolar_Correction_Options := C_Policy;
   begin
      O.Minimum_Second_Singular_Ratio := 1.0E-3;
      Assert (not Rejected (D2, P1, P2, O), "s2/s1 exactly at minimum accepted");
      --  Succ of the stored machine value, not of the exact static literal 1/1000.
      O.Minimum_Second_Singular_Ratio := OpenCV.Float64_Value'Succ (O.Minimum_Second_Singular_Ratio);
      Assert (Rejected (D2, P1, P2, O), "s2/s1 one ulp below caller minimum rejected");
      O := C_Policy;
      O.Maximum_Third_Singular_Ratio := 2.0**(-30);
      Assert (not Rejected (D3, P1, P2, O), "s3/s1 exactly at maximum accepted");
      O.Maximum_Third_Singular_Ratio := OpenCV.Float64_Value'Pred (O.Maximum_Third_Singular_Ratio);
      Assert (Rejected (D3, P1, P2, O), "s3/s1 one ulp above caller maximum rejected");
      Assert (Rejected (Full, P1, P2, C_Policy), "full-rank F outside policy rejected");
      Assert (Rejected (C_Rank_One, [[-30.0, 55.0]], [[12.0, 70.0]], C_Policy), "rank-one F rejected");
      Assert (not Rejected (C_General, [[-30.0, 55.0]], [[12.0, 70.0]], C_Policy), "s2/s1=0.5 accepted");
      --  Stricter caller policy rejects the same well-conditioned matrix.
      O := C_Policy;
      O.Minimum_Second_Singular_Ratio := 0.75;
      Assert (Rejected (C_General, [[-30.0, 55.0]], [[12.0, 70.0]], O), "stricter policy rejects s2/s1=0.5");
   end Correction_Conditioning;

   procedure Correction_Native_Bound (T : in out Fixture) is
      pragma Unreferenced (T);
      --  Valid but deliberately permissive policy: conditioning admits these
      --  matrices so ONLY the post-native one-sided bound can reject them.
      Permissive : constant Epipolar_Correction_Options := (1.0E-20, 1.0E-21, 0.0, 1.0E3, 1.0E-9, 1.0E-9);
      Near_Policy : constant Epipolar_Correction_Options := (1.0E-7, 1.0E-15, 0.0, 1.0E3, 1.0E-9, 1.0E-9);
   begin
      --  Research: native displacement 54.27 vs feasible one-sided bound 11.15.
      Assert (Rejected (C_Rank_One, [[-30.0, 55.0]], [[12.0, 70.0]], Permissive),
              "rank-one finite-but-wrong native output rejected by one-sided bound");
      Assert (Rejected (C_Near_Rank_One, [[-30.0, 55.0]], [[12.0, 70.0]], Near_Policy),
              "near-rank-one native displacement above feasible bound rejected");
      --  A point where native output stays within the bound is still accepted.
      Assert (not Rejected (C_Near_Rank_One, [[100.0, 20.0]], [[90.0, 24.0]], Near_Policy),
              "bound is necessary, not sufficient: some near-rank-one outputs pass");
   end Correction_Native_Bound;

   procedure Correction_Epipoles (T : in out Fixture) is
      pragma Unreferenced (T);
      Ex : constant Fundamental_Matrix := [[0.0, -1.0, 50.0], [1.0, 0.0, -100.0], [-50.0, 100.0, 0.0]];
      Line : Epipolar_Correction_Options := C_Policy;
      Loose : constant Epipolar_Correction_Options := (1.0E-3, 1.0E-9, 0.0, 1.0E-2, 1.0E-9, 1.0E-9);
      Near_Result : Boolean;
   begin
      Assert (Rejected (Ex, [[100.0, 50.0]], [[130.0, 75.0]], Loose), "exactly at epipole: undefined line");
      Line.Minimum_Epipolar_Line_Ratio := 1.0E-12;
      Assert (Rejected (Ex, [[100.0 + 1.0E-9, 50.0]], [[130.0, 75.0]], Line),
              "near epipole below caller line quality rejected (quality 5.7e-14)");
      Assert (not Rejected (Ex, [[101.0, 50.0]], [[130.0, 75.0]], Line),
              "line quality 5.6e-5 above caller threshold accepted");
      Line.Minimum_Epipolar_Line_Ratio := 1.0E-4;
      Assert (Rejected (Ex, [[101.0, 50.0]], [[130.0, 75.0]], Line), "stricter line policy rejects");
      Near_Result := Rejected (Ex, [[100.0 + 1.0E-9, 50.0]], [[130.0, 75.0]], Loose);
      --  Version-specific native behaviour, read from the private metadata import.
      if Interfaces.C.Strings.Value (ABI.Native_Version) (1) = '4' then
         Assert (Near_Result, "4.x near-epipole nonoptimal output (0.17 vs 6.4e-10) rejected");
      else
         Assert (not Near_Result, "5.0 near-epipole output stays within the bound");
      end if;
   end Correction_Epipoles;

   procedure Correction_Residual (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant Image_Point_Array := [[-30.0, 55.0], [100.0, 20.0], [7.0, 8.0], [7.0, 8.0]];
      Second : constant Image_Point_Array := [[12.0, 70.0], [90.0, 24.0], [9.0, 3.0], [9.0, 3.0]];
      Tight : Epipolar_Correction_Options := C_Policy;
      R : constant Corrected_Epipolar_Matches := Correct_Epipolar_Matches (C_General, First, Second, C_Policy);
      Worst : OpenCV.Float64_Value := 0.0;
   begin
      Check_Correction (R, C_General, First, Second, C_Policy);
      for D of R.Diagnostics loop
         Worst := OpenCV.Float64_Value'Max (Worst, D.Corrected_Error_Pixels);
         Assert (D.Corrected_Error_Pixels < D.Original_Error_Pixels, "inconsistency reduced");
      end loop;
      for K in 0 .. 1 loop
         Assert (R.First_Points (3) (K) = R.First_Points (4) (K) and then
                 R.Second_Points (3) (K) = R.Second_Points (4) (K), "repeated correspondences corrected identically");
      end loop;
      Ada.Text_IO.Put_Line ("correction general rank-two: maximum corrected error=" &
        OpenCV.Float64_Value'Image (Worst));
      --  Zero residual tolerance: measured general-F residual ~1e-14 is rejected,
      --  while the exactly representable rectified case passes at 0.
      Tight.Maximum_Corrected_Error_Pixels := 0.0;
      Assert (Rejected (C_General, First, Second, Tight) or else Worst = 0.0,
              "residual policy is enforced exactly as configured");
      Assert (not Rejected (C_Rect, [[100.0, 20.0]], [[90.0, 24.0]], Tight), "exact residual 0 accepted");
      --  Identical correspondence on a rectified line: unchanged, zero bound.
      declare
         Same : constant Corrected_Epipolar_Matches :=
           Correct_Epipolar_Matches (C_Rect, [[5.0, 5.0]], [[5.0, 5.0]], Tight);
      begin
         Assert (Same.First_Points (1) (0) = 5.0 and then Same.First_Points (1) (1) = 5.0 and then Same.Diagnostics (1).One_Sided_Bound_Pixels = 0.0
                 and then Same.Diagnostics (1).Total_Displacement_Pixels = 0.0, "already consistent pair");
      end;
   end Correction_Residual;

   procedure Correction_Coordinates (T : in out Fixture) is
      pragma Unreferenced (T);
      Any_Line : Epipolar_Correction_Options := C_Policy;
      Big : Boolean;
   begin
      Any_Line.Minimum_Epipolar_Line_Ratio := 0.0;
      declare
         R : constant Corrected_Epipolar_Matches := Correct_Epipolar_Matches
           (C_Rect, [[1.0E100, 2.0E100]], [[9.0E99, 2.4E100]], Any_Line);
      begin
         Assert (abs (R.First_Points (1) (1) / 2.2E100 - 1.0) < 1.0E-12 and then
                 Finite (R.Diagnostics (1).Total_Displacement_Pixels),
                 "rectified large representable coordinates: finite scaled diagnostics");
      end;
      Assert (Rejected (C_Rect, [[1.0E100, 2.0E100]], [[9.0E99, 2.4E100]], C_Policy),
              "line-quality policy is coordinate-magnitude sensitive");
      Assert (not Rejected (C_General, [[1.0E3, 2.0E3]], [[9.0E2, 2.4E3]], C_Policy), "ordinary coordinates");
      --  Research: 4.x returns NaN from 1e5; 5.0 returns a large residual at 1e15.
      Big := Rejected (C_General, [[1.0E15, 2.0E15]], [[9.0E14, 2.4E15]], C_Policy);
      Assert (Big, "known native coordinate failure rejected on every version");
   end Correction_Coordinates;

   --  Independent nonidentity two-view oracle: E=[t]x R, F=K^-T E K^-1 built in
   --  Ada; observations projected directly (positive depth in both cameras).
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
   Stereo_Rotation : constant M3 :=
     Mul ([[Math.Cos (0.08), 0.0, Math.Sin (0.08)], [0.0, 1.0, 0.0], [-Math.Sin (0.08), 0.0, Math.Cos (0.08)]],
          [[1.0, 0.0, 0.0], [0.0, Math.Cos (0.03), -Math.Sin (0.03)], [0.0, Math.Sin (0.03), Math.Cos (0.03)]]);
   Stereo_T : constant Object_Point := [-0.75, 0.08, 0.12];
   function Stereo_F return Fundamental_Matrix is
      Tx : constant M3 := [[0.0, -Stereo_T (2), Stereo_T (1)], [Stereo_T (2), 0.0, -Stereo_T (0)],
                           [-Stereo_T (1), Stereo_T (0), 0.0]];
      K_Inv : constant M3 := [[1.0 / 800.0, 0.0, -320.0 / 800.0], [0.0, 1.0 / 820.0, -240.0 / 820.0],
                              [0.0, 0.0, 1.0]];
      K_Inv_T : constant M3 := [[K_Inv (0, 0), 0.0, 0.0], [0.0, K_Inv (1, 1), 0.0],
                                [K_Inv (0, 2), K_Inv (1, 2), 1.0]];
      F : constant M3 := Mul (K_Inv_T, Mul (Mul (Tx, Stereo_Rotation), K_Inv));
   begin
      return [for I in 0 .. 2 => [for J in 0 .. 2 => F (I, J)]];
   end Stereo_F;
   procedure Stereo_Views (Count : Positive; Noise : OpenCV.Float64_Value;
                           First, Second : out Image_Point_Array) is
   begin
      for J in 0 .. Count - 1 loop
         declare
            X : constant Object_Point := [OpenCV.Float64_Value ((J * 7) mod 17 - 8) * 0.24,
                                          OpenCV.Float64_Value ((J * 11) mod 19 - 9) * 0.19,
                                          4.0 + OpenCV.Float64_Value ((J * 13) mod 23) * 0.17];
            Y : Object_Point := Stereo_T;
            Sign : constant OpenCV.Float64_Value := (if J mod 2 = 0 then 1.0 else -1.0);
         begin
            for I in 0 .. 2 loop
               for K in 0 .. 2 loop
                  Y (I) := Y (I) + Stereo_Rotation (I, K) * X (K);
               end loop;
            end loop;
            Assert (X (2) > 0.0 and then Y (2) > 0.0, "positive depth in both cameras");
            First (First'First + J) := [800.0 * X (0) / X (2) + 320.0 + Sign * 0.7 * Noise,
                                        820.0 * X (1) / X (2) + 240.0 - 0.4 * Noise];
            Second (Second'First + J) := [800.0 * Y (0) / Y (2) + 320.0 - 0.3 * Noise,
                                          820.0 * Y (1) / Y (2) + 240.0 + Sign * 0.9 * Noise];
         end;
      end loop;
   end Stereo_Views;

   --  Measured on 4.1/4.10/5.0: s2/s1=0.0165, s3/s1~1e-22 for this pixel F.
   Stereo_Policy : constant Epipolar_Correction_Options := (1.0E-3, 1.0E-12, 1.0E-6, 1.0E-9, 1.0E-9, 1.0E-9);

   procedure Correction_Stereo (T : in out Fixture) is
      pragma Unreferenced (T);
      F : constant Fundamental_Matrix := Stereo_F;
   begin
      for Count of Inlier_Index_Array'[1, 6, 25] loop
         declare
            Clean1, Clean2, Noisy1, Noisy2 : Image_Point_Array (1 .. Count);
         begin
            Stereo_Views (Count, 0.0, Clean1, Clean2);
            Stereo_Views (Count, 1.0, Noisy1, Noisy2);
            declare
               Clean : constant Corrected_Epipolar_Matches :=
                 Correct_Epipolar_Matches (F, Clean1, Clean2, Stereo_Policy);
               Noisy : constant Corrected_Epipolar_Matches :=
                 Correct_Epipolar_Matches (F, Noisy1, Noisy2, Stereo_Policy);
               Max_Clean, Max_Noisy_In, Max_Noisy_Out : OpenCV.Float64_Value := 0.0;
            begin
               Check_Correction (Clean, F, Clean1, Clean2, Stereo_Policy);
               Check_Correction (Noisy, F, Noisy1, Noisy2, Stereo_Policy);
               for I in 1 .. Count loop
                  Max_Clean := OpenCV.Float64_Value'Max (Max_Clean, Clean.Diagnostics (I).Total_Displacement_Pixels);
                  Max_Noisy_In := OpenCV.Float64_Value'Max (Max_Noisy_In, Noisy.Diagnostics (I).Original_Error_Pixels);
                  Max_Noisy_Out := OpenCV.Float64_Value'Max
                    (Max_Noisy_Out, Noisy.Diagnostics (I).Corrected_Error_Pixels);
                  --  Independent geometry: corrected noisy points stay near the true projections.
                  Assert (Math.Sqrt ((Noisy.First_Points (I) (0) - Clean1 (I) (0))**2 +
                                     (Noisy.First_Points (I) (1) - Clean1 (I) (1))**2) < 2.0,
                          "corrected point near independent noiseless projection");
               end loop;
               Assert (Max_Clean < 1.0E-9, "noiseless observations essentially unchanged");
               Assert (Max_Noisy_Out < Max_Noisy_In, "controlled noise inconsistency reduced");
               Ada.Text_IO.Put_Line ("correction stereo N=" & Natural'Image (Count) & ": clean max disp=" &
                 OpenCV.Float64_Value'Image (Max_Clean) & " noisy max error in/out=" &
                 OpenCV.Float64_Value'Image (Max_Noisy_In) & "/" & OpenCV.Float64_Value'Image (Max_Noisy_Out));
            end;
         end;
      end loop;
   end Correction_Stereo;

   procedure Correction_RANSAC (T : in out Fixture) is
      pragma Unreferenced (T);
      First, Second : Image_Point_Array (1 .. 32);
   begin
      Stereo_Views (32, 0.0, First, Second);
      for I of Inlier_Index_Array'[2, 7, 15] loop
         Second (I) := [Second (I) (0) + 50.0, Second (I) (1) + 500.0];
      end loop;
      declare
         Saved1 : constant Image_Point_Array := First;
         Saved2 : constant Image_Point_Array := Second;
         Estimate : constant Fundamental_Estimate :=
           Estimate_Fundamental_RANSAC (First, Second, (0.1, 0.999));
      begin
         Assert (Found (Estimate), "existing native fundamental RANSAC found a model");
         declare
            F : constant Fundamental_Matrix := Fundamental (Estimate);
            Saved_F : constant Fundamental_Matrix := F;
            Selected : constant Inlier_Index_Array := Inliers (Estimate);
            Sel1, Sel2 : Image_Point_Array (101 .. 100 + Selected'Length);
            --  RANSAC fits observations to <=0.1 px; measured residual after
            --  native correction is well under the explicit 1e-6 tolerance.
            Policy : constant Epipolar_Correction_Options := (1.0E-3, 1.0E-6, 1.0E-6, 1.0E-6, 1.0E-9, 1.0E-9);
         begin
            for K in Selected'Range loop
               Sel1 (100 + K) := First (Selected (K));
               Sel2 (100 + K) := Second (Selected (K));
            end loop;
            declare
               R : constant Corrected_Epipolar_Matches := Correct_Epipolar_Matches (F, Sel1, Sel2, Policy);
            begin
               Check_Correction (R, F, Sel1, Sel2, Policy);
               Assert (R.Count = Selected'Length, "one output pair per selected inlier");
               Assert (F = Saved_F and then First = Saved1 and then Second = Saved2,
                       "F and original correspondences unchanged");
               Assert (Inliers (Estimate) = Selected, "inlier ordering unchanged");
               for K in 1 .. R.Count loop
                  Assert (R.Diagnostics (K).Original_Error_Pixels <= 0.1, "selected pairs are final-F inliers");
               end loop;
               Ada.Text_IO.Put_Line ("correction after RANSAC: selected=" & Natural'Image (R.Count));
            end;
         end;
      end;
   end Correction_RANSAC;

   procedure Correction_Bounds (T : in out Fixture) is
      pragma Unreferenced (T);
      First : constant Image_Point_Array (7 .. 11) :=
        [[100.0, 20.0], [300.0, 40.0], [-50.0, 7.0], [5.0, 5.0], [8.0, 1.0]];
      Second : constant Image_Point_Array (20 .. 24) :=
        [[90.0, 24.0], [250.0, 30.0], [-60.0, -3.0], [5.0, 5.0], [2.0, 3.0]];
      R : constant Corrected_Epipolar_Matches := Correct_Epipolar_Matches (C_Rect, First, Second, C_Policy);
      Empty : constant Image_Point_Array (5 .. 4) := [others => [0.0, 0.0]];
   begin
      Assert (R.Count = 5 and then R.First_Points'First = 1 and then R.First_Points'Last = 5 and then
              R.Second_Points'Last = 5 and then R.Diagnostics'Last = 5, "Count=5, all arrays 1 .. 5");
      for I in 1 .. 5 loop
         --  Paired by position: midpoint of the I-th elements of each source.
         Assert (Near (R.First_Points (I) (1),
                       (First (6 + I) (1) + Second (19 + I) (1)) / 2.0, 1.0E-12), "paired by position");
      end loop;
      Check_Correction (R, C_Rect, First, Second, C_Policy);
      declare
         E : constant Corrected_Epipolar_Matches := Correct_Epipolar_Matches (C_Rect, Empty, Empty, C_Policy);
      begin
         Assert (E.Count = 0 and then E.First_Points'Length = 0 and then E.Diagnostics'Length = 0,
                 "empty/empty valid");
      end;
      Assert (Rejected ([others => [others => 0.0]], Empty, Empty, C_Policy), "invalid F not empty success");
      Assert (Rejected (C_Rank_One, Empty, Empty, C_Policy), "empty batch still enforces SVD policy");
      Assert (Rejected (C_Rect, Empty, [[1.0, 2.0]], C_Policy), "mismatched empty/nonempty");
      Assert (Rejected (C_Rect, First, Second (20 .. 23), C_Policy), "mismatched lengths");
   end Correction_Bounds;

   procedure Correction_Upper_Bound (T : in out Fixture) is
      pragma Unreferenced (T);
      --  Regression: production positional indexing must not compute
      --  'First + I before subtracting 1. A lower bound of Positive'Last makes
      --  that intermediate overflow. The assertions below index the inputs
      --  only by their own 'First and never re-derive positions, so they test
      --  the production expression rather than repeating it.
      First : constant Image_Point_Array (Positive'Last .. Positive'Last) := [[100.0, 20.0]];
      Second : constant Image_Point_Array (5 .. 5) := [[90.0, 24.0]];
      Saved1 : constant Image_Point_Array := First;
      Saved2 : constant Image_Point_Array := Second;
   begin
      declare
         R : constant Corrected_Epipolar_Matches := Correct_Epipolar_Matches (C_Rect, First, Second, C_Policy);
      begin
         Assert (R.Count = 1, "Count = 1");
         Assert (R.First_Points'First = 1 and then R.First_Points'Last = 1 and then
                 R.Second_Points'First = 1 and then R.Second_Points'Last = 1 and then
                 R.Diagnostics'First = 1 and then R.Diagnostics'Last = 1, "all result arrays 1 .. 1");
         --  Analytic rectified optimum: x unchanged, both y at the midpoint 22.
         Assert (Near (R.First_Points (1) (0), 100.0, 1.0E-12) and then
                 Near (R.First_Points (1) (1), 22.0, 1.0E-12) and then
                 Near (R.Second_Points (1) (0), 90.0, 1.0E-12) and then
                 Near (R.Second_Points (1) (1), 22.0, 1.0E-12), "analytic midpoint at Positive'Last");
         Assert (Near (R.Diagnostics (1).Total_Displacement_Pixels, Math.Sqrt (8.0), 1.0E-12) and then
                 R.Diagnostics (1).One_Sided_Bound_Pixels = 4.0, "diagnostics at Positive'Last");
      end;
      Assert (First = Saved1 and then Second = Saved2 and then
              First'First = Positive'Last and then Second'First = 5, "inputs unchanged");
   exception
      when Constraint_Error =>
         Assert (False, "Constraint_Error escaped Correct_Epipolar_Matches at Positive'Last");
      when E : OpenCV.OpenCV_Error =>
         --  Production converts index-arithmetic Constraint_Error into this.
         Assert (False, "valid Positive'Last batch rejected: " & Ada.Exceptions.Exception_Message (E));
   end Correction_Upper_Bound;

   procedure Correction_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      --  Deliberately nonfinite test data (same convention as other invalid tests).
      pragma Suppress (Validity_Check);
      P1 : constant Image_Point_Array := [[100.0, 20.0]];
      P2 : constant Image_Point_Array := [[90.0, 24.0]];
      F : Fundamental_Matrix;
      O : Epipolar_Correction_Options;
   begin
      for Kind in Interfaces.Integer_32 range 0 .. 2 loop
         declare
            Invalid : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Nonfinite (Kind));
            Bad1 : Image_Point_Array := P1;
            Bad2 : Image_Point_Array := P2;
         begin
            F := C_Rect;
            F (1, 2) := Invalid;
            Assert (Rejected (F, P1, P2, C_Policy), "nonfinite F");
            Bad1 (1) (0) := Invalid;
            Assert (Rejected (C_Rect, Bad1, P2, C_Policy), "nonfinite first coordinate");
            Bad2 (1) (1) := Invalid;
            Assert (Rejected (C_Rect, P1, Bad2, C_Policy), "nonfinite second coordinate");
         end;
      end loop;
      Assert (Rejected ([others => [others => 0.0]], P1, P2, C_Policy), "zero F");
      --  Each option: nonfinite (all three kinds) and negative are invalid;
      --  1.5 is valid only for the two unbounded pixel tolerances.
      for Kind in Interfaces.Integer_32 range 0 .. 2 loop
         for Field in 1 .. 6 loop
            declare
               Invalid : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Nonfinite (Kind));
            begin
               O := C_Policy;
               case Field is
                  when 1 => O.Minimum_Second_Singular_Ratio := Invalid;
                  when 2 => O.Maximum_Third_Singular_Ratio := Invalid;
                  when 3 => O.Minimum_Epipolar_Line_Ratio := Invalid;
                  when 4 => O.Maximum_Corrected_Error_Pixels := Invalid;
                  when 5 => O.Absolute_Displacement_Excess_Pixels := Invalid;
                  when others => O.Relative_Displacement_Excess := Invalid;
               end case;
               Assert (Rejected (C_Rect, P1, P2, O), "nonfinite option rejected");
            end;
         end loop;
      end loop;
      for Field in 1 .. 6 loop
         for Value of Reprojection_Error_Array'[-1.0E-300, 1.5] loop
            O := C_Policy;
            case Field is
               when 1 => O.Minimum_Second_Singular_Ratio := Value;
               when 2 => O.Maximum_Third_Singular_Ratio := Value;
               when 3 => O.Minimum_Epipolar_Line_Ratio := Value;
               when 4 => O.Maximum_Corrected_Error_Pixels := Value;
               when 5 => O.Absolute_Displacement_Excess_Pixels := Value;
               when others => O.Relative_Displacement_Excess := Value;
            end case;
            Assert (Rejected (C_Rect, P1, P2, O) = (Value /= 1.5 or else Field in 1 | 2 | 3 | 6),
                    "documented option range enforced");
         end loop;
      end loop;
      O := C_Policy;
      O.Minimum_Second_Singular_Ratio := 0.0;
      Assert (Rejected (C_Rect, P1, P2, O), "zero minimum second ratio");
      O := C_Policy;
      O.Maximum_Third_Singular_Ratio := C_Policy.Minimum_Second_Singular_Ratio;
      Assert (Rejected (C_Rect, P1, P2, O), "third ratio must be below second ratio");
      --  Very large finite observations whose input-line arithmetic overflows.
      Assert (Rejected ([[0.0, -1.0, 50.0], [1.0, 0.0, -100.0], [-50.0, 100.0, 0.0]],
                        [[1.0E308, 1.0E308]], [[1.0, 1.0]], C_Policy), "nonrepresentable input line");
   end Correction_Invalid;

   procedure Correction_Layout (T : in out Fixture) is
      pragma Unreferenced (T);
      function Layout (Field : Interfaces.Integer_32) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_correction_options_layout";
      procedure Fill (Value : access ABI.C_Correction_Options)
        with Import, Convention => C, External_Name => "calib3d_test_fill_correction_options";
      function Read (Value : access constant ABI.C_Correction_Options) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_read_correction_options";
      function Read_F (Value : access constant ABI.C_Fundamental) return Interfaces.Integer_32
        with Import, Convention => C, External_Name => "calib3d_test_read_fundamental";
      Value : aliased ABI.C_Correction_Options := (others => 0.0);
      Sent : aliased constant ABI.C_Correction_Options := (0.75, 0.375, 0.03125, 6.5, 7.25, 0.875);
      Sent_F : aliased constant ABI.C_Fundamental := (11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0);
      type Positions is array (Natural range <>) of Natural;
      Offsets : constant Positions :=
        [Value.Minimum_Second_Singular_Ratio'Position, Value.Maximum_Third_Singular_Ratio'Position,
         Value.Minimum_Epipolar_Line_Ratio'Position, Value.Maximum_Corrected_Error_Pixels'Position,
         Value.Absolute_Displacement_Excess_Pixels'Position, Value.Relative_Displacement_Excess'Position];
   begin
      Assert (ABI.C_Correction_Options'Size = Natural (Layout (0)) * System.Storage_Unit, "options size");
      Assert (ABI.C_Correction_Options'Alignment = Natural (Layout (1)), "options alignment");
      for I in Offsets'Range loop
         Assert (Offsets (I) = Natural (Layout (Interfaces.Integer_32 (I + 2))), "options field offset");
      end loop;
      Fill (Value'Access);
      Assert (Value.Minimum_Second_Singular_Ratio = 0.125 and then Value.Maximum_Third_Singular_Ratio = 0.0625
              and then Value.Minimum_Epipolar_Line_Ratio = 0.25 and then Value.Maximum_Corrected_Error_Pixels = 3.5
              and then Value.Absolute_Displacement_Excess_Pixels = 4.75
              and then Value.Relative_Displacement_Excess = 0.5, "C-to-Ada options interchange, all six fields");
      Assert (Read (Sent'Access) = 1, "Ada-to-C options interchange, all six fields");
      Assert (Read_F (Sent_F'Access) = 1, "Ada-to-C fundamental interchange, all nine fields");
      Ada.Text_IO.Put_Line ("correction options layout: size=" & Interfaces.Integer_32'Image (Layout (0)) &
        " alignment=" & Interfaces.Integer_32'Image (Layout (1)) & "; six offsets + both interchanges PASS");
   end Correction_Layout;







   Metric_K : constant Camera_Intrinsics := (1.0, 1.0, 0.0, 0.0);
   Metric_First : constant World_To_Camera_Pose :=
     (Rotation => [0.0, 0.0, 0.0], Translation => [0.0, 0.0, 0.0]);
   Metric_Second : constant World_To_Camera_Pose :=
     (Rotation => [0.0, 0.0, 0.0], Translation => [-2.0, 0.0, 0.0]);
   Metric_Policy : constant Metric_Triangulation_Options := (0.0, 1.0E-8, 1.0E-6, 1.0E-6);

   function Metric_Rectified
     (A : Image_Point := [0.5, 0.125]; B : Image_Point := [0.25, 0.125];
      First : World_To_Camera_Pose := Metric_First;
      Second : World_To_Camera_Pose := Metric_Second;
      Policy : Metric_Triangulation_Options := Metric_Policy)
      return Metric_Triangulated_Point
   is
      V : constant Metric_Triangulated_Point_Array := Triangulate_World_Points
        ([1 => A], [1 => B], Metric_K, Metric_K, No_Distortion, No_Distortion,
         First, Second, Policy);
   begin
      Assert (V'First = 1 and then V'Length = 1, "metric one-based result");
      return V (1);
   end Metric_Rectified;

   procedure Metric_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      V : constant Metric_Triangulated_Point := Metric_Rectified;
   begin
      Assert (V.Disposition = Metric_Accepted and then V.Source.Point.Status = Usable,
              "rectified metric accepted");
      Assert (Point_Gap (V.Geometry.World_Position, [4.0, 1.0, 8.0]) < 1.0E-12,
              "independent disparity oracle (4,1,8)");
      Assert (abs (V.Geometry.First_Depth - 8.0) < 1.0E-12 and then
              abs (V.Geometry.Second_Depth - 8.0) < 1.0E-12, "baseline-scaled depths");
   end Metric_Oracle;

   procedure Metric_General (T : in out Fixture) is
      pragma Unreferenced (T);
      --  Independent noncommuting Rz(0.3), Rx(-0.2), scalar Brown projections.
      A : constant World_To_Camera_Pose :=
        (Rotation => [0.0, 0.0, 0.3], Translation => [0.2, -0.1, 0.4]);
      B : constant World_To_Camera_Pose :=
        (Rotation => [-0.2, 0.0, 0.0], Translation => [-1.8, 0.3, 0.2]);
      KA : constant Camera_Intrinsics := (800.0, 820.0, 320.0, 240.0);
      KB : constant Camera_Intrinsics := (600.0, 640.0, 310.0, 220.0);
      DA : constant Distortion_Coefficients := (-0.12, 0.03, 0.001, -0.0015, 0.004);
      DB : constant Distortion_Coefficients := (0.08, -0.02, -0.002, 0.001, -0.003);
      Truth : constant Object_Point := [1.4, 0.6, 8.0];
      CA : constant Object_Point :=
        [Math.Cos (0.3) * Truth (0) - Math.Sin (0.3) * Truth (1) + 0.2,
         Math.Sin (0.3) * Truth (0) + Math.Cos (0.3) * Truth (1) - 0.1, 8.4];
      CB : constant Object_Point :=
        [Truth (0) - 1.8, Math.Cos (-0.2) * Truth (1) - Math.Sin (-0.2) * Truth (2) + 0.3,
         Math.Sin (-0.2) * Truth (1) + Math.Cos (-0.2) * Truth (2) + 0.2];
   begin
      for Model in 1 .. 2 loop
         declare
            D1 : constant Distortion_Coefficients := (if Model = 1 then No_Distortion else DA);
            D2 : constant Distortion_Coefficients := (if Model = 1 then No_Distortion else DB);
            P : constant Image_Point_Array := [1 => Scalar_Project (CA, KA, D1)];
            Q : constant Image_Point_Array := [1 => Scalar_Project (CB, KB, D2)];
            N1 : constant Image_Point_Array := Project_Points ([1 => Truth], KA, D1, A);
            N2 : constant Image_Point_Array := Project_Points ([1 => Truth], KB, D2, B);
            V : constant Metric_Triangulated_Point_Array := Triangulate_World_Points
              (P, Q, KA, KB, D1, D2, A, B, Metric_Policy);
            Reversed : constant Metric_Triangulated_Point_Array := Triangulate_World_Points
              (Q, P, KB, KA, D2, D1, B, A, Metric_Policy);
         begin
            for Axis in 0 .. 1 loop
               Assert (abs (P (1) (Axis) - N1 (1) (Axis)) < 1.0E-10 and then
                       abs (Q (1) (Axis) - N2 (1) (Axis)) < 1.0E-10, "analytic Brown/native match");
            end loop;
            Assert (V (1).Disposition = Metric_Accepted and then
                    Reversed (1).Disposition = Metric_Accepted, "general and reversed acceptance");
            Assert (Point_Gap (V (1).Geometry.World_Position, Truth) < 1.0E-9 and then
                    Point_Gap (Reversed (1).Geometry.World_Position, Truth) < 1.0E-9,
                    "independent world truth and camera reversal");
         end;
      end loop;
   end Metric_General;

   procedure Metric_Degenerate (T : in out Fixture) is
      pragma Unreferenced (T);
      Zero : constant Metric_Triangulated_Point := Metric_Rectified (Second => Metric_First);
      Infinity : constant Metric_Triangulated_Point :=
        Metric_Rectified (A => [0.0, 0.0], B => [0.0, 0.0]);
      Behind : constant Metric_Triangulated_Point :=
        Metric_Rectified (A => [-0.5, -0.125], B => [-0.25, -0.125]);
   begin
      Assert (Zero.Disposition = No_Physical_Baseline and then not Zero.Source.Available
              and then not Zero.Geometry.Available and then not Zero.Pixels.Available, "no zero-baseline point");
      Assert (Infinity.Disposition = Source_At_Infinity and then
              Infinity.Source.Point.Status = At_Infinity and then not Infinity.Geometry.Available,
              "genuine homogeneous infinity retained");
      Assert (Behind.Disposition = Source_Non_Positive_Depth and then
              Behind.Source.Point.Status = Non_Positive_Depth and then not Behind.Geometry.Available,
              "nonpositive source retained");
   end Metric_Degenerate;

   procedure Metric_Quality (T : in out Fixture) is
      pragma Unreferenced (T);
      Low : constant Metric_Triangulated_Point := Metric_Rectified
        (A => [0.5, 0.125], B => [0.499999, 0.125], Policy => (0.01, 1.0, 1.0, 1.0));
      Normalized : constant Metric_Triangulated_Point := Metric_Rectified
        (B => [0.25, 0.13], Policy => (0.0, 0.0, 1.0, 1.0));
      Pixel : constant Metric_Triangulated_Point := Metric_Rectified
        (B => [0.25, 0.13], Policy => (0.0, 1.0, 0.0, 0.0));
      Limits : constant Metric_Triangulation_Options :=
        (Pixel.Source.Quality.Parallax.Acute_Line_Angle_Radians,
         OpenCV.Float64_Value'Max (Pixel.Source.Point.First_Normalized_Error,
                                  Pixel.Source.Point.Second_Normalized_Error),
         Pixel.Pixels.First_Error_Pixels, Pixel.Pixels.Second_Error_Pixels);
      Inclusive : constant Metric_Triangulated_Point := Metric_Rectified
        (B => [0.25, 0.13], Policy => Limits);
   begin
      Assert (Low.Source.Point.Status = Usable and then Low.Disposition = Metric_Policy_Rejected
              and then not Low.Source.Quality.Passes_Parallax_Limit, "explicit acute parallax rejection");
      Assert (Normalized.Source.Point.Status = Usable and then
              Normalized.Disposition = Metric_Policy_Rejected and then
              not Normalized.Source.Quality.Passes_Reprojection_Limit, "normalized policy rejection");
      Assert (Pixel.Source.Quality.Accepted and then Pixel.Disposition = Metric_Policy_Rejected
              and then Pixel.Geometry.Available and then Pixel.Pixels.Available, "pixel units/policy distinct");
      Assert (Inclusive.Disposition = Metric_Accepted, "all four limits inclusive without epsilon");
   end Metric_Quality;

   procedure Metric_Range (T : in out Fixture) is
      pragma Unreferenced (T);
      P : World_To_Camera_Pose := Metric_Second;
      A : World_To_Camera_Pose := Metric_First;
   begin
      P.Translation := [-2.0E-300, 0.0, 0.0];
      declare
         V : constant Metric_Triangulated_Point := Metric_Rectified (Second => P);
      begin
         Assert (V.Disposition = Metric_Accepted and then
                 abs (V.Geometry.World_Position (2) / 1.0E-300 - 8.0) < 1.0E-12,
                 "tiny nonzero baseline not collapsed");
      end;
      P.Translation := [-2.0E300, 0.0, 0.0];
      declare
         V : constant Metric_Triangulated_Point := Metric_Rectified (Second => P);
      begin
         Assert (V.Disposition = Metric_Accepted and then
                 abs (V.Geometry.World_Position (2) / 1.0E300 - 8.0) < 1.0E-12, "large finite baseline");
      end;
      P.Translation := [-1.0E308, 0.0, 0.0];
      declare
         V : constant Metric_Triangulated_Point := Metric_Rectified (Second => P);
      begin
         Assert (V.Disposition = Metric_Conversion_Unavailable and then
                 V.Source.Point.Status = Usable and then not V.Geometry.Available, "metric scaling overflow");
      end;
      A.Translation := [-1.0E12, 1.0E12, 0.0];
      P := A;
      P.Translation (0) := P.Translation (0) - 2.0;
      declare
         V : constant Metric_Triangulated_Point := Metric_Rectified (First => A, Second => P);
      begin
         Assert (V.Disposition = Metric_Accepted and then
                 abs (V.Geometry.World_Position (0) - (1.0E12 + 4.0)) <= 0.0002,
                 "large world origin with representable small baseline");
      end;
   end Metric_Range;

   procedure Metric_Bounds (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant Image_Point_Array (Positive'Last - 1 .. Positive'Last) :=
        [[0.5, 0.125], [0.25, 0.0625]];
      B : constant Image_Point_Array (17 .. 18) := [[0.25, 0.125], [0.125, 0.0625]];
      V : constant Metric_Triangulated_Point_Array := Triangulate_World_Points
        (A, B, Metric_K, Metric_K, No_Distortion, No_Distortion,
         Metric_First, Metric_Second, Metric_Policy);
      Empty : constant Metric_Triangulated_Point_Array := Triangulate_World_Points
        (A (1 .. 0), B (1 .. 0), Metric_K, Metric_K, No_Distortion, No_Distortion,
         Metric_First, Metric_Second, Metric_Policy);
   begin
      Assert (V'First = 1 and then V'Length = 2 and then Empty'First = 1 and then Empty'Length = 0,
              "extreme lower bounds and empty");
      Assert (Point_Gap (V (1).Geometry.World_Position, [4.0, 1.0, 8.0]) < 1.0E-12 and then
              Point_Gap (V (2).Geometry.World_Position, [4.0, 1.0, 16.0]) < 1.0E-11,
              "positional order preserved at Positive'Last");
   end Metric_Bounds;

   procedure Metric_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check); --  Deliberate IEEE invalid input tests.
      A : Image_Point_Array := [1 => [0.5, 0.125]];
      B : Image_Point_Array := [1 => [0.25, 0.125]];
      KA : Camera_Intrinsics := Metric_K;
      D : Distortion_Coefficients := No_Distortion;
      P : World_To_Camera_Pose := Metric_First;
      O : Metric_Triangulation_Options := Metric_Policy;
      procedure Reject (Empty : Boolean := False; Mismatch : Boolean := False) is
      begin
         declare
            V : constant Metric_Triangulated_Point_Array := Triangulate_World_Points
              (A (1 .. (if Empty then 0 else 1)),
               B (1 .. (if Empty or Mismatch then 0 else 1)),
               KA, Metric_K, D, No_Distortion, P, Metric_Second, O);
         begin
            Assert (False, "invalid metric request returned" & V'Length'Image);
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Reject;
   begin
      Reject (Mismatch => True);
      for Kind in Interfaces.Integer_32 range 0 .. 2 loop
         declare
            Bad : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Nonfinite (Kind));
         begin
            A (1) (0) := Bad;
            Reject;
            A (1) (0) := 0.5;
            B (1) (1) := Bad;
            Reject;
            B (1) (1) := 0.125;
            KA.Focal_X := Bad;
            Reject (Empty => True);
            KA := Metric_K;
            D.K3 := Bad;
            Reject (Empty => True);
            D := No_Distortion;
            P.Translation (1) := Bad;
            Reject (Empty => True);
            P := Metric_First;
            for Field in 1 .. 4 loop
               O := Metric_Policy;
               case Field is
                  when 1 => O.Minimum_Acute_Parallax_Radians := Bad;
                  when 2 => O.Maximum_Normalized_Reprojection_Error := Bad;
                  when 3 => O.Maximum_First_Pixel_Reprojection_Error := Bad;
                  when 4 => O.Maximum_Second_Pixel_Reprojection_Error := Bad;
               end case;
               Reject (Empty => True);
            end loop;
            O := Metric_Policy;
         end;
      end loop;
      KA.Focal_Y := 0.0;
      Reject;
      KA := Metric_K;
      for Field in 1 .. 5 loop
         O := Metric_Policy;
         case Field is
            when 1 => O.Minimum_Acute_Parallax_Radians := -1.0;
            when 2 => O.Maximum_Normalized_Reprojection_Error := -1.0;
            when 3 => O.Maximum_First_Pixel_Reprojection_Error := -1.0;
            when 4 => O.Maximum_Second_Pixel_Reprojection_Error := -1.0;
            when 5 => O.Minimum_Acute_Parallax_Radians := 2.0;
         end case;
         Reject;
      end loop;
   end Metric_Invalid;

   function Metric_Refined
     (Policy : Metric_Triangulation_Options := (0.0, 1.0, 1.0, 1.0);
      Allowance : Metric_Refinement_Acceptance_Options := (1.0, 1.0);
      Options : Triangulation_Refinement_Options := (20, 0.0);
      A : Image_Point := [0.5, 0.126]; B : Image_Point := [0.25, 0.124];
      First : World_To_Camera_Pose := Metric_First;
      Second : World_To_Camera_Pose := Metric_Second)
      return Metric_Point_Refinement_Result
   is
      V : constant Metric_Point_Refinement_Result_Array := Refine_Metric_World_Points
        ([1 => A], [1 => B], Metric_K, Metric_K, No_Distortion, No_Distortion,
         First, Second, Policy, Options, Allowance);
   begin
      return V (1);
   end Metric_Refined;

   procedure Metric_Refinement_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      V : constant Metric_Point_Refinement_Result := Metric_Refined;
      O : constant Metric_Triangulated_Point := Original_Reconstruction (V);
      R : constant Optional_Point_Refinement := Normalized_Refinement (V);
      C : constant Metric_Triangulated_Point := Refined_Diagnostics (V);
      Expected : constant Metric_Triangulated_Point := Metric_Rectified
        (A => [0.5, 0.126], B => [0.25, 0.124], Policy => (0.0, 1.0, 1.0, 1.0));
   begin
      Assert (Accepted (V) and then Disposition (V) = Refined_Metric_Accepted, "metric refinement accepted");
      Assert (R.Available and then R.Value.Outcome = Improved and then R.Value.Accepted_Steps > 0
              and then R.Value.Residuals_Evaluated and then
              R.Value.Final_Normalized_RMS < R.Value.Initial_Normalized_RMS, "strict normalized improvement");
      Assert (O = Expected, "original full reconstruction preserved exactly");
      --  Rectified equal-x fit: X/Z=.5, (X-2)/Z=.25, Y/Z=mean(.126,.124).
      Assert (Point_Gap (Accepted_World_Position (V), [4.0, 1.0, 8.0]) < 1.0E-8,
              "independent noisy rectified least-squares world oracle");
      Assert (abs (C.Geometry.First_Depth - 8.0) < 1.0E-8 and then
              abs (C.Source.Point.First_Normalized_Error - 0.001) < 1.0E-10 and then
              abs (C.Source.Point.Second_Normalized_Error - 0.001) < 1.0E-10,
              "fresh normalized errors and metric depths");
   end Metric_Refinement_Oracle;

   procedure Metric_Refinement_Acceptance (T : in out Fixture) is
      pragma Unreferenced (T);
      V : constant Metric_Point_Refinement_Result := Metric_Refined;
      O : constant Metric_Triangulated_Point := Original_Reconstruction (V);
      C : constant Metric_Triangulated_Point := Refined_Diagnostics (V);
      Worsening : constant OpenCV.Float64_Value := C.Pixels.First_Error_Pixels - O.Pixels.First_Error_Pixels;
      Strict : constant Metric_Point_Refinement_Result := Metric_Refined (Allowance => (0.0, 0.0));
      Equal : constant Metric_Point_Refinement_Result := Metric_Refined (Allowance => (Worsening, 1.0));
      Below : constant Metric_Point_Refinement_Result := Metric_Refined
        (Allowance => (OpenCV.Float64_Value'Adjacent (Worsening, 0.0), 1.0));
      Rescue : constant Metric_Point_Refinement_Result := Metric_Refined
        (Policy => (0.0, 1.0, 1.0, (O.Pixels.Second_Error_Pixels + C.Pixels.Second_Error_Pixels) / 2.0));
      Rejected : constant Metric_Point_Refinement_Result := Metric_Refined (Policy => (0.0, 0.0, 1.0, 1.0));
   begin
      Assert (Worsening > 0.0 and then not Accepted (Strict) and then
              Disposition (Strict) = Refined_Policy_Rejected, "normalized improvement does not imply pixel improvement");
      Assert (Accepted (Equal), "direct finite subtraction, allowance equality inclusive");
      Assert (not Accepted (Below), "one Float64 step below worsening boundary rejected without epsilon");
      Assert (Original_Reconstruction (Rescue).Disposition = Metric_Policy_Rejected and then
              Accepted (Rescue), "initial policy rejection can be rescued only by final full policy");
      Assert (not Accepted (Rejected), "normalized final policy enforced");
      declare
         Inclusive : constant Metric_Point_Refinement_Result := Metric_Refined
           (Policy => (C.Source.Quality.Parallax.Acute_Line_Angle_Radians,
             OpenCV.Float64_Value'Max (C.Source.Point.First_Normalized_Error, C.Source.Point.Second_Normalized_Error),
             C.Pixels.First_Error_Pixels, C.Pixels.Second_Error_Pixels));
      begin
         Assert (Accepted (Inclusive), "all four final metric threshold equalities inclusive");
      end;
      begin
         declare
            P : constant Object_Point := Accepted_World_Position (Strict);
            pragma Unreferenced (P);
         begin
            Assert (False, "unaccepted candidate position published as accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
   end Metric_Refinement_Acceptance;

   procedure Metric_Refinement_Skips (T : in out Fixture) is
      pragma Unreferenced (T);
      Zero : constant Metric_Point_Refinement_Result := Metric_Refined (Second => Metric_First);
      Infinity : constant Metric_Point_Refinement_Result := Metric_Refined (A => [0.0, 0.0], B => [0.0, 0.0]);
      Behind : constant Metric_Point_Refinement_Result := Metric_Refined (A => [-0.5, -0.125], B => [-0.25, -0.125]);
      Low : constant Metric_Point_Refinement_Result := Metric_Refined (Options => (20, 1.0));
      Exact : constant Metric_Point_Refinement_Result := Metric_Refined (A => [0.0, 0.0], B => [-0.5, 0.0]);
   begin
      Assert (Disposition (Zero) = Initial_No_Baseline and then
              not Normalized_Refinement (Zero).Available, "zero baseline explicit absent optimization");
      Assert (Disposition (Infinity) = Initial_Source_Unusable and then
              Disposition (Behind) = Initial_Source_Unusable, "non-Usable source skipped");
      Assert (Disposition (Low) = Refinement_Low_Parallax and then
              Normalized_Refinement (Low).Value.Outcome = Skipped_Low_Parallax, "low acute parallax skipped");
      Assert ((Disposition (Exact) = Refinement_No_Improving_Step or else Accepted (Exact)) and then
              Point_Gap (Original_Reconstruction (Exact).Geometry.World_Position, [0.0, 0.0, 4.0]) < 1.0E-12,
              "exact scene preserves initial despite version-dependent SVD rounding");
   end Metric_Refinement_Skips;

   procedure Metric_Refinement_General (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant World_To_Camera_Pose := ([0.0, 0.0, 0.3], [0.2, -0.1, 0.4]);
      B : constant World_To_Camera_Pose := ([-0.2, 0.0, 0.0], [-1.8, 0.3, 0.2]);
      KA : constant Camera_Intrinsics := (800.0, 820.0, 320.0, 240.0);
      KB : constant Camera_Intrinsics := (600.0, 640.0, 310.0, 220.0);
      DA : constant Distortion_Coefficients := (-0.12, 0.03, 0.001, -0.0015, 0.004);
      DB : constant Distortion_Coefficients := (0.08, -0.02, -0.002, 0.001, -0.003);
      Truth : constant Object_Point := [1.4, 0.6, 8.0];
      CA : constant Object_Point :=
        [Math.Cos (0.3) * 1.4 - Math.Sin (0.3) * 0.6 + 0.2,
         Math.Sin (0.3) * 1.4 + Math.Cos (0.3) * 0.6 - 0.1, 8.4];
      CB : constant Object_Point :=
        [-0.4, Math.Cos (-0.2) * 0.6 - Math.Sin (-0.2) * 8.0 + 0.3,
         Math.Sin (-0.2) * 0.6 + Math.Cos (-0.2) * 8.0 + 0.2];
   begin
      for Model in 1 .. 2 loop
         declare
            D1 : constant Distortion_Coefficients := (if Model = 1 then No_Distortion else DA);
            D2 : constant Distortion_Coefficients := (if Model = 1 then No_Distortion else DB);
            P : Image_Point_Array := [1 => Scalar_Project (CA, KA, D1)];
            Q : Image_Point_Array := [1 => Scalar_Project (CB, KB, D2)];
         begin
            P (1) (1) := P (1) (1) + 0.2;
            Q (1) (0) := Q (1) (0) - 0.15;
            declare
               V : constant Metric_Point_Refinement_Result_Array := Refine_Metric_World_Points
                 (P, Q, KA, KB, D1, D2, A, B, (0.01, 0.01, 5.0, 5.0), (30, 0.0), (5.0, 5.0));
               W : constant Metric_Point_Refinement_Result_Array := Refine_Metric_World_Points
                 (Q, P, KB, KA, D2, D1, B, A, (0.01, 0.01, 5.0, 5.0), (30, 0.0), (5.0, 5.0));
            begin
               Assert (Accepted (V (1)) and then Accepted (W (1)), "noncommuting reversed Brown refinement accepted");
               Assert (Point_Gap (Accepted_World_Position (V (1)), Truth) < 0.02 and then
                       Point_Gap (Accepted_World_Position (W (1)), Truth) < 0.02 and then
                       Point_Gap (Accepted_World_Position (V (1)), Accepted_World_Position (W (1))) < 1.0E-7,
                       "independent noisy geometry and reversal tolerance");
            end;
         end;
      end loop;
   end Metric_Refinement_General;

   procedure Metric_Refinement_Range (T : in out Fixture) is
      pragma Unreferenced (T);
      B : World_To_Camera_Pose := Metric_Second;
      A : World_To_Camera_Pose := Metric_First;
   begin
      for Scale of Object_Point'[1.0E-300, 1.0E300, 1.0] loop
         B.Translation := [-2.0 * Scale, 0.0, 0.0];
         declare
            V : constant Metric_Point_Refinement_Result := Metric_Refined (Second => B);
         begin
            Assert (Accepted (V), "nonzero tiny/large baseline retained");
            Assert (abs (Accepted_World_Position (V) (2) / Scale - 8.0) < 1.0E-8, "scaled analytic world depth");
         end;
      end loop;
      A.Translation := [-1.0E12, 0.0, 0.0];
      B.Translation := [-1.0E12 - 2.0, 0.0, 0.0];
      declare
         V : constant Metric_Point_Refinement_Result := Metric_Refined (First => A, Second => B);
      begin
         Assert (Original_Reconstruction (V).Geometry.Available, "large origin initial remains owned");
         if Accepted (V) then
            Assert (abs (Accepted_World_Position (V) (0) - (1.0E12 + 4.0)) < 0.001,
                    "representable small offset at large origin");
         end if;
      end;
      B := Metric_Second;
      B.Translation (0) := -1.0E308;
      declare
         V : constant Metric_Point_Refinement_Result := Metric_Refined (Second => B);
      begin
         Assert (Disposition (V) = Initial_Geometry_Unavailable and then
                 not Normalized_Refinement (V).Available, "initial overflow cannot masquerade as refinable geometry");
      end;
   end Metric_Refinement_Range;

   procedure Metric_Refinement_Bounds (T : in out Fixture) is
      pragma Unreferenced (T);
      P : constant Image_Point_Array (Positive'Last - 2 .. Positive'Last) :=
        [[0.5, 0.126], [0.0, 0.0], [0.0, 0.0]];
      Q : constant Image_Point_Array (7 .. 9) := [[0.25, 0.124], [0.0, 0.0], [-0.5, 0.0]];
      V : constant Metric_Point_Refinement_Result_Array := Refine_Metric_World_Points
        (P, Q, Metric_K, Metric_K, No_Distortion, No_Distortion, Metric_First, Metric_Second,
         (0.0, 1.0, 1.0, 1.0), (20, 0.0), (0.0, 0.0));
      E : constant Metric_Point_Refinement_Result_Array := Refine_Metric_World_Points
        (P (1 .. 0), Q (1 .. 0), Metric_K, Metric_K, No_Distortion, No_Distortion,
         Metric_First, Metric_Second, Metric_Policy, (20, 0.0), (0.0, 0.0));
   begin
      Assert (V'First = 1 and then V'Last = 3 and then E'First = 1 and then E'Last = 0,
              "positional bounds and valid empty");
      Assert (Disposition (V (1)) = Refined_Policy_Rejected and then
              Disposition (V (2)) = Initial_Source_Unusable and then
              Original_Reconstruction (V (3)).Geometry.Available and then
              Point_Gap (Original_Reconstruction (V (3)).Geometry.World_Position, [0.0, 0.0, 4.0]) < 1.0E-12,
              "mixed rejected unavailable retained batch order: " & Disposition (V (1))'Image &
              Disposition (V (2))'Image & Disposition (V (3))'Image);
   end Metric_Refinement_Bounds;

   procedure Metric_Refinement_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check);
      function Nonfinite (Kind : Interfaces.C.int) return Interfaces.C.double
        with Import, Convention => C, External_Name => "calib3d_test_nonfinite";
      K : Camera_Intrinsics := Metric_K;
      D : Distortion_Coefficients := No_Distortion;
      Pose : World_To_Camera_Pose := Metric_First;
      Policy : Metric_Triangulation_Options := Metric_Policy;
      Options : Triangulation_Refinement_Options := (20, 0.0);
      Allowance : Metric_Refinement_Acceptance_Options := (0.0, 0.0);
      P : Image_Point := [0.5, 0.126];
      procedure Reject (Empty : Boolean := True; Mismatch : Boolean := False) is
      begin
         declare
            V : constant Metric_Point_Refinement_Result_Array := Refine_Metric_World_Points
              ((if Empty then Image_Point_Array'[1 .. 0 => <>] else [1 => P]),
               (if Empty or Mismatch then Image_Point_Array'[1 .. 0 => <>] else [1 => [0.25, 0.124]]),
               K, Metric_K, D, No_Distortion, Pose, Metric_Second, Policy, Options, Allowance);
            pragma Unreferenced (V);
         begin
            Assert (False, "invalid metric refinement input accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Reject;
   begin
      for Kind in Interfaces.C.int range 0 .. 2 loop
         declare
            Bad : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Nonfinite (Kind));
         begin
            Allowance.Maximum_First_Pixel_Worsening := Bad; Reject;
            Allowance := (0.0, Bad); Reject;
            Allowance := (0.0, 0.0);
            Options.Minimum_Acute_Parallax_Radians := Bad; Reject;
            Options := (20, 0.0);
            K.Focal_X := Bad; Reject; K := Metric_K;
            D.K3 := Bad; Reject; D := No_Distortion;
            Pose.Rotation (2) := Bad; Reject; Pose := Metric_First;
            P (0) := Bad; Reject (False); P := [0.5, 0.126];
            for Field in 1 .. 4 loop
               case Field is
                  when 1 => Policy.Minimum_Acute_Parallax_Radians := Bad;
                  when 2 => Policy.Maximum_Normalized_Reprojection_Error := Bad;
                  when 3 => Policy.Maximum_First_Pixel_Reprojection_Error := Bad;
                  when 4 => Policy.Maximum_Second_Pixel_Reprojection_Error := Bad;
               end case;
               Reject; Policy := Metric_Policy;
            end loop;
         end;
      end loop;
      Allowance := (-1.0, 0.0); Reject; Allowance := (0.0, -1.0); Reject;
      Allowance := (0.0, 0.0);
      Options.Maximum_Iterations := 101; Reject; Options := (20, 2.0); Reject;
      Options := (20, -1.0); Reject; Options := (20, 0.0);
      K.Focal_Y := 0.0; Reject; K := Metric_K;
      Reject (False, True);
   end Metric_Refinement_Invalid;

   --  Task 021 oracles deliberately do not use production transforms or
   --  Project_Points to generate camera truth. R=Rz(.3), all five Brown terms.
   Reloc_Truth : constant World_To_Camera_Pose := ([0.0, 0.0, 0.3], [0.2, -0.1, 6.0]);
   Reloc_D : constant Distortion_Coefficients := (0.02, -0.003, 0.001, -0.002, 0.0004);
   Reloc_RANSAC : constant RANSAC_Options := (1000, 1.0, 0.999);
   Reloc_Policy : constant Relocalization_Verification_Options := (12, 0.5, 0.5, 0.8, 0.5);
   Reloc_H : constant Object_Point_Array :=
     [[-1.3, 0.4, 3.0], [1.1, -0.7, 2.9], [0.2, 1.2, 3.2], [-0.5, -1.1, 3.4]];

   function Reloc_Project (Points : Object_Point_Array;
                           Distortion : Distortion_Coefficients := Reloc_D) return Image_Point_Array is
      C : constant OpenCV.Float64_Value := Math.Cos (0.3);
      S : constant OpenCV.Float64_Value := Math.Sin (0.3);
   begin
      return Pixels : Image_Point_Array (1 .. Points'Length) do
         for I in 1 .. Points'Length loop
            declare
               P : constant Object_Point := Points (Points'First + (I - 1));
               X : constant OpenCV.Float64_Value := (C * P (0) - S * P (1) + 0.2) / (P (2) + 6.0);
               Y : constant OpenCV.Float64_Value := (S * P (0) + C * P (1) - 0.1) / (P (2) + 6.0);
               R2 : constant OpenCV.Float64_Value := X * X + Y * Y;
               Radial : constant OpenCV.Float64_Value :=
                 1.0 + Distortion.K1 * R2 + Distortion.K2 * R2 ** 2 + Distortion.K3 * R2 ** 3;
            begin
               Pixels (I) :=
                 [K.Focal_X * (X * Radial + 2.0 * Distortion.P1 * X * Y +
                    Distortion.P2 * (R2 + 2.0 * X * X)) + K.Center_X,
                  K.Focal_Y * (Y * Radial + Distortion.P1 * (R2 + 2.0 * Y * Y) +
                    2.0 * Distortion.P2 * X * Y) + K.Center_Y];
            end;
         end loop;
      end return;
   end Reloc_Project;

   function Reloc_Field return Object_Point_Array is
   begin
      return Points : Object_Point_Array (1 .. 30) do
         for I in Points'Range loop
            Points (I) := [OpenCV.Float64_Value (I mod 6) * 0.5 - 1.3,
                          OpenCV.Float64_Value (I / 6) * 0.4 - 0.9,
                          OpenCV.Float64_Value (I mod 7) * 0.23];
         end loop;
      end return;
   end Reloc_Field;

   function Reloc_Noisy return Image_Point_Array is
      Pixels : Image_Point_Array := Reloc_Project (Reloc_Field);
   begin
      for I in Pixels'Range loop
         Pixels (I) (0) := Pixels (I) (0) + OpenCV.Float64_Value (I mod 3 - 1) * 0.08;
         Pixels (I) (1) := Pixels (I) (1) + OpenCV.Float64_Value (I mod 5 - 2) * 0.04;
      end loop;
      return Pixels;
   end Reloc_Noisy;

   procedure Reloc_Guards (R : Camera_Relocalization_Result) is
      P : World_To_Camera_Pose;
      C : Object_Point;
      Summary : Reprojection_Summary;
      Errors : Reprojection_Error_Array (1 .. R.Count);
      pragma Unreferenced (P, C, Summary, Errors);
   begin
      Assert (not Accepted (R), "guard fixture must not be accepted");
      begin
         P := Accepted_Pose (R); Assert (False, "rejected pose exposed");
      exception when OpenCV.OpenCV_Error => null; end;
      begin
         C := Camera_Center (R); Assert (False, "rejected center exposed");
      exception when OpenCV.OpenCV_Error => null; end;
      if not Found (R) then
         begin
            P := Initial_Pose (R); Assert (False, "absent native pose exposed");
         exception when OpenCV.OpenCV_Error => null; end;
      end if;
      if not Initial_Holdout_Valid (R) then
         begin
            Summary := Initial_Holdout (R); Assert (False, "absent initial summary exposed");
         exception when OpenCV.OpenCV_Error => null; end;
      end if;
      if not Refined_Holdout_Valid (R) then
         begin
            Summary := Refined_Holdout (R); Assert (False, "absent refined summary exposed");
         exception when OpenCV.OpenCV_Error => null; end;
      end if;
      if not Final_Inliers_Valid (R) then
         begin
            Summary := Final_Inlier_Summary (R); Assert (False, "absent final summary exposed");
         exception when OpenCV.OpenCV_Error => null; end;
         begin
            Errors := Final_Inlier_Errors (R); Assert (False, "absent final errors exposed");
         exception when OpenCV.OpenCV_Error => null; end;
      end if;
   end Reloc_Guards;

   procedure Reloc_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      W : constant Object_Point_Array (Positive'Last - 29 .. Positive'Last) := Reloc_Field;
      Pixels : Image_Point_Array (7 .. 36) := Reloc_Project (W);
      H : constant Image_Point_Array := Reloc_Project (Reloc_H);
   begin
      for Noisy in Boolean loop
         if Noisy then
            Pixels := Reloc_Noisy;
         end if;
         --  Gross deterministic mismatches, never used in refinement.
         for Position in 1 .. W'Length loop
            if Position mod 7 = 0 then
               Pixels (Pixels'First + (Position - 1)) := [2000.0, -1500.0];
            end if;
         end loop;
         declare
            R : constant Camera_Relocalization_Result := Relocalize_Camera
              (W, Pixels, Reloc_H, H, K, Reloc_D, Reloc_RANSAC, Reloc_Policy);
            Indices : constant Inlier_Index_Array := Inliers (R);
            IW : Object_Point_Array (1 .. R.Count);
            IP : Image_Point_Array (1 .. R.Count);
            Manual : World_To_Camera_Pose := Initial_Pose (R);
            OK : Boolean := False;
         begin
            Assert (Accepted (R) and then Refinement_Attempted (R) and then
                    Native_Refinement_Succeeded (R), "robust exact/noisy accepted");
            Assert (Indices'First = 1 and then R.Count = Inlier_Count (R) and then
                    R.Count >= Reloc_Policy.Minimum_Consensus_Inliers and then R.Count < 30,
                    "actual native consensus, no version-specific count");
            for I in Indices'Range loop
               Assert (Indices (I) <= 30 and then Indices (I) mod 7 /= 0 and then
                       (I = 1 or else Indices (I) > Indices (I - 1)), "original sorted inlier positions");
               IW (I) := W (W'First + (Indices (I) - 1));
               IP (I) := Pixels (Pixels'First + (Indices (I) - 1));
            end loop;
            Refine_Pose_Iterative (IW, IP, K, Reloc_D, Manual, OK);
            Assert (OK, "manual inlier-only native refinement");
            declare
               A : constant Rotation_Matrix := Rotation_Matrix_Of (Accepted_Pose (R));
               B : constant Rotation_Matrix := Rotation_Matrix_Of (Manual);
               Errors : constant Reprojection_Error_Array := Reprojection_Errors
                 (IW, IP, K, Reloc_D, Manual);
               Center : constant Object_Point := Camera_Center (R);
               C : constant OpenCV.Float64_Value := Math.Cos (0.3);
               S : constant OpenCV.Float64_Value := Math.Sin (0.3);
            begin
               Assert (Near (A (0, 0), C, 0.002) and then Near (A (0, 1), -S, 0.002)
                 and then Near (A (1, 0), S, 0.002) and then Near (A (2, 2), 1.0, 0.002),
                 "independent analytic Rz matrix, not Rodrigues components");
               for Axis in 0 .. 2 loop
                  Assert (Near (Accepted_Pose (R).Translation (Axis), Reloc_Truth.Translation (Axis), 0.003),
                          "known nonzero t");
                  for Column in 0 .. 2 loop
                     Assert (Near (A (Axis, Column), B (Axis, Column), 1.0E-12), "exact inlier-only replay");
                  end loop;
               end loop;
               Assert (Near (Center (0), -C * 0.2 + S * 0.1, 0.02) and then
                       Near (Center (1), S * 0.2 + C * 0.1, 0.02) and then
                       Near (Center (2), -6.0, 0.02), "analytic -R^T t center");
               for I in Errors'Range loop
                  Assert (Near (Final_Inlier_Errors (R) (I), Errors (I), 1.0E-12), "inlier diagnostics replay");
               end loop;
               Assert (Final_Inlier_Summary (R).Count = R.Count and then
                       Initial_Holdout (R).Count = Reloc_H'Length and then
                       Refined_Holdout (R).Count = Reloc_H'Length, "whole-set scores");
            end;
         end;
      end loop;
      declare
         R : constant Camera_Relocalization_Result := Relocalize_Camera
           (W, Reloc_Project (W, No_Distortion), Reloc_H, Reloc_Project (Reloc_H, No_Distortion),
            K, No_Distortion, Reloc_RANSAC, Reloc_Policy);
      begin
         Assert (Accepted (R), "zero Brown also qualified");
      end;
   end Reloc_Oracle;

   procedure Reloc_Boundary (T : in out Fixture) is
      pragma Unreferenced (T);
      W : constant Object_Point_Array := Reloc_Field;
      P : constant Image_Point_Array := Reloc_Project (W);
   begin
      for N in 0 .. 5 loop
         declare
            R : constant Camera_Relocalization_Result := Relocalize_Camera
              (W (1 .. N), P (1 .. N), Reloc_H, Reloc_Project (Reloc_H),
               K, Reloc_D, Reloc_RANSAC, Reloc_Policy);
         begin
            Assert (Disposition (R) = Insufficient_Fitting_Landmarks and then not Found (R)
              and then Inlier_Count (R) = 0 and then not Refinement_Attempted (R), "0..5 no direct solve");
            Reloc_Guards (R);
         end;
      end loop;
      declare
         Policy : Relocalization_Verification_Options := Reloc_Policy;
         Six : constant Object_Point_Array := [W (1), W (7), W (13), W (20), W (25), W (30)];
      begin
         Policy.Minimum_Consensus_Inliers := 4;
         declare
            R : constant Camera_Relocalization_Result := Relocalize_Camera
              (Six, Reloc_Project (Six), Reloc_H, Reloc_Project (Reloc_H),
               K, Reloc_D, Reloc_RANSAC, Policy);
         begin
            Assert (Accepted (R), "six correspondences enter robust path");
         end;
      end;
      declare
         Policy : Relocalization_Verification_Options := Reloc_Policy;
      begin
         Policy.Minimum_Consensus_Inliers := 31;
         declare
            R : constant Camera_Relocalization_Result := Relocalize_Camera
              (W, P, Reloc_H, Reloc_Project (Reloc_H), K, Reloc_D, Reloc_RANSAC, Policy);
         begin
            Assert (Disposition (R) = Insufficient_Consensus_Inliers and then Found (R)
              and then not Refinement_Attempted (R), "caller minimum cannot be accepted");
            Reloc_Guards (R);
         end;
      end;
      declare
         Bad : Image_Point_Array (1 .. 30);
         Options : RANSAC_Options := Reloc_RANSAC;
      begin
         for I in Bad'Range loop
            Bad (I) := [OpenCV.Float64_Value (I * I * 17 mod 701),
                        OpenCV.Float64_Value (I * I * I * 13 mod 503)];
         end loop;
         Options.Reprojection_Error_Pixels := 1.0E-8;
         declare
            R : constant Camera_Relocalization_Result := Relocalize_Camera
              (W, Bad, Reloc_H, Reloc_Project (Reloc_H), K, Reloc_D, Options, Reloc_Policy);
         begin
            Assert (Disposition (R) = No_Native_RANSAC_Pose and then not Found (R), "inconsistent no consensus fixture");
            Reloc_Guards (R);
         end;
      end;
   end Reloc_Boundary;

   procedure Reloc_Policies (T : in out Fixture) is
      pragma Unreferenced (T);
      W : constant Object_Point_Array := Reloc_Field;
      P : constant Image_Point_Array := Reloc_Noisy;
      H : Image_Point_Array := Reloc_Project (Reloc_H);
      R : constant Camera_Relocalization_Result := Relocalize_Camera
        (W, P, Reloc_H, H, K, Reloc_D, Reloc_RANSAC, Reloc_Policy);
      Policy : Relocalization_Verification_Options := Reloc_Policy;
      procedure Check (Expected : Boolean) is
         V : constant Camera_Relocalization_Result := Relocalize_Camera
           (W, P, Reloc_H, H, K, Reloc_D, Reloc_RANSAC, Policy);
      begin
         Assert (Accepted (V) = Expected, "inclusive numerical policy boundary");
         if not Expected then
            Assert (Disposition (V) = Rejected_By_Policy and then Native_Refinement_Succeeded (V)
              and then Found (V) and then Initial_Holdout_Valid (V) and then Refined_Holdout_Valid (V)
              and then Final_Inliers_Valid (V), "native true is not acceptance; scores retained");
            Reloc_Guards (V);
         end if;
      end Check;
   begin
      Assert (Accepted (R), "policy fixture");
      Policy.Maximum_Final_Inlier_RMS_Pixels := Final_Inlier_Summary (R).RMS_Error_Pixels;
      Check (True);
      Policy.Maximum_Final_Inlier_RMS_Pixels := OpenCV.Float64_Value'Adjacent
        (Policy.Maximum_Final_Inlier_RMS_Pixels, 0.0); Check (False);
      Policy := Reloc_Policy;
      Policy.Maximum_Holdout_RMS_Pixels := Refined_Holdout (R).RMS_Error_Pixels; Check (True);
      Policy.Maximum_Holdout_RMS_Pixels := OpenCV.Float64_Value'Adjacent
        (Policy.Maximum_Holdout_RMS_Pixels, 0.0); Check (False);
      Policy := Reloc_Policy;
      Policy.Maximum_Holdout_Point_Error_Pixels := Refined_Holdout (R).Maximum_Error_Pixels; Check (True);
      Policy.Maximum_Holdout_Point_Error_Pixels := OpenCV.Float64_Value'Adjacent
        (Policy.Maximum_Holdout_Point_Error_Pixels, 0.0); Check (False);
      --  Comparator fixture, NOT an independent truth oracle: observations at
      --  the initial native pose make its RMS zero and expose positive worsening.
      H := Project_Points (Reloc_H, K, Reloc_D, Initial_Pose (R));
      Policy := Reloc_Policy;
      declare
         V : constant Camera_Relocalization_Result := Relocalize_Camera
           (W, P, Reloc_H, H, K, Reloc_D, Reloc_RANSAC, Policy);
         Worsening : constant OpenCV.Float64_Value :=
           Refined_Holdout (V).RMS_Error_Pixels - Initial_Holdout (V).RMS_Error_Pixels;
      begin
         Assert (Worsening > 0.0, "native refinement genuinely worsens heldout fixture");
         Policy.Maximum_Holdout_RMS_Worsening_Pixels := Worsening; Check (True);
         Policy.Maximum_Holdout_RMS_Worsening_Pixels := OpenCV.Float64_Value'Adjacent (Worsening, 0.0); Check (False);
         Policy.Maximum_Holdout_RMS_Worsening_Pixels := 0.0; Check (False);
      end;
   end Reloc_Policies;

   procedure Reloc_Unusable (T : in out Fixture) is
      pragma Unreferenced (T);
      W : Object_Point_Array := Reloc_Field;
      H : Object_Point_Array (Positive'Last .. Positive'Last) := [others => [0.0, 0.0, -20.0]];
      P : constant Image_Point_Array := Reloc_Project (W);
   begin
      for Huge in Boolean loop
         if Huge then
            H (H'First) := [OpenCV.Float64_Value'Last, -OpenCV.Float64_Value'Last, 1.0];
         end if;
         declare
            R : constant Camera_Relocalization_Result := Relocalize_Camera
              (W, P, H, [Positive'Last => [0.0, 0.0]], K, Reloc_D, Reloc_RANSAC, Reloc_Policy);
         begin
            Assert (Disposition (R) = Initial_Holdout_Unusable and then Found (R) and then
              not Refinement_Attempted (R) and then not Initial_Holdout_Valid (R), "behind/overflow whole set unusable");
            Reloc_Guards (R);
         end;
      end loop;
      --  PnP does not promise positive depth for all native consensus points.
      --  A mixed front/back field tests the independent final depth screen.
      for I in 1 .. 4 loop
         W (I) (2) := W (I) (2) - 15.0;
      end loop;
      declare
         Estimate : constant Pose_Estimate := Solve_PnP_RANSAC
           (W, Reloc_Project (W), K, Reloc_D, Reloc_RANSAC);
         --  This diagnostic fixture places the held-out point in front of the
         --  actual native initial camera, without assuming which mixed-depth
         --  EPNP orientation a version returns. It is not a truth oracle.
         HW : constant Object_Point_Array :=
           [1 => Camera_To_World_Point (Pose => Pose (Estimate), Point => [0.0, 0.0, 10.0])];
         HP : constant Image_Point_Array := Project_Points (HW, K, Reloc_D, Pose (Estimate));
         R : constant Camera_Relocalization_Result := Relocalize_Camera
           (W, Reloc_Project (W), HW, HP, K, Reloc_D, Reloc_RANSAC, Reloc_Policy);
      begin
         Ada.Text_IO.Put_Line ("mixed-depth relocalization: " & Disposition (R)'Image & Inlier_Count (R)'Image);
         Assert (Disposition (R) = Final_Inlier_Verification_Unusable and then
           Native_Refinement_Succeeded (R) and then Refined_Holdout_Valid (R), "final native consensus depth checked");
         Reloc_Guards (R);
      end;
   end Reloc_Unusable;

   procedure Reloc_Refined_Depth (T : in out Fixture) is
      pragma Unreferenced (T);
      W : constant Object_Point_Array := Reloc_Field;
      P : constant Image_Point_Array := Reloc_Noisy;
      Base : constant Camera_Relocalization_Result := Relocalize_Camera
        (W, P, Reloc_H, Reloc_Project (Reloc_H), K, Reloc_D, Reloc_RANSAC, Reloc_Policy);
      Initial : constant World_To_Camera_Pose := Initial_Pose (Base);
      Candidate : constant World_To_Camera_Pose := Accepted_Pose (Base);
      Exercised : Boolean := False;
   begin
      --  Diagnostic boundary construction, not an independent truth oracle.
      --  Find a point just inside the initial camera's front half-space but
      --  outside the refined camera's. Native pose is never altered by a hook.
      for X of Reprojection_Error_Array'[-10000.0, 0.0, 10000.0] loop
         for Y of Reprojection_Error_Array'[-10000.0, 0.0, 10000.0] loop
            declare
               At_Zero : constant Object_Point := World_To_Camera_Point
                 (Candidate, Camera_To_World_Point (Initial, [X, Y, 0.0]));
               At_One : constant Object_Point := World_To_Camera_Point
                 (Candidate, Camera_To_World_Point (Initial, [X, Y, 1.0]));
               Slope : constant OpenCV.Float64_Value := At_One (2) - At_Zero (2);
            begin
               if not Exercised and then At_Zero (2) < 0.0 and then Slope > 0.0 then
                  declare
                     H : constant Object_Point_Array := [1 => Camera_To_World_Point
                       (Initial, [X, Y, -At_Zero (2) / Slope / 2.0])];
                     V : constant Camera_Relocalization_Result := Relocalize_Camera
                       (W, P, H, [1 => [320.0, 240.0]], K, Reloc_D, Reloc_RANSAC, Reloc_Policy);
                  begin
                     Assert (Disposition (V) = Refined_Holdout_Unusable and then
                       Native_Refinement_Succeeded (V) and then Initial_Holdout_Valid (V) and then
                       not Refined_Holdout_Valid (V) and then Final_Inliers_Valid (V),
                       "refined held-out depth rejects, retaining initial/final available scores");
                     Reloc_Guards (V);
                     Exercised := True;
                  end;
               end if;
            end;
         end loop;
      end loop;
      Assert (Exercised, "noisy native refinement provides distinct camera half-spaces");
   end Reloc_Refined_Depth;

   procedure Reloc_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check); --  Deliberate IEEE invalid inputs.
      W : Object_Point_Array := Reloc_Field;
      H : Object_Point_Array := Reloc_H;
      P : Image_Point_Array := Reloc_Project (W);
      Q : Image_Point_Array := Reloc_Project (H);
      Policy : Relocalization_Verification_Options := Reloc_Policy;
      Options : RANSAC_Options := Reloc_RANSAC;
      Intrinsics : Camera_Intrinsics := K;
      D : Distortion_Coefficients := Reloc_D;
      procedure Reject (N : Natural := 30; M : Natural := 4; Mismatch : Boolean := False) is
      begin
         declare
            R : constant Camera_Relocalization_Result := Relocalize_Camera
              (W (1 .. N), P (1 .. N - Boolean'Pos (Mismatch)), H (1 .. M), Q (1 .. M),
               Intrinsics, D, Options, Policy);
            pragma Unreferenced (R);
         begin
            Assert (False, "invalid relocalization accepted");
         end;
      exception when OpenCV.OpenCV_Error => null;
      end Reject;
   begin
      Reject (Mismatch => True); Reject (M => 0);
      W (2) := W (1); Reject; W := Reloc_Field;
      H (2) := H (1); Reject; H := Reloc_H;
      H (1) := W (1); Reject; H := Reloc_H;
      Policy.Minimum_Consensus_Inliers := 3; Reject (0); Policy := Reloc_Policy;
      for Value of Reprojection_Error_Array'[-1.0, OpenCV.Float64_Value (Nonfinite (0)),
                                           OpenCV.Float64_Value (Nonfinite (1))] loop
         Policy.Maximum_Final_Inlier_RMS_Pixels := Value; Reject (0); Policy := Reloc_Policy;
         Policy.Maximum_Holdout_RMS_Pixels := Value; Reject (4); Policy := Reloc_Policy;
         Policy.Maximum_Holdout_Point_Error_Pixels := Value; Reject (5); Policy := Reloc_Policy;
         Policy.Maximum_Holdout_RMS_Worsening_Pixels := Value; Reject (0); Policy := Reloc_Policy;
      end loop;
      for Kind in Interfaces.Integer_32 range 0 .. 1 loop
         declare
            V : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Nonfinite (Kind));
         begin
            W (1) (0) := V; Reject; W := Reloc_Field;
            H (1) (2) := V; Reject (0); H := Reloc_H;
            P (1) (1) := V; Reject; P := Reloc_Project (W);
            Q (1) (0) := V; Reject (0); Q := Reloc_Project (H);
            Intrinsics.Focal_X := V; Reject (0); Intrinsics := K;
            D.K1 := V; Reject (0); D := Reloc_D;
            Options.Confidence := V; Reject (0); Options := Reloc_RANSAC;
         end;
      end loop;
      Options.Reprojection_Error_Pixels := 0.0; Reject (0); Options := Reloc_RANSAC;
      Intrinsics.Focal_Y := 0.0; Reject (0);
   end Reloc_Invalid;

   procedure Reloc_Adapter (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check); --  Deliberately invalid adapter pixels.
      A : constant World_To_Camera_Pose := ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
      B : constant World_To_Camera_Pose := ([0.0, 0.0, 0.0], [-2.0, 0.0, 0.0]);
      Identity : constant Camera_Intrinsics := (1.0, 1.0, 0.0, 0.0);
      Values : constant Metric_Point_Refinement_Result_Array := Refine_Metric_World_Points
        ([[0.5, 0.126], [0.0, 0.0], [0.3, 0.152]],
         [[0.25, 0.124], [0.0, 0.0], [0.1, 0.148]], Identity, Identity,
         No_Distortion, No_Distortion, A, B, (0.0, 1.0, 1.0, 1.0), (20, 0.0), (1.0, 1.0));
      Shifted : Metric_Point_Refinement_Result_Array (Positive'Last - 2 .. Positive'Last) := Values;
      Pixels : constant Image_Point_Array (7 .. 9) := [[10.0, 20.0], [30.0, 40.0], [50.0, 60.0]];
      Set : constant Accepted_Landmark_Set := Accepted_Landmark_Observations (Shifted, Pixels);
      N : Natural := 0;
   begin
      Assert (Set.Count > 0 and then Set.Count < 3, "adapter mixed accepted/unavailable");
      for I in 1 .. Values'Length loop
         if Accepted (Values (I)) then
            N := N + 1;
            Assert (Original_Positions (Set) (N) = I and then
              World_Points (Set) (N) = Accepted_World_Position (Values (I)) and then
              Image_Points (Set) (N) = Pixels (Pixels'First + (I - 1)), "compaction preserves positional pairing");
         end if;
      end loop;
      Shifted := [others => Values (2)];
      declare
         Empty : constant Accepted_Landmark_Set := Accepted_Landmark_Observations (Shifted, Pixels);
         None : constant Accepted_Landmark_Set := Accepted_Landmark_Observations
           (Values (1 .. 0), Pixels (7 .. 6));
      begin
         Assert (Empty.Count = 0 and then None.Count = 0 and then World_Points (Empty)'First = 1,
                 "empty adapter no candidate/original fallback");
      end;
      begin
         declare
            Bad : constant Accepted_Landmark_Set := Accepted_Landmark_Observations (Shifted, Pixels (7 .. 8));
            pragma Unreferenced (Bad);
         begin
            Assert (False, "adapter mismatch accepted");
         end;
      exception when OpenCV.OpenCV_Error => null; end;
      declare
         First, Second : Image_Point_Array (1 .. 30);
         New_Image : Image_Point_Array (Positive'Last - 29 .. Positive'Last);
      begin
         for I in First'Range loop
            declare
               X : constant OpenCV.Float64_Value := OpenCV.Float64_Value (I mod 6) * 0.5 - 1.3;
               Y : constant OpenCV.Float64_Value := OpenCV.Float64_Value (I / 6) * 0.4 - 0.9;
               Z : constant OpenCV.Float64_Value := 8.0 + OpenCV.Float64_Value (I mod 7) * 0.23;
            begin
               First (I) := [X / Z, Y / Z + 0.001];
               Second (I) := [(X - 2.0) / Z, Y / Z - 0.001];
               if I mod 7 = 0 then
                  First (I) := [0.0, 0.0]; Second (I) := [0.0, 0.0];
               end if;
            end;
         end loop;
         declare
            Map : constant Metric_Point_Refinement_Result_Array (11 .. 40) := Refine_Metric_World_Points
              (First, Second, Identity, Identity, No_Distortion, No_Distortion, A, B,
               (0.0, 1.0, 1.0, 1.0), (20, 0.0), (1.0, 1.0));
         begin
            for I in 1 .. Map'Length loop
               New_Image (New_Image'First + (I - 1)) := [2000.0, -1500.0];
               if Accepted (Map (Map'First + (I - 1))) and then I mod 5 /= 0 then
                  New_Image (New_Image'First + (I - 1)) := Reloc_Project
                    ([1 => Accepted_World_Position (Map (Map'First + (I - 1)))]) (1);
               end if;
            end loop;
            declare
               Compact : constant Accepted_Landmark_Set := Accepted_Landmark_Observations (Map, New_Image);
               R : constant Camera_Relocalization_Result := Relocalize_Camera
                 (World_Points (Compact), Image_Points (Compact), Reloc_H, Reloc_Project (Reloc_H),
                  K, Reloc_D, Reloc_RANSAC, Reloc_Policy);
            begin
               Assert (Compact.Count >= 20 and then Compact.Count < 30 and then Accepted (R),
                       "Task 020 accepted compact map robustly relocalizes");
               for Position of Inliers (R) loop
                  declare
                     Original : constant Positive := Original_Positions (Compact) (Position);
                  begin
                     Assert (Original mod 7 /= 0 and then Original mod 5 /= 0 and then
                       Accepted (Map (Map'First + (Original - 1))) and then
                       World_Points (Compact) (Position) =
                         Accepted_World_Position (Map (Map'First + (Original - 1))),
                       "native compact consensus maps through adapter to original landmark position");
                  end;
               end loop;
            end;
         end;
      end;
      declare
         One : constant Accepted_Landmark_Set := Accepted_Landmark_Observations
           ([Positive'Last => Values (1)], [Positive'Last => [10.0, 20.0]]);
      begin
         Assert (One.Count = 1 and then Original_Positions (One) (1) = 1,
                 "Positive Last singleton positional indexing");
      end;
      for Kind in Interfaces.Integer_32 range 0 .. 2 loop
         begin
            declare
               Bad : constant Accepted_Landmark_Set := Accepted_Landmark_Observations
                 ([1 => Values (2)], [1 => [OpenCV.Float64_Value (Nonfinite (Kind)), 0.0]]);
               pragma Unreferenced (Bad);
            begin
               Assert (False, "nonfinite adapter pixel accepted even for skipped landmark");
            end;
         exception when OpenCV.OpenCV_Error => null; end;
      end loop;
   end Reloc_Adapter;

   Pair_K : constant Camera_Intrinsics := (1.0, 1.0, 0.0, 0.0);
   Pair_A : constant World_To_Camera_Pose := ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
   Pair_B : constant World_To_Camera_Pose := ([0.0, 0.0, 0.0], [-2.0, 0.0, 0.0]);
   Pair_Policy : constant Epipolar_Verification_Options := (1, 1.0, 1.0);
   function Pair_Verify
     (P, Q : Image_Point_Array; A : World_To_Camera_Pose := Pair_A;
      B : World_To_Camera_Pose := Pair_B;
      Policy : Epipolar_Verification_Options := Pair_Policy) return Known_Pose_Epipolar_Result is
     (Verify_Known_Pose_Epipolar (P, Q, Pair_K, Pair_K, No_Distortion, No_Distortion, A, B, Policy));

   procedure Pair_Rectified (T : in out Fixture) is
      pragma Unreferenced (T);
      G : constant Known_Pose_Essential_Result := Essential_From_Camera_Poses (Pair_A, Pair_B);
      Expected : constant Essential_Matrix := [[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]];
      R : constant Known_Pose_Epipolar_Result := Pair_Verify
        ([[0.3, 0.1], [0.2, 0.0]], [[-0.1, 0.1], [-0.2, 0.2]]);
      Errors : constant Epipolar_Correspondence_Result_Array := Correspondence_Errors (R);
   begin
      Assert (G.Defined and then G.Baseline_Distance = 2.0, "known metric baseline");
      Assert (G.Translation_Direction (0) = -1.0, "translation sign");
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            Assert (Near (G.Matrix (I, J), Expected (I, J), 1.0E-14), "analytic E coefficient");
         end loop;
      end loop;
      Assert (Accepted (R) and then Errors (1).Defined and then Errors (1).Error = 0.0, "exact y equality");
      Assert (Errors (2).Defined and then Near (Errors (2).Error, 0.2 / Math.Sqrt (2.0), 1.0E-14), "analytic Sampson");
      Assert (Near (Summary (R).RMS_Normalized_Sampson, 0.1, 1.0E-14), "independent RMS");
   end Pair_Rectified;

   procedure Pair_Noncommuting (T : in out Fixture) is
      pragma Unreferenced (T);
      C : constant OpenCV.Float64_Value := Math.Cos (0.4);
      S : constant OpenCV.Float64_Value := Math.Sin (0.4);
      D : constant OpenCV.Float64_Value := Math.Cos (-0.3);
      U : constant OpenCV.Float64_Value := Math.Sin (-0.3);
      A : constant World_To_Camera_Pose := ([0.0, 0.0, 0.4], [1.0, 0.0, 0.0]);
      B : constant World_To_Camera_Pose := ([-0.3, 0.0, 0.0], [C, -D * S - 3.0, -U * S + 4.0]);
      R : constant Rotation_Matrix := [[C, S, 0.0], [-D * S, D * C, -U], [-U * S, U * C, D]];
      E : constant Essential_Matrix :=
        [[(0.6 * U + 0.8 * D) * S, -(0.6 * U + 0.8 * D) * C, -0.6 * D + 0.8 * U],
         [0.8 * C, 0.8 * S, 0.0], [0.6 * C, 0.6 * S, 0.0]];
      G : constant Known_Pose_Essential_Result := Essential_From_Camera_Poses (A, B);
      Reverse_G : constant Known_Pose_Essential_Result := Essential_From_Camera_Poses (B, A);
   begin
      for I in 0 .. 2 loop
         Assert (Near (G.Relative_Pose.Translation (I), (case I is when 0 => 0.0, when 1 => -3.0, when others => 4.0),
                       1.0E-12), "second frame translation independent oracle");
         for J in 0 .. 2 loop
            Assert (Near (G.Rotation_First_To_Second (I, J), R (I, J), 1.0E-12), "Rx Rz transpose order");
            Assert (Near (G.Matrix (I, J), E (I, J), 1.0E-12), "hand-derived signed E");
            Assert (Near (Reverse_G.Matrix (I, J), G.Matrix (J, I), 1.0E-12), "reversed E transpose");
         end loop;
      end loop;
      Assert (Near (G.Baseline_Distance, 5.0, 1.0E-12), "analytic baseline");
   end Pair_Noncommuting;

   --  Independent scalar forward Brown model, not native projection.
   function Pair_Observe (P : Object_Point; Intrinsics : Camera_Intrinsics;
                          D : Distortion_Coefficients; Second : Boolean) return Image_Point is
      C : constant OpenCV.Float64_Value := Math.Cos (0.2);
      S : constant OpenCV.Float64_Value := Math.Sin (0.2);
      X : constant OpenCV.Float64_Value :=
        (if Second then (C * P (0) - S * P (1) - 2.0) / P (2) else P (0) / P (2));
      Y : constant OpenCV.Float64_Value :=
        (if Second then (S * P (0) + C * P (1) + 0.3) / P (2) else P (1) / P (2));
      R2 : constant OpenCV.Float64_Value := X * X + Y * Y;
      Radial : constant OpenCV.Float64_Value := 1.0 + D.K1 * R2 + D.K2 * R2 ** 2 + D.K3 * R2 ** 3;
   begin
      return [Intrinsics.Focal_X * (X * Radial + 2.0 * D.P1 * X * Y + D.P2 * (R2 + 2.0 * X * X)) + Intrinsics.Center_X,
              Intrinsics.Focal_Y * (Y * Radial + D.P1 * (R2 + 2.0 * Y * Y) + 2.0 * D.P2 * X * Y) + Intrinsics.Center_Y];
   end Pair_Observe;

   procedure Pair_Calibration (T : in out Fixture) is
      pragma Unreferenced (T);
      K2 : constant Camera_Intrinsics := (630.0, 710.0, 280.0, 190.0);
      D1 : constant Distortion_Coefficients := (0.02, -0.003, 0.001, -0.002, 0.0004);
      D2 : constant Distortion_Coefficients := (-0.03, 0.004, -0.002, 0.001, -0.0005);
      B : constant World_To_Camera_Pose := ([0.0, 0.0, 0.2], [-2.0, 0.3, 0.0]);
      P, Q : Image_Point_Array (1 .. World'Length);
   begin
      for I in World'Range loop
         declare
            W : Object_Point := World (I);
         begin
            W (2) := W (2) + 8.0;
            P (I) := Pair_Observe (W, K, D1, False);
            Q (I) := Pair_Observe (W, K2, D2, True);
         end;
      end loop;
      declare
         R : constant Known_Pose_Epipolar_Result := Verify_Known_Pose_Epipolar
           (P, Q, K, K2, D1, D2, Pair_A, B, (6, 1.0E-10, 1.0E-10));
         V : constant Known_Pose_Epipolar_Result := Verify_Known_Pose_Epipolar
           (Q, P, K2, K, D2, D1, B, Pair_A, (6, 1.0E-10, 1.0E-10));
      begin
         Assert (Accepted (R) and then Accepted (V), "different all-five Brown cameras");
         for I in 1 .. R.Count loop
            Assert (Near (Correspondence_Errors (R) (I).Error, Correspondence_Errors (V) (I).Error, 1.0E-12),
                    "reversal preserves normalized error");
         end loop;
      end;
   end Pair_Calibration;

   procedure Pair_Baselines (T : in out Fixture) is
      pragma Unreferenced (T);
      B : World_To_Camera_Pose := Pair_A;
   begin
      for Pure in Boolean loop
         if Pure then B.Rotation := [0.0, 0.0, 0.7]; end if;
         declare
            G : constant Known_Pose_Essential_Result := Essential_From_Camera_Poses (Pair_A, B);
            V : constant Known_Pose_Epipolar_Result := Pair_Verify ([[0.0, 0.0]], [[0.0, 0.0]], B => B);
         begin
            Assert (not G.Defined and then G.Baseline_Distance = 0.0, "exact common center no E");
            Assert (Disposition (V) = No_Physical_Baseline and then not Geometry (V).Defined and then
                    not Correspondence_Errors (V) (1).Defined and then not Entire_Set_Evaluable (V), "no invented scores");
            begin
               declare S : constant Epipolar_Verification_Summary := Summary (V); pragma Unreferenced (S);
               begin Assert (False, "zero baseline summary guard"); end;
            exception when OpenCV.OpenCV_Error => null; end;
         end;
      end loop;
      B := Pair_B;
      for Scale of Reprojection_Error_Array'[1.0E-300, 1.0E300] loop
         B.Translation := [-Scale, 0.0, 0.0];
         declare G : constant Known_Pose_Essential_Result := Essential_From_Camera_Poses (Pair_A, B);
         begin
            Assert (G.Defined and then G.Baseline_Distance = Scale and then G.Translation_Direction (0) = -1.0,
                    "scaled tiny/large nonzero baseline");
         end;
      end loop;
      begin
         B.Translation := [OpenCV.Float64_Value'Last, OpenCV.Float64_Value'Last, 0.0];
         declare G : constant Known_Pose_Essential_Result := Essential_From_Camera_Poses (Pair_A, B);
            pragma Unreferenced (G);
         begin Assert (False, "unrepresentable baseline must raise"); end;
      exception when OpenCV.OpenCV_Error => null; end;
   end Pair_Baselines;

   procedure Pair_Thresholds (T : in out Fixture) is
      pragma Unreferenced (T);
      P : constant Image_Point_Array := [[0.0, 0.0]];
      Q : constant Image_Point_Array := [[0.0, 0.2]];
      R : constant Known_Pose_Epipolar_Result := Pair_Verify (P, Q);
      Error : constant OpenCV.Float64_Value := Correspondence_Errors (R) (1).Error;
      Below : constant OpenCV.Float64_Value := OpenCV.Float64_Value'Adjacent (Error, 0.0);
   begin
      Assert (Accepted (Pair_Verify (P, P, Policy => (1, 0.0, 0.0))), "exact zero inclusive");
      Assert (Accepted (Pair_Verify (P, Q, Policy => (1, Error, Error))), "both equality thresholds");
      Assert (not Accepted (Pair_Verify (P, Q, Policy => (1, Below, Error))), "adjacent RMS rejection");
      Assert (not Accepted (Pair_Verify (P, Q, Policy => (1, Error, Below))), "adjacent maximum rejection");
      declare
         V : constant Known_Pose_Epipolar_Result := Pair_Verify (P, [[0.0, 100.0]], Policy => (1, 0.01, 0.01));
      begin
         Assert (Disposition (V) = Rejected_By_Policy and then Entire_Set_Evaluable (V) and then
                 not Correspondence_Errors (V) (1).Passes_Point_Limit, "gross mismatch evaluated rejected");
      end;
      declare
         V : constant Known_Pose_Epipolar_Result := Pair_Verify
           ([[0.0, 0.0], [0.0, 0.0]], [[0.0, 0.0], [0.0, 1.0E160]],
            Policy => (1, OpenCV.Float64_Value'Last, OpenCV.Float64_Value'Last));
      begin
         Assert (Accepted (V) and then
                 abs (Summary (V).RMS_Normalized_Sampson / 5.0E159 - 1.0) < 1.0E-14,
                 "scaled RMS avoids squaring huge unscaled finite errors");
      end;
   end Pair_Thresholds;

   procedure Pair_Undefined (T : in out Fixture) is
      pragma Unreferenced (T);
      B : constant World_To_Camera_Pose := ([0.0, 0.0, 0.0], [0.0, 0.0, 2.0]);
      V : constant Known_Pose_Epipolar_Result := Pair_Verify
        ([[0.0, 0.0], [0.2, 0.1]], [[0.0, 0.0], [0.1, 0.05]], B => B);
      G : constant Known_Pose_Essential_Result := Essential_From_Camera_Poses (Pair_A, B);
      Huge : constant Normalized_Sampson_Error_Result := Normalized_Sampson_Error
        (G.Matrix, [1.0E300, 1.0E300], [-1.0E300, 1.0E300]);
   begin
      Assert (Disposition (V) = Undefined_Correspondence_Errors and then not Accepted (V) and then
              not Entire_Set_Evaluable (V), "epipole not zero score");
      Assert (not Correspondence_Errors (V) (1).Defined and then Correspondence_Errors (V) (2).Defined,
              "evaluate every pair after undefined");
      Assert (not Huge.Defined, "natural native arithmetic unavailability");
      declare
         Overflow : constant Known_Pose_Epipolar_Result := Pair_Verify
           ([[1.0E160, 1.0E160]], [[-1.0E160, 1.0E160]], B => B);
      begin
         Assert (Disposition (Overflow) = Undefined_Correspondence_Errors and then
                 not Correspondence_Errors (Overflow) (1).Defined, "end-to-end native numerical unavailability");
      end;
      begin
         declare S : constant Epipolar_Verification_Summary := Summary (V); pragma Unreferenced (S);
         begin Assert (False, "partial set summary guarded"); end;
      exception when OpenCV.OpenCV_Error => null; end;
   end Pair_Undefined;

   procedure Pair_Bounds (T : in out Fixture) is
      pragma Unreferenced (T);
      Empty : constant Known_Pose_Epipolar_Result := Pair_Verify ([1 .. 0 => <>], [1 .. 0 => <>]);
   begin
      Assert (Empty.Count = 0 and then Disposition (Empty) = Insufficient_Correspondences and then
              not Geometry_Evaluated (Empty) and then Correspondence_Errors (Empty)'First = 1, "empty guarded");
      begin
         declare G : constant Known_Pose_Essential_Result := Geometry (Empty); pragma Unreferenced (G);
         begin Assert (False, "insufficient geometry guard"); end;
      exception when OpenCV.OpenCV_Error => null; end;
      for N in 1 .. 2 loop
         declare
            P : constant Image_Point_Array (Positive'Last - (N - 1) .. Positive'Last) := [others => [0.2, 0.1]];
            Q : constant Image_Point_Array (7 .. 6 + N) := [others => [-0.2, 0.1]];
            R : constant Known_Pose_Epipolar_Result := Pair_Verify (P, Q);
         begin
            Assert (R.Count = N and then Accepted (R) and then Correspondence_Errors (R)'First = 1,
                    "Positive Last positional pairing");
         end;
      end loop;
   end Pair_Bounds;

   procedure Pair_Invalid (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Suppress (Validity_Check);
      P, Q : Image_Point_Array (1 .. 1) := [others => [0.0, 0.0]];
      K1, K2 : Camera_Intrinsics := Pair_K;
      D1, D2 : Distortion_Coefficients := No_Distortion;
      A, B : World_To_Camera_Pose := Pair_A;
      Policy : Epipolar_Verification_Options := Pair_Policy;
      procedure Reject (N : Natural := 0; M : Natural := 0) is
      begin
         declare R : constant Known_Pose_Epipolar_Result := Verify_Known_Pose_Epipolar
           (P (1 .. N), Q (1 .. M), K1, K2, D1, D2, A, B, Policy); pragma Unreferenced (R);
         begin Assert (False, "invalid epipolar request accepted"); end;
      exception when OpenCV.OpenCV_Error => null; end Reject;
   begin
      Reject (1, 0);
      for Value of Reprojection_Error_Array'[-1.0, OpenCV.Float64_Value (Nonfinite (0)), OpenCV.Float64_Value (Nonfinite (1))] loop
         Policy.Maximum_RMS_Normalized_Sampson := Value; Reject; Policy := Pair_Policy;
         Policy.Maximum_Point_Normalized_Sampson := Value; Reject; Policy := Pair_Policy;
      end loop;
      K1.Focal_X := 0.0; Reject; K1 := Pair_K;
      K2.Focal_Y := -1.0; Reject; K2 := Pair_K;
      for Kind in Interfaces.Integer_32 range 0 .. 1 loop
         declare V : constant OpenCV.Float64_Value := OpenCV.Float64_Value (Nonfinite (Kind));
         begin
            P (1) (0) := V; Reject (1, 1); P := [others => [0.0, 0.0]];
            Q (1) (1) := V; Reject (1, 1); Q := P;
            K1.Center_X := V; Reject; K1 := Pair_K;
            K2.Focal_Y := V; Reject; K2 := Pair_K;
            D1.K3 := V; Reject; D1 := No_Distortion;
            D2.P2 := V; Reject; D2 := No_Distortion;
            A.Rotation (1) := V; Reject; A := Pair_A;
            B.Translation (2) := V; Reject; B := Pair_A;
         end;
      end loop;
   end Pair_Invalid;

   procedure Pair_Adapter (T : in out Fixture) is
      pragma Unreferenced (T);
      function Owned return Known_Pose_Epipolar_Result is
         R : constant Camera_Relocalization_Result := Relocalize_Camera
           (Reloc_Field, Reloc_Project (Reloc_Field), Reloc_H, Reloc_Project (Reloc_H), K, Reloc_D, Reloc_RANSAC, Reloc_Policy);
         S : constant Camera_Relocalization_Result := Relocalize_Camera
           (World, Project_Points (World, K, No_Distortion, Known_Pose), Reloc_H,
            Project_Points (Reloc_H, K, No_Distortion, Known_Pose), K, No_Distortion,
            Reloc_RANSAC, (6, 0.1, 0.1, 0.1, 0.1));
      begin
         Assert (Accepted (R) and then Accepted (S), "accepted adapter fixtures");
         return Verify_Known_Pose_Epipolar
           ([[1.0, 2.0]], [[3.0, 4.0]], K, K, Reloc_D, No_Distortion, R, S, (1, 1.0, 1.0));
      end Owned;
      V : constant Known_Pose_Epipolar_Result := Owned;
      Rejected : constant Camera_Relocalization_Result := Relocalize_Camera
        ([1 .. 0 => <>], [1 .. 0 => <>], Reloc_H, Reloc_Project (Reloc_H), K, Reloc_D, Reloc_RANSAC, Reloc_Policy);
   begin
      Assert (Geometry (V).Defined and then Correspondence_Errors (V) (1).Defined, "owned after temporary relocalization values finalize");
      begin
         declare R : constant Known_Pose_Epipolar_Result := Verify_Known_Pose_Epipolar
           ([1 .. 0 => <>], [1 .. 0 => <>], K, K, Reloc_D, Reloc_D, Rejected, Rejected, Pair_Policy);
            pragma Unreferenced (R);
         begin Assert (False, "unaccepted adapter rejected even for empty"); end;
      exception when OpenCV.OpenCV_Error => null; end;
   end Pair_Adapter;

   function Suite return AUnit.Test_Suites.Access_Test_Suite is
      Result : constant AUnit.Test_Suites.Access_Test_Suite := AUnit.Test_Suites.New_Suite;
   begin
      Result.Add_Test (Caller.Create ("known pose rectified signed Essential analytic Sampson RMS", Pair_Rectified'Access));
      Result.Add_Test (Caller.Create ("known pose noncommuting signed coefficients and transpose reversal", Pair_Noncommuting'Access));
      Result.Add_Test (Caller.Create ("known pose separate Brown calibration and reversed errors", Pair_Calibration'Access));
      Result.Add_Test (Caller.Create ("known pose exact zero pure rotation tiny large baseline", Pair_Baselines'Access));
      Result.Add_Test (Caller.Create ("known pose inclusive zero equality adjacent thresholds", Pair_Thresholds'Access));
      Result.Add_Test (Caller.Create ("known pose epipole undefined mixed diagnostics native arithmetic", Pair_Undefined'Access));
      Result.Add_Test (Caller.Create ("known pose empty insufficient positional Positive Last", Pair_Bounds'Access));
      Result.Add_Test (Caller.Create ("known pose invalid finite camera policy and counts", Pair_Invalid'Access));
      Result.Add_Test (Caller.Create ("known pose accepted relocalization adapter ownership rejection", Pair_Adapter'Access));
      Result.Add_Test (Caller.Create ("relocalization analytic Brown robust inlier-only pose ownership", Reloc_Oracle'Access));
      Result.Add_Test (Caller.Create ("relocalization four five no-pose and insufficient consensus", Reloc_Boundary'Access));
      Result.Add_Test (Caller.Create ("relocalization inclusive policy adjacent rejection and worsening", Reloc_Policies'Access));
      Result.Add_Test (Caller.Create ("relocalization behind unavailable and final inlier depth", Reloc_Unusable'Access));
      Result.Add_Test (Caller.Create ("relocalization refined heldout unusable preserves available scores", Reloc_Refined_Depth'Access));
      Result.Add_Test (Caller.Create ("relocalization invalid options finite counts duplicates isolation", Reloc_Invalid'Access));
      Result.Add_Test (Caller.Create ("relocalization accepted landmark adapter positional compaction", Reloc_Adapter'Access));
      Result.Add_Test (Caller.Create ("metric refinement analytic noisy world oracle and preserved original", Metric_Refinement_Oracle'Access));
      Result.Add_Test (Caller.Create ("metric refinement independent pixel acceptance and equality", Metric_Refinement_Acceptance'Access));
      Result.Add_Test (Caller.Create ("metric refinement zero baseline unusable low parallax no improvement", Metric_Refinement_Skips'Access));
      Result.Add_Test (Caller.Create ("metric refinement noncommuting unequal intrinsics Brown reversal", Metric_Refinement_General'Access));
      Result.Add_Test (Caller.Create ("metric refinement tiny large baseline origin and overflow", Metric_Refinement_Range'Access));
      Result.Add_Test (Caller.Create ("metric refinement mixed positional Positive Last bounds and empty", Metric_Refinement_Bounds'Access));
      Result.Add_Test (Caller.Create ("metric refinement invalid finite counts calibration and policy", Metric_Refinement_Invalid'Access));
      Result.Add_Test (Caller.Create ("compiler-derived fundamental/options layouts", Fundamental_Layout'Access));
      Result.Add_Test (Caller.Create ("planar pure/general/scale independent oracle", Planar_Oracle'Access));
      Result.Add_Test (Caller.Create ("visibility independent normal-sign oracle", Visibility_Signs'Access));
      Result.Add_Test (Caller.Create ("visibility strict zero boundary", Visibility_Zero'Access));
      Result.Add_Test (Caller.Create ("visibility R*n frame oracle", Visibility_Frame'Access));
      Result.Add_Test (Caller.Create ("visibility decomposition composition", Visibility_Composition'Access));
      Result.Add_Test (Caller.Create ("visibility robust selected-inlier composition", Visibility_Selected'Access));
      Result.Add_Test (Caller.Create ("visibility pure/near rotation not applicable", Visibility_Pure'Access));
      Result.Add_Test (Caller.Create ("visibility empty/no-reference and array bounds", Visibility_Empty'Access));
      Result.Add_Test (Caller.Create ("visibility invalid hypotheses and selection", Visibility_Invalid'Access));
      Result.Add_Test (Caller.Create ("visibility Float32 overflow/underflow boundary", Visibility_Float32'Access));
      Result.Add_Test (Caller.Create ("visibility compiler result layout", Visibility_Layout'Access));
      Result.Add_Test (Caller.Create ("planar compiler layout/interchange", Planar_Layout'Access));
      Result.Add_Test (Caller.Create ("planar invalid inputs", Planar_Invalid'Access));
      Result.Add_Test (Caller.Create ("independent known-F epipolar/scale oracle", Fundamental_Oracle'Access));
      Result.Add_Test (Caller.Create ("epipolar error finite range/invalid inputs", Fundamental_Range'Access));
      Result.Add_Test (Caller.Create ("clean noncoplanar stereo fundamental RANSAC", Fundamental_Clean'Access));
      Result.Add_Test (Caller.Create ("gross vertical outlier final-F classification", Fundamental_Outliers'Access));
      Result.Add_Test (Caller.Create ("fundamental 14 rejection/15 true RANSAC", Fundamental_Minimum'Access));
      Result.Add_Test (Caller.Create ("fundamental real collinear no-model", Fundamental_No_Model'Access));
      Result.Add_Test (Caller.Create ("fundamental invalid points/options", Fundamental_Invalid'Access));
      Result.Add_Test (Caller.Create ("homography independent projective map/scale/infinity", Homography_Map'Access));
      Result.Add_Test (Caller.Create ("homography mapping range/nonfinite validation", Homography_Map_Range'Access));
      Result.Add_Test (Caller.Create ("clean robust homography final-model oracle", Homography_Clean'Access));
      Result.Add_Test (Caller.Create ("gross-outlier homography final classification", Homography_Outliers'Access));
      Result.Add_Test (Caller.Create ("collinear homography no-model contract", Homography_No_Model'Access));
      Result.Add_Test (Caller.Create ("homography four-point rejection/five robust path", Homography_Counts'Access));
      Result.Add_Test (Caller.Create ("invalid homography points/options", Homography_Invalid'Access));
      Result.Add_Test (Caller.Create ("compiler-derived homography layout/interchange", Homography_Layout'Access));
      Result.Add_Test (Caller.Create ("projectPoints identity pinhole oracle", Projection_Identity'Access));
      Result.Add_Test (Caller.Create ("projectPoints translation oracle", Projection_Translation'Access));
      Result.Add_Test (Caller.Create ("projectPoints rotation oracle", Projection_Rotation'Access));
      Result.Add_Test (Caller.Create ("projectPoints five-coefficient distortion oracle", Projection_Distortion'Access));
      Result.Add_Test (Caller.Create ("camera center identity rotation", Camera_Center_Translation'Access));
      Result.Add_Test (Caller.Create ("camera center nonidentity rotation", Camera_Center_Rotation'Access));
      Result.Add_Test (Caller.Create ("clean synthetic EPNP RANSAC pose", PnP_Clean'Access));
      Result.Add_Test (Caller.Create ("synthetic robust PnP rejects gross outliers", PnP_Outliers'Access));
      Result.Add_Test (Caller.Create ("native false return exposes no pose", PnP_Not_Found'Access));
      Result.Add_Test (Caller.Create ("invalid camera intrinsics rejected", Invalid_Intrinsics'Access));
      Result.Add_Test (Caller.Create ("invalid PnP options and counts rejected", Invalid_Options_And_Counts'Access));
      Result.Add_Test (Caller.Create ("compiler-derived C/Ada ABI layouts", ABI_Layout'Access));
      Result.Add_Test (Caller.Create ("zero-distortion normalized oracle", Normalized_Zero'Access));
      Result.Add_Test (Caller.Create ("independent five-coefficient inversion", Normalized_Brown'Access));
      Result.Add_Test (Caller.Create ("Rodrigues identity/quarter-turn/orthonormality", Rotation_Oracle'Access));
      Result.Add_Test (Caller.Create ("point transform exact and roundtrip oracles", Point_Transforms'Access));
      Result.Add_Test (Caller.Create ("direction frames/magnitude/translation oracles", Direction_Transforms'Access));
      Result.Add_Test (Caller.Create ("principal/off-axis camera bearings", Camera_Bearings'Access));
      Result.Add_Test (Caller.Create ("off-axis world ray R transpose oracle", World_Bearings'Access));
      Result.Add_Test (Caller.Create ("distorted pixel to world ray oracle", Distorted_World_Bearings'Access));
      Result.Add_Test (Caller.Create ("compiler-derived rotation matrix layout/interchange", Rotation_Layout'Access));
      Result.Add_Test (Caller.Create ("empty normalized/camera/world rays", Ray_Empty'Access));
      Result.Add_Test (Caller.Create ("invalid normalized/camera/world rays", Ray_Invalid'Access));
      Result.Add_Test (Caller.Create ("invalid pose/point/direction transforms", Transform_Invalid'Access));
      Result.Add_Test (Caller.Create ("scaled camera bearing normalization", Bearing_Range'Access));
       Result.Add_Test (Caller.Create ("independent 5/0/13 pixel diagnostic oracle", Diagnostic_Oracle'Access));
       Result.Add_Test (Caller.Create ("empty diagnostics and count mismatch", Diagnostic_Empty_And_Counts'Access));
       Result.Add_Test (Caller.Create ("scaled diagnostic range and validation", Diagnostic_Range'Access));
       Result.Add_Test (Caller.Create ("noiseless iterative refinement", Refinement_Exact'Access));
       Result.Add_Test (Caller.Create ("all-five distorted iterative refinement", Refinement_Distorted'Access));
       Result.Add_Test (Caller.Create ("RANSAC inlier subset iterative refinement", Refinement_Inliers'Access));
       Result.Add_Test (Caller.Create ("invalid refinement preserves initial pose", Refinement_Invalid'Access));
      Result.Add_Test (Caller.Create ("independent normalized Sampson and known Essential oracle", Essential_Oracle'Access));
      Result.Add_Test (Caller.Create ("Sampson undefined/range/nonfinite geometry", Essential_Error_Range'Access));
      Result.Add_Test (Caller.Create ("clean Essential and signed relative pose geometry", Essential_Clean'Access));
      Result.Add_Test (Caller.Create ("gross Essential/pose outliers and final Float64 classification", Essential_Outliers'Access));
      Result.Add_Test (Caller.Create ("Essential five rejection/six actual RANSAC", Essential_Minimum'Access));
      Result.Add_Test (Caller.Create ("Essential invalid normalized observations/options", Essential_Invalid'Access));
      Result.Add_Test (Caller.Create ("relative camera-center direction validation", Relative_Center_Invalid'Access));
      Result.Add_Test (Caller.Create ("compiler-derived Essential/relative-pose/options layouts", Essential_Layout'Access));
      Result.Add_Test (Caller.Create ("relative cheirality retains points beyond 50 baseline units", Essential_Far_Points'Access));
      Result.Add_Test (Caller.Create ("triangulation rectified disparity/behind/bounds oracle", Triangulation_Rectified'Access));
      Result.Add_Test (Caller.Create ("triangulation nonidentity scale and inverse frames", Triangulation_Frames'Access));
      Result.Add_Test (Caller.Create ("Essential pose-inlier triangulation composition", Triangulation_Composition'Access));
      Result.Add_Test (Caller.Create ("compiler-derived triangulation record layout/interchange", Triangulation_Layout'Access));
      Result.Add_Test (Caller.Create ("triangulation malformed pose rejected even when empty", Triangulation_Invalid'Access));
      Result.Add_Test (Caller.Create ("independent noisy normalized reprojection diagnostics", Triangulation_Noisy'Access));
      Result.Add_Test (Caller.Create ("independent first-only and second-only negative depths", Triangulation_Depths'Access));
      Result.Add_Test (Caller.Create ("Parallax_Rectified", Parallax_Rectified'Access));
      Result.Add_Test (Caller.Create ("Parallax_Tiny", Parallax_Tiny'Access));
      Result.Add_Test (Caller.Create ("Parallax_Transpose", Parallax_Transpose'Access));
      Result.Add_Test (Caller.Create ("Parallax_Antiparallel", Parallax_Antiparallel'Access));
      Result.Add_Test (Caller.Create ("Parallax_Frames", Parallax_Frames'Access));
      Result.Add_Test (Caller.Create ("Quality_Boundaries", Quality_Boundaries'Access));
      Result.Add_Test (Caller.Create ("Quality_Low_Parallax", Quality_Low_Parallax'Access));
      Result.Add_Test (Caller.Create ("Quality_Noisy", Quality_Noisy'Access));
      Result.Add_Test (Caller.Create ("Quality_Invalid", Quality_Invalid'Access));
      Result.Add_Test (Caller.Create ("refinement independent analytic Jacobian", Refinement_Jacobian'Access));
      Result.Add_Test (Caller.Create ("refinement scaled damped solver and trial rejection", Refinement_Solver'Access));
      Result.Add_Test (Caller.Create ("refinement clean noisy fixed pose and monotonicity", Refinement_Geometry'Access));
      Result.Add_Test (Caller.Create ("refinement low parallax mixed batch preservation", Refinement_Batch'Access));
      Result.Add_Test (Caller.Create ("refinement malformed input and empty validation", Point_Refinement_Invalid'Access));
      Result.Add_Test (Caller.Create ("refinement extreme finite numerical preservation", Refinement_Extremes'Access));
      Result.Add_Test (Caller.Create ("Parallax_Large", Parallax_Large'Access));
      Result.Add_Test (Caller.Create ("AP3P known nonidentity pose and distortion", AP3P_Known_Pose'Access));
      Result.Add_Test (Caller.Create ("AP3P multiple native hypotheses", AP3P_Multiple'Access));
      Result.Add_Test (Caller.Create ("AP3P world-to-camera frame convention", AP3P_Frame'Access));
      Result.Add_Test (Caller.Create ("AP3P count and degenerate rejection", AP3P_Reject'Access));
      Result.Add_Test (Caller.Create ("AP3P valid native zero result", AP3P_Zero'Access));
      Result.Add_Test (Caller.Create ("compiler-derived AP3P aggregate layout", AP3P_Layout'Access));
       Result.Add_Test (Caller.Create ("AP3P independent fourth-reference diagnostics", AP3P_Reference_Known'Access));
       Result.Add_Test (Caller.Create ("AP3P multiple independent reference diagnostics", AP3P_Reference_Many'Access));
       Result.Add_Test (Caller.Create ("AP3P reference diagnostics with different lower bounds", AP3P_Reference_Bounds'Access));
       Result.Add_Test (Caller.Create ("AP3P strict positive and nonpositive reference depth", AP3P_Reference_Depth'Access));
       Result.Add_Test (Caller.Create ("AP3P five-coefficient reference projection", AP3P_Reference_Distortion'Access));
       Result.Add_Test (Caller.Create ("AP3P reference numerical unavailability", AP3P_Reference_Numbers'Access));
       Result.Add_Test (Caller.Create ("AP3P reference summary and malformed records", AP3P_Reference_Summary'Access));
       Result.Add_Test (Caller.Create ("AP3P empty and invalid reference diagnostics", AP3P_Reference_Empty'Access));
      Result.Add_Test (Caller.Create ("AP3P fourth-reference unique pose selection", AP3P_Select_Known'Access));
      Result.Add_Test (Caller.Create ("AP3P distorted multi-reference unique selection", AP3P_Select_Distortion'Access));
      Result.Add_Test (Caller.Create ("AP3P ambiguous and tied reference scores", AP3P_Select_Ambiguous'Access));
      Result.Add_Test (Caller.Create ("AP3P no hypotheses and no eligible references", AP3P_Select_Empty'Access));
      Result.Add_Test (Caller.Create ("AP3P behind-camera reference rejection", AP3P_Select_Depth'Access));
      Result.Add_Test (Caller.Create ("AP3P numerically unavailable reference rejection", AP3P_Select_Unavailable'Access));
      Result.Add_Test (Caller.Create ("AP3P invalid selection thresholds and references", AP3P_Select_Invalid'Access));
      Result.Add_Test (Caller.Create ("AP3P selection with different lower bounds", AP3P_Select_Bounds'Access));
      Result.Add_Test (Caller.Create ("AP3P selection candidate-order invariance", AP3P_Select_Order'Access));
      Result.Add_Test (Caller.Create ("AP3P selection Positive'Last lower bound", AP3P_Select_Upper'Access));
      Result.Add_Test (Caller.Create ("AP3P owned pose provenance", AP3P_Select_Provenance'Access));
      Result.Add_Test (Caller.Create ("AP3P known-pose held-out refinement", AP3P_Refine_Known'Access));
      Result.Add_Test (Caller.Create ("AP3P noisy fitting with held-out acceptance", AP3P_Refine_Noisy'Access));
      Result.Add_Test (Caller.Create ("AP3P five-coefficient held-out refinement", AP3P_Refine_Distortion'Access));
      Result.Add_Test (Caller.Create ("AP3P held-out worsening rejects refinement", AP3P_Refine_Worsens'Access));
      Result.Add_Test (Caller.Create ("AP3P caller maximum rejects refinement", AP3P_Refine_Threshold'Access));
      Result.Add_Test (Caller.Create ("AP3P nonpositive held-out depth", AP3P_Refine_Depth'Access));
      Result.Add_Test (Caller.Create ("AP3P numerically unavailable held-out", AP3P_Refine_Unavailable'Access));
      Result.Add_Test (Caller.Create ("AP3P refinement without unique selection", AP3P_Refine_No_Selection'Access));
      Result.Add_Test (Caller.Create ("AP3P refinement duplicate world points", AP3P_Refine_Duplicates'Access));
      Result.Add_Test (Caller.Create ("AP3P refinement invalid inputs and thresholds", AP3P_Refine_Invalid'Access));
      Result.Add_Test (Caller.Create ("AP3P refinement different lower bounds", AP3P_Refine_Bounds'Access));
      Result.Add_Test (Caller.Create ("AP3P refinement Positive'Last lower bound", AP3P_Refine_Upper'Access));
      Result.Add_Test (Caller.Create ("AP3P direct held-out RMS worsening comparison", AP3P_Refine_Worsening_Arithmetic'Access));
      Result.Add_Test (Caller.Create ("AP3P refinement order and owned provenance", AP3P_Refine_Order'Access));
      Result.Add_Test (Caller.Create ("pose composition identity oracle", Compose_Identity'Access));
      Result.Add_Test (Caller.Create ("pose composition noncommutative order oracle", Compose_Order'Access));
      Result.Add_Test (Caller.Create ("pose composition sequential point and direction oracle", Compose_Sequential'Access));
      Result.Add_Test (Caller.Create ("pose composition camera-center oracle", Compose_Camera_Center'Access));
      Result.Add_Test (Caller.Create ("pose composition native projection with Brown distortion", Compose_Projection'Access));
      Result.Add_Test (Caller.Create ("pose composition Rodrigues boundary cases", Compose_Rodrigues'Access));
      Result.Add_Test (Caller.Create ("pose composition numerical and invalid inputs", Compose_Numbers'Access));
      Result.Add_Test (Caller.Create ("pose composition C/Ada layout, interchange and aliasing", Compose_Layout'Access));
      Result.Add_Test (Caller.Create ("pose inversion and identical relative motion", Motion_Identity'Access));
      Result.Add_Test (Caller.Create ("translation-only relative pose and baseline", Motion_Translation'Access));
      Result.Add_Test (Caller.Create ("noncommuting relative camera rotation", Motion_Noncommuting'Access));
      Result.Add_Test (Caller.Create ("pure rotation zero baseline", Motion_Pure_Rotation'Access));
      Result.Add_Test (Caller.Create ("general relative motion and point composition", Motion_General'Access));
      Result.Add_Test (Caller.Create ("pose inversion and inverse relative motion", Motion_Roundtrip'Access));
      Result.Add_Test (Caller.Create ("relative motion native projection with Brown distortion", Motion_Projection'Access));
      Result.Add_Test (Caller.Create ("relative motion numerical and invalid inputs", Motion_Range'Access));
      Result.Add_Test (Caller.Create ("relative motion frame order and displacement sign", Motion_Frame_Order'Access));
      Result.Add_Test (Caller.Create ("epipolar correction horizontal analytic oracle", Correction_Rectified'Access));
      Result.Add_Test (Caller.Create ("epipolar correction vertical analytic oracle", Correction_Vertical'Access));
      Result.Add_Test (Caller.Create ("epipolar correction F scale, sign and normalization", Correction_Scale'Access));
      Result.Add_Test (Caller.Create ("epipolar correction SVD conditioning policy", Correction_Conditioning'Access));
      Result.Add_Test (Caller.Create ("epipolar correction one-sided bound rejects native output",
                                      Correction_Native_Bound'Access));
      Result.Add_Test (Caller.Create ("epipolar correction epipole and line-quality policy", Correction_Epipoles'Access));
      Result.Add_Test (Caller.Create ("epipolar correction corrected residual policy", Correction_Residual'Access));
      Result.Add_Test (Caller.Create ("epipolar correction coordinate magnitudes", Correction_Coordinates'Access));
      Result.Add_Test (Caller.Create ("epipolar correction nonidentity noisy stereo", Correction_Stereo'Access));
      Result.Add_Test (Caller.Create ("epipolar correction after fundamental RANSAC", Correction_RANSAC'Access));
      Result.Add_Test (Caller.Create ("epipolar correction lower bounds and empty", Correction_Bounds'Access));
      Result.Add_Test (Caller.Create ("epipolar correction Positive'Last lower bound", Correction_Upper_Bound'Access));
      Result.Add_Test (Caller.Create ("epipolar correction invalid inputs and options", Correction_Invalid'Access));
      Result.Add_Test (Caller.Create ("epipolar correction C/Ada layout and interchange", Correction_Layout'Access));
      Result.Add_Test (Caller.Create ("metric rectified independent oracle", Metric_Oracle'Access));
      Result.Add_Test (Caller.Create ("metric noncommuting Brown and reversed cameras", Metric_General'Access));
      Result.Add_Test (Caller.Create ("metric zero baseline infinity and nonpositive depth", Metric_Degenerate'Access));
      Result.Add_Test (Caller.Create ("metric explicit inclusive quality policy", Metric_Quality'Access));
      Result.Add_Test (Caller.Create ("metric tiny large and overflowing coordinates", Metric_Range'Access));
      Result.Add_Test (Caller.Create ("metric extreme positional bounds and empty", Metric_Bounds'Access));
      Result.Add_Test (Caller.Create ("metric invalid inputs and options", Metric_Invalid'Access));
      return Result;
   end Suite;
end Calib3D_Tests;
