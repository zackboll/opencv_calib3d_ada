with Ada.Numerics;
with Ada.Numerics.Generic_Elementary_Functions;
with Ada.Text_IO;
with AUnit.Assertions;
with AUnit.Test_Caller;
with AUnit.Test_Fixtures;
with Interfaces;
with Interfaces.C;
with OpenCV.Calib3D;
with OpenCV.Calib3D.Internal.C_API;
with System;

package body Calib3D_Tests is
   use AUnit.Assertions;
   use OpenCV.Calib3D;
   use type OpenCV.Float64_Value;
   use type Interfaces.Integer_32;
   use type Interfaces.C.double;

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
         Assert (After.Maximum_Error_Pixels < 1.0E-7, "noiseless final pixel error");
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
          Maximum_Error_Pixels < 1.0E-7, "conservative four-point refinement contract");
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

   package Caller is new AUnit.Test_Caller (Fixture);

   function Suite return AUnit.Test_Suites.Access_Test_Suite is
      Result : constant AUnit.Test_Suites.Access_Test_Suite := AUnit.Test_Suites.New_Suite;
   begin
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
       Result.Add_Test (Caller.Create ("independent 5/0/13 pixel diagnostic oracle", Diagnostic_Oracle'Access));
       Result.Add_Test (Caller.Create ("empty diagnostics and count mismatch", Diagnostic_Empty_And_Counts'Access));
       Result.Add_Test (Caller.Create ("scaled diagnostic range and validation", Diagnostic_Range'Access));
       Result.Add_Test (Caller.Create ("noiseless iterative refinement", Refinement_Exact'Access));
       Result.Add_Test (Caller.Create ("all-five distorted iterative refinement", Refinement_Distorted'Access));
       Result.Add_Test (Caller.Create ("RANSAC inlier subset iterative refinement", Refinement_Inliers'Access));
       Result.Add_Test (Caller.Create ("invalid refinement preserves initial pose", Refinement_Invalid'Access));
      return Result;
   end Suite;
end Calib3D_Tests;
