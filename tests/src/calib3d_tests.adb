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
         Maximum : OpenCV.Float64_Value := 0.0;
      begin
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

   function Suite return AUnit.Test_Suites.Access_Test_Suite is
      Result : constant AUnit.Test_Suites.Access_Test_Suite := AUnit.Test_Suites.New_Suite;
   begin
      Result.Add_Test (Caller.Create ("compiler-derived fundamental/options layouts", Fundamental_Layout'Access));
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
      return Result;
   end Suite;
end Calib3D_Tests;
