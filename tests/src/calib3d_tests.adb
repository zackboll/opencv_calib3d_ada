with Ada.Numerics;
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

   type Fixture is new AUnit.Test_Fixtures.Test_Fixture with null record;

   K : constant Camera_Intrinsics :=
     (Focal_X => 800.0, Focal_Y => 820.0, Center_X => 320.0, Center_Y => 240.0);
   Known_Pose : constant World_To_Camera_Pose :=
     (Rotation => (0 => 0.10, 1 => -0.05, 2 => 0.08),
      Translation => (0 => 0.20, 1 => -0.10, 2 => 6.00));
   World : constant Object_Point_Array :=
     [(0 => -1.0, 1 => -1.0, 2 => 0.0),
      (0 =>  0.0, 1 => -1.0, 2 => 0.2),
      (0 =>  1.0, 1 => -1.0, 2 => 0.4),
      (0 => -1.2, 1 =>  0.0, 2 => 0.5),
      (0 =>  0.0, 1 =>  0.0, 2 => 0.8),
      (0 =>  1.2, 1 =>  0.0, 2 => 0.3),
      (0 => -1.0, 1 =>  1.0, 2 => 1.0),
      (0 =>  0.0, 1 =>  1.0, 2 => 1.3),
      (0 =>  1.0, 1 =>  1.0, 2 => 0.7),
      (0 => -0.5, 1 => -0.4, 2 => 1.7),
      (0 =>  0.6, 1 => -0.3, 2 => 1.9),
      (0 =>  0.3, 1 =>  0.7, 2 => 2.1),
      (0 => -1.4, 1 =>  0.6, 2 => 1.5),
      (0 =>  1.5, 1 =>  0.5, 2 => 1.2),
      (0 => -0.8, 1 => -1.4, 2 => 1.1),
      (0 =>  0.9, 1 => -1.3, 2 => 1.6),
      (0 => -1.5, 1 => -0.5, 2 => 2.0),
      (0 =>  1.4, 1 => -0.6, 2 => 2.2),
      (0 => -0.2, 1 =>  1.5, 2 => 1.8),
      (0 =>  0.8, 1 =>  1.4, 2 => 2.4)];

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
      Points : constant Object_Point_Array := [(0 => 1.0, 1 => 2.0, 2 => 10.0)];
      Intrinsics : constant Camera_Intrinsics :=
        (Focal_X => 100.0, Focal_Y => 200.0, Center_X => 10.0, Center_Y => 20.0);
      Identity : constant World_To_Camera_Pose :=
        (Rotation => (others => 0.0), Translation => (others => 0.0));
      Result : constant Image_Point_Array := Project_Points (Points, Intrinsics, No_Distortion, Identity);
   begin
      Assert (Result'Length = 1, "projection result count");
      Assert (Near (Result (1) (0), 20.0) and then Near (Result (1) (1), 60.0),
              "identity pinhole projection differs");
   end Projection_Identity;

   procedure Projection_Translation (T : in out Fixture) is
      pragma Unreferenced (T);
      Points : constant Object_Point_Array := [(0 => 0.0, 1 => 0.0, 2 => 5.0)];
      Intrinsics : constant Camera_Intrinsics :=
        (Focal_X => 100.0, Focal_Y => 100.0, Center_X => 10.0, Center_Y => 20.0);
      Shift : constant World_To_Camera_Pose :=
        (Rotation => (others => 0.0), Translation => (0 => 1.0, 1 => 0.0, 2 => 0.0));
      Result : constant Image_Point_Array := Project_Points (Points, Intrinsics, No_Distortion, Shift);
   begin
      Assert (Near (Result (1) (0), 30.0) and then Near (Result (1) (1), 20.0),
              "translation projection differs");
   end Projection_Translation;

   procedure Projection_Distortion (T : in out Fixture) is
      pragma Unreferenced (T);
      Points : constant Object_Point_Array := [(0 => 1.0, 1 => 2.0, 2 => 10.0)];
      Intrinsics : constant Camera_Intrinsics :=
        (Focal_X => 100.0, Focal_Y => 100.0, Center_X => 0.0, Center_Y => 0.0);
      Identity : constant World_To_Camera_Pose :=
        (Rotation => (others => 0.0), Translation => (others => 0.0));
      Distortion : constant Distortion_Coefficients :=
        (K1 => 0.1, K2 => 0.0, P1 => 0.0, P2 => 0.0, K3 => 0.0);
      Result : constant Image_Point_Array := Project_Points (Points, Intrinsics, Distortion, Identity);
   begin
      Assert (Near (Result (1) (0), 10.05, 1.0E-9)
              and then Near (Result (1) (1), 20.10, 1.0E-9),
              "radial distortion oracle differs");
   end Projection_Distortion;

   procedure Camera_Center_Translation (T : in out Fixture) is
      pragma Unreferenced (T);
      P : constant World_To_Camera_Pose :=
        (Rotation => (others => 0.0), Translation => (0 => 1.0, 1 => 2.0, 2 => 3.0));
      Center : constant Object_Point := Camera_Center (P);
   begin
      Assert (Near (Center (0), -1.0) and then Near (Center (1), -2.0)
              and then Near (Center (2), -3.0), "identity camera center differs");
   end Camera_Center_Translation;

   procedure Camera_Center_Rotation (T : in out Fixture) is
      pragma Unreferenced (T);
      P : constant World_To_Camera_Pose :=
        (Rotation => (0 => 0.0, 1 => 0.0,
                      2 => OpenCV.Float64_Value (Ada.Numerics.Pi / 2.0)),
         Translation => (0 => 1.0, 1 => 2.0, 2 => 3.0));
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
         Assert (Inlier_Count (Estimate) >= 10, "too few robust inliers");
         declare
            Accepted : constant Inlier_Index_Array := Inliers (Estimate);
         begin
            Assert (not Contains (Accepted, 2) and then not Contains (Accepted, 7)
                    and then not Contains (Accepted, 15), "gross synthetic outlier accepted");
            for I in Accepted'First + 1 .. Accepted'Last loop
               Assert (Accepted (I) > Accepted (I - 1), "inliers are not strictly ascending");
            end loop;
         end;
      end;
   end PnP_Outliers;

   procedure Invalid_Intrinsics (T : in out Fixture) is
      pragma Unreferenced (T);
      Points : constant Object_Point_Array := [(0 => 0.0, 1 => 0.0, 2 => 5.0)];
      Bad : constant Camera_Intrinsics :=
        (Focal_X => 0.0, Focal_Y => 100.0, Center_X => 10.0, Center_Y => 20.0);
      Identity : constant World_To_Camera_Pose :=
        (Rotation => (others => 0.0), Translation => (others => 0.0));
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
      Assert (Intrinsics.Focal_X = 800.0 and then Intrinsics.Center_Y = 240.0,
              "C-written intrinsics interchange");
      Assert (Distortion.K1 = 0.1 and then Distortion.K3 = 0.03,
              "C-written distortion interchange");
      Assert (Native_Pose.RX = 0.1 and then Native_Pose.TZ = 6.0,
              "C-written pose interchange");
   end ABI_Layout;

   package Caller is new AUnit.Test_Caller (Fixture);

   function Suite return AUnit.Test_Suites.Access_Test_Suite is
      Result : constant AUnit.Test_Suites.Access_Test_Suite := AUnit.Test_Suites.New_Suite;
   begin
      Result.Add_Test (Caller.Create ("projectPoints identity pinhole oracle", Projection_Identity'Access));
      Result.Add_Test (Caller.Create ("projectPoints translation oracle", Projection_Translation'Access));
      Result.Add_Test (Caller.Create ("projectPoints radial distortion oracle", Projection_Distortion'Access));
      Result.Add_Test (Caller.Create ("camera center identity rotation", Camera_Center_Translation'Access));
      Result.Add_Test (Caller.Create ("camera center nonidentity rotation", Camera_Center_Rotation'Access));
      Result.Add_Test (Caller.Create ("clean synthetic EPNP RANSAC pose", PnP_Clean'Access));
      Result.Add_Test (Caller.Create ("synthetic robust PnP rejects gross outliers", PnP_Outliers'Access));
      Result.Add_Test (Caller.Create ("invalid camera intrinsics rejected", Invalid_Intrinsics'Access));
      Result.Add_Test (Caller.Create ("invalid PnP options and counts rejected", Invalid_Options_And_Counts'Access));
      Result.Add_Test (Caller.Create ("compiler-derived C/Ada ABI layouts", ABI_Layout'Access));
      return Result;
   end Suite;
end Calib3D_Tests;
