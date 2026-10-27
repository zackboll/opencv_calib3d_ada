from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from workflow_topology import check_topology  # noqa: E402


class RepositoryTests(unittest.TestCase):
    def test_physical_homography_execution_order_and_independence(self):
        text = (ROOT / "src/opencv-calib3d.adb").read_text()
        start = text.index("   function Verify_Physical_Plane_Homography_Consistency")
        stop = text.index("   end Verify_Physical_Plane_Homography_Consistency;", start)
        operation = text[start:stop]
        self.assertLess(operation.index("Validate_Consistency_Request"), operation.index("Validate (Plane, Physical_Options)"))
        self.assertLess(operation.index("Validate (Plane, Physical_Options)"), operation.index("Algebraic : constant"))
        self.assertLess(operation.index("Algebraic : constant"), operation.index("Physical : constant"))
        self.assertLess(operation.index("Physical : constant"), operation.index("return Result"))
        self.assertEqual(operation.count(":= Verify_Known_Plane_Homography_Consistency"), 1)
        self.assertEqual(operation.count(":= Verify_Reciprocal_Plane_Correspondences"), 1)
        physical = operation[operation.index("Physical : constant"):operation.index("      begin", operation.index("Physical : constant"))]
        self.assertNotIn("Fitting_", physical)
        self.assertNotIn("if ", operation[:operation.index("return Result")])
        for forbidden in ("exception", "C.", "Estimate_", "Refine_", "Project_Points", "Solve_", "Unchecked", "Access"):
            self.assertNotIn(forbidden, operation)
        self.assertIn("not Accepted (Algebraic)", operation)
        self.assertIn("for some D of Physical => not Accepted (D)", operation)

    def test_physical_homography_owned_representation_and_example(self):
        spec = (ROOT / "src/opencv-calib3d.ads").read_text()
        record = spec[spec.index("   type Physical_Plane_Homography_Result (Fitting_Count"):]
        self.assertIn("Algebraic : Plane_Homography_Consistency_Result (Fitting_Count, Heldout_Count)", record)
        self.assertIn("Diagnostics : Physical_Plane_Holdout_Array (1 .. Heldout_Count)", record)
        self.assertNotIn("access", record)
        for name in ("cross-platform.yml", "opencv-compatibility.yml"):
            self.assertIn("bin/physical_plane_homography_synthetic", (ROOT / ".github/workflows" / name).read_text())

    def test_plane_consistency_normalized_batches_and_separation(self):
        text = (ROOT / "src/opencv-calib3d.adb").read_text()
        start = text.index("   function Verify_Known_Plane_Homography_Consistency")
        stop = text.index("   end Verify_Known_Plane_Homography_Consistency;", start)
        operation = text[start:stop]
        self.assertEqual(operation.count(":= Construct_Known_Plane_Homography"), 1)
        self.assertEqual(operation.count(":= Undistort_To_Normalized"), 4)
        self.assertEqual(operation.count(":= Estimate_Homography_RANSAC"), 1)
        self.assertEqual(operation.count(":= Transfer_Normalized_Plane_Points"), 1)
        estimate = operation[operation.index("Estimate : constant"):operation.index("Held_First : constant")]
        self.assertIn("Estimator_Points (Fit_First), Estimator_Points (Fit_Second)", estimate)
        self.assertNotIn("Holdout", estimate)
        self.assertNotIn("Held_", estimate)
        for validation in ("Validate (Holdout_Second_Pixels)", "Validate (Second_Distortion)",
                           "Validate (Second_Pose)", "for Limit of Limits loop",
                           "Options.Minimum_Final_Fitting_Inliers >", "Validate (RANSAC_Options'"):
            validator = text[text.index("   procedure Validate_Consistency_Request"):start]
            self.assertIn(validation, validator)
            self.assertLess(operation.index("Validate_Consistency_Request"), operation.index(":= Construct_Known_Plane_Homography"))
        loop = operation[operation.index("for I in Result.Diagnostics'Range", operation.index("Estimate : constant")):]
        self.assertNotIn("Undistort_To_Normalized", loop)
        self.assertNotIn("Estimate_Homography_RANSAC", loop)
        self.assertEqual(loop.count("Evaluate_Normalized_Residual ("), 3)
        self.assertLess(loop.index("D, Model_Separation)"), loop.index("D.State :="))
        self.assertNotIn("exception", operation)
        for forbidden in ("C.", "Ideal_Pixel_Homography", "Project_Points", "when OpenCV"):
            self.assertNotIn(forbidden, operation)

    def test_plane_consistency_baseline_preserved_and_example_registered(self):
        tests = (ROOT / "tests/src/calib3d_tests.adb").read_text()
        self.assertEqual(tests.count("Result.Add_Test (Caller.Create (\"plane consistency"), 11)
        for name in ("cross-platform.yml", "opencv-compatibility.yml"):
            self.assertIn("bin/plane_homography_consistency_synthetic",
                          (ROOT / ".github/workflows" / name).read_text())
        contract = (ROOT / "docs/plane-homography-consistency-contract.md").read_text()
        for limitation in ("statistical", "Float32", "not pixels", "posterior", "physical"):
            self.assertIn(limitation, contract)

    def test_known_plane_reuses_geometry_and_public_mapping(self):
        text = (ROOT / "src/opencv-calib3d.adb").read_text()
        start = text.index("   function Construct_Known_Plane_Homography")
        stop = text.index("   function Ideal_Pixel_Homography", start)
        construction = text[start:stop]
        for reused in ("Unit_Vector", "Measure_Camera_Motion", "Rotation_Matrix_Of",
                       "World_To_Camera_Direction"):
            self.assertIn(reused, construction)
        native_end = construction.index("N1 : constant Camera_Direction")
        handler_start = construction.index("         begin", native_end)
        self.assertNotIn("when OpenCV", construction)
        for forbidden in ("C.", "Compose_Poses (", "Camera_Center (", "Rotation_Matrix_Of",
                          "World_To_Camera_Direction", "Estimate_Homography"):
            self.assertNotIn(forbidden, construction[handler_start:])
        self.assertLess(construction.index("not Motion.Baseline_Direction_Defined"),
                        construction.index("if D1 = 0.0"))
        start = text.index("   function Transfer_Normalized_Plane_Points")
        stop = text.index("   end Transfer_Normalized_Plane_Points;", start)
        transfer = text[start:stop]
        self.assertIn("Map_With_Homography", transfer)
        self.assertIn("Points'First + (I - 1)", transfer)
        self.assertLess(transfer.index("Is_Finite"), transfer.index("Map_With_Homography"))
        self.assertNotIn("C.", transfer)
        self.assertNotIn("exception", transfer)

    def test_known_plane_example_in_hosted_qualification(self):
        for name in ("cross-platform.yml", "opencv-compatibility.yml"):
            text = (ROOT / ".github/workflows" / name).read_text()
            self.assertIn("bin/known_plane_homography_synthetic", text)
        contract = (ROOT / "docs/known-plane-homography-contract.md").read_text()
        self.assertIn("raw Brown-distorted", contract)
        self.assertIn("does not test positive depth", contract)

    def test_reciprocal_validation_batches_and_arithmetic_isolation(self):
        text = (ROOT / "src/opencv-calib3d.adb").read_text()
        start = text.index("   function Verify_Reciprocal_Plane_Correspondences")
        stop = text.index("   end Verify_Reciprocal_Plane_Correspondences;", start)
        operation = text[start:stop]
        validation = operation[:operation.index("Forward : constant")]
        for name in ("First_Pixels", "Second_Pixels", "First_Intrinsics", "Second_Intrinsics",
                     "First_Distortion", "Second_Distortion", "First_Pose", "Second_Pose"):
            self.assertIn(f"Validate ({name})", validation)
        self.assertIn("Validate (Plane, Options)", validation)
        policy = text[text.index("   procedure Validate (Plane : World_Plane; Options : Reciprocal_Plane_Verification_Options)"):start]
        for name in ("Options.Forward", "Options.Backward"):
            self.assertIn(f"Validate (Plane, {name})", policy)
        self.assertIn("Is_Finite (Options.Maximum_World_Separation)", policy)
        self.assertIn("Interfaces.Integer_32'Last", validation)
        self.assertEqual(operation.count(":= Verify_Plane_Constrained_Correspondences"), 2)
        self.assertIn("(First_Pixels, Second_Pixels, First_Intrinsics, Second_Intrinsics,\n"
                      "            First_Distortion, Second_Distortion, First_Pose, Second_Pose, Plane, Options.Forward)", operation)
        self.assertIn("(Second_Pixels, First_Pixels, Second_Intrinsics, First_Intrinsics,\n"
                      "            Second_Distortion, First_Distortion, Second_Pose, First_Pose, Plane, Options.Backward)", operation)
        self.assertLess(operation.index("Backward : constant"), operation.index("for I in Results'Range"))
        arithmetic = operation[operation.index("                        begin\n                           for Axis"):
                               operation.index("                  --  Separation is diagnostic")]
        for name in ("Checked_Difference", "Scaled_Vector_Norm", "when OpenCV.OpenCV_Error"):
            self.assertIn(name, arithmetic)
        for name in ("Source_World_Position", "Verify_", "Source_Intersection", "C."):
            self.assertNotIn(name, arithmetic)
        self.assertIn("if F and then B then", operation)
        self.assertLess(operation.index("R.Has_Separation := True"), operation.index("if not F and then not B then"))
        for name in ("Project_Points", "Unproject_", "World_Bearing_Rays", "Solve_", "Refine_", "C."):
            self.assertNotIn(name, operation)

    def test_reciprocal_example_and_hosted_registration(self):
        for filename in ("cross-platform.yml", "opencv-compatibility.yml"):
            self.assertIn("bin/reciprocal_plane_correspondence_synthetic",
                          (ROOT / ".github/workflows" / filename).read_text())
        example = (ROOT / "examples/src/reciprocal_plane_correspondence_synthetic.adb").read_text()
        self.assertNotIn(" := Project_Points", example)
        self.assertIn("raise Program_Error", example)

    def test_world_plane_reuses_rays_and_has_no_native_arithmetic_handler(self):
        text = (ROOT / "src/opencv-calib3d.adb").read_text()
        start = text.index("   function Intersect_World_Rays_With_Plane")
        stop = text.index("   function Project_Points", start)
        operation = text[start:stop]
        self.assertIn("Unit_Vector (Object_Point (Plane.Normal))", operation)
        self.assertIn("Unit_Vector (Object_Point (Rays", operation)
        self.assertIn("when Arithmetic_Unavailable", operation)
        self.assertNotIn("when OpenCV.OpenCV_Error", operation)
        self.assertNotIn("C.", operation)
        self.assertEqual(operation.count("World_Bearing_Rays (Pixels, Intrinsics, Distortion, Pose)"), 1)
        for filename in ("cross-platform.yml", "opencv-compatibility.yml"):
            self.assertIn("bin/ray_plane_unprojection_synthetic",
                          (ROOT / ".github/workflows" / filename).read_text())

    def test_reference_projection_is_one_batch_outside_reference_loops(self):
        text = (ROOT / "src/opencv-calib3d.adb").read_text()
        start = text.index("    function Evaluate_AP3P_References")
        stop = text.index("    function Summarize_AP3P_References", start)
        evaluator = text[start:stop]
        self.assertEqual(evaluator.count("Project_Points"), 1)
        transform_end = evaluator.index("if Eligible_Count > 0 then")
        projection = evaluator.index("Project_Points")
        residual_loop = evaluator.index("for Compact_Position in")
        self.assertLess(transform_end, projection)
        self.assertLess(projection, residual_loop)
        self.assertIn("end loop;", evaluator[evaluator.index("for Reference_Index in"):transform_end])
        self.assertIn("Eligible_World (1 .. Eligible_Count), Intrinsics, Distortion, Pose", evaluator)
        self.assertIn("Original_Positions (Eligible_Count) := Column", evaluator)
        self.assertIn("Positive_Depths (Eligible_Count) := Depth", evaluator)

    def test_three_view_reuses_existing_stages_and_isolates_third_pixels(self):
        text = (ROOT / "src/opencv-calib3d.adb").read_text()
        start = text.index("   function Qualify_Three_View_Landmarks")
        stop = text.index("   function Three_View_Landmark_Observations", start)
        operation = text[start:stop]
        fitting = operation[operation.index("Screen : constant"):operation.index("if N > 0 then")]
        self.assertNotIn("Third_Pixels", fitting)
        for name in ("Verify_Known_Pose_Epipolar", "Triangulate_World_Points",
                     "Refine_Metric_World_Points", "Evaluate_AP3P_References"):
            self.assertEqual(operation.count(name), 1)
        self.assertNotIn("C.", operation)
        for filename in ("cross-platform.yml", "opencv-compatibility.yml"):
            self.assertIn("bin/three_view_landmarks_synthetic",
                          (ROOT / ".github/workflows" / filename).read_text())

    def test_plane_correspondence_reuses_batches_and_guards_source_coordinates(self):
        text = (ROOT / "src/opencv-calib3d.adb").read_text()
        start = text.index("   function Verify_Plane_Constrained_Correspondences")
        stop = text.index("   function Plane_Constrained_Landmark_Observations", start)
        operation = text[start:stop]
        validation = operation[:operation.index("Sources : constant")]
        for value in ("First_Pixels", "Second_Pixels", "First_Intrinsics", "Second_Intrinsics",
                      "First_Distortion", "Second_Distortion", "First_Pose", "Second_Pose"):
            self.assertIn(f"Validate ({value})", validation)
        self.assertIn("Validate (Plane, Options)", validation)
        policy = text[text.index("   procedure Validate\n     (Plane : World_Plane; Options : Plane_Correspondence"):start]
        self.assertIn("Validate (Plane, Options.Intersection)", policy)
        self.assertIn("Is_Finite (Options.Maximum_Second_Pixel_Error)", policy)
        for name in ("Unproject_Pixels_Onto_World_Plane", "Evaluate_AP3P_References"):
            self.assertEqual(operation.count(name), 1)
        self.assertIn("if Accepted (Sources (I)) then", operation)
        self.assertIn("World (N) := Accepted_World_Position (Sources (I))", operation)
        self.assertIn("Mapping (N) := I", operation)
        self.assertIn("Results (Mapping (J))", operation)
        for name in ("World_Bearing_Rays", "Project_Points", "Solve_", "Refine_", "C.", "exception"):
            self.assertNotIn(name, operation)

    def test_plane_correspondence_example_contract_and_hosted_gates(self):
        for filename in ("cross-platform.yml", "opencv-compatibility.yml"):
            self.assertIn("bin/plane_correspondence_synthetic",
                          (ROOT / ".github/workflows" / filename).read_text())
        example = (ROOT / "examples/src/plane_correspondence_synthetic.adb").read_text()
        self.assertNotIn(" := Project_Points", example)
        self.assertIn("raise Program_Error", example)
        contract = (ROOT / "docs/plane-correspondence-contract.md").read_text()
        for term in ("all-or-error", "coincident physical camera centers", "six-degree-of-freedom",
                     "Source_Intersection_Rejected"):
            self.assertIn(term.lower(), contract.lower())

    def test_known_pose_examples_and_fault_sanitizer_gates(self):
        for filename in ("cross-platform.yml", "opencv-compatibility.yml"):
            text = (ROOT / ".github/workflows" / filename).read_text()
            self.assertIn("bin/known_pose_epipolar_synthetic", text)
            self.assertIn("run_ada_faults.sh sanitizers", text)
        contract = (ROOT / "docs/known-pose-epipolar-contract.md").read_text()
        self.assertIn("second**", contract)
        self.assertIn("not pixels", contract.lower())
        self.assertIn("No_Physical_Baseline", contract)

    def test_backend_split(self):
        text = (ROOT / "scripts/configure_opencv.sh").read_text()
        self.assertIn("backend=calib3d", text)
        self.assertIn("backend=geometry", text)
        self.assertIn("opencv2/calib3d.hpp", text)
        self.assertIn("opencv2/geometry/3d.hpp", text)

    def test_public_namespace_does_not_claim_geometry(self):
        self.assertTrue((ROOT / "src/opencv-calib3d.ads").exists())
        self.assertFalse(any((ROOT / "src").glob("opencv-geometry*.ads")))

    def test_bridge_not_vendored(self):
        hits = [p for p in ROOT.rglob("opencv_core_module_bridge.hpp") if "alire" not in p.parts]
        self.assertEqual(hits, [])

    def test_pnp_method_is_fixed(self):
        text = (ROOT / "cpp/opencv_calib3d_shim.cpp").read_text()
        self.assertIn("cv::SOLVEPNP_EPNP", text)
        self.assertNotIn("SOLVEPNP_DLS", text)
        self.assertNotIn("SOLVEPNP_UPNP", text)

    def test_pose_convention_documented(self):
        text = (ROOT / "docs/pnp-contract.md").read_text()
        self.assertIn("X_camera = R * X_world + t", text)
        self.assertIn("C_world = -R^T * t", text)

    def test_windows_pr_is_limited_to_its_workflow_file(self):
        text = (ROOT / ".github/workflows/windows-post-merge.yml").read_text()
        self.assertIn("pull_request:", text)
        self.assertIn("- .github/workflows/windows-post-merge.yml", text)
        self.assertNotIn("workflow_dispatch:", text)

    def test_compatibility_is_manual(self):
        text = (ROOT / ".github/workflows/opencv-compatibility.yml").read_text()
        self.assertIn("workflow_dispatch:", text)
        self.assertNotIn("pull_request:", text)

    def test_topology_parser_accepts_repository(self):
        check_topology(
            (ROOT / ".github/workflows/cross-platform.yml").read_text(),
            (ROOT / ".github/workflows/windows-post-merge.yml").read_text(),
            (ROOT / ".github/workflows/opencv-compatibility.yml").read_text(),
        )

    def test_no_features_dependency(self):
        text = (ROOT / "alire.toml").read_text()
        self.assertNotIn("opencv_features", text)

    def test_no_generated_binding_source(self):
        self.assertFalse(any(p.name.startswith("generate") for p in (ROOT / "scripts").iterdir()))

    def test_camera_rays_fixed_standard_policy(self):
        text = (ROOT / "cpp/opencv_calib3d_shim.cpp").read_text()
        self.assertIn("cv::TermCriteria::COUNT | cv::TermCriteria::EPS", text)
        self.assertIn("20, 1.0e-12", text)
        self.assertIn("images.isContinuous() ? images : images.clone()", text)
        self.assertNotIn("cv::fisheye::", text)

    def test_camera_ray_units_and_frames_documented(self):
        text = (ROOT / "docs/camera-rays-contract.md").read_text()
        for equation in ("X_world = R^T * (X_camera - t)", "d_world = R^T * d_camera",
                         "X_world(s) = C_world + s * d_world"):
            self.assertIn(equation, text)
        self.assertIn("dimensionless", text)

    def test_homography_final_classifier_is_independent_of_native_mask(self):
        text = (ROOT / "cpp/opencv_calib3d_shim.cpp").read_text()
        start = text.index("opencv_calib3d_status opencv_calib3d_find_homography_ransac(")
        end = text.index("opencv_calib3d_status opencv_calib3d_homography_result_found(", start)
        estimator = text[start:end]
        self.assertEqual(estimator.count("cv::findHomography("), 1)
        self.assertIn("cv::findHomography(source, destination, cv::RANSAC", estimator)
        self.assertIn("count >= 5", estimator)
        self.assertIn("original_source.at<cv::Vec2d>(i,0)", estimator)
        self.assertIn("original_destination.at<cv::Vec2d>(i,0)", estimator)
        self.assertIn("final_homography_inlier(value->matrix", estimator)
        self.assertIn("value->inliers.size() >= 4", estimator)
        self.assertNotRegex(estimator, r"native_mask\s*\.\s*(at|ptr|copyTo)")
        helper = (ROOT / "cpp/homography_profile.hpp").read_text()
        self.assertIn("std::hypot(xp - dx, yp - dy)", helper)
        self.assertIn("error <= threshold", helper)

    def test_homography_contract_records_precision_minimum_and_mask_difference(self):
        contract = (ROOT / "docs/homography-contract.md").read_text()
        for required in ("n=4 findHomography path bypasses RANSAC", "Float32 internally",
                         "original caller Float64", "destination-image", "h22 = 1",
                         "not a general 3-D", "4.1/4.10", "5.0", "upstream mask"):
            self.assertIn(required, contract)

    def test_fundamental_true_ransac_common_call_and_final_classifier(self):
        text = (ROOT / "cpp/opencv_calib3d_shim.cpp").read_text()
        start = text.index("opencv_calib3d_status opencv_calib3d_find_fundamental_ransac(")
        end = text.index("opencv_calib3d_status opencv_calib3d_fundamental_result_found(", start)
        estimator = text[start:end]
        self.assertEqual(estimator.count("cv::findFundamentalMat("), 1)
        self.assertIn("count >= 15", estimator)
        self.assertIn("cv::findFundamentalMat(first, second, cv::FM_RANSAC", estimator)
        self.assertIn("options->epipolar_threshold_pixels, options->confidence, native_mask)", estimator)
        self.assertIn("original_first.at<cv::Vec2d>(i,0)", estimator)
        self.assertIn("original_second.at<cv::Vec2d>(i,0)", estimator)
        self.assertIn("final_fundamental_inlier(value->matrix", estimator)
        self.assertIn("value->inliers.size() >= 7", estimator)
        self.assertNotIn("maximum_iterations", estimator)
        self.assertNotRegex(estimator, r"native_mask\s*\.\s*(at|ptr|copyTo)")
        for excluded in ("FM_LMEDS", "FM_7POINT", "FM_8POINT", "recoverPose(", "findEssentialMat("):
            self.assertNotIn(excluded, estimator)

    def test_fundamental_numeric_profile_and_all_examples(self):
        helper = (ROOT / "cpp/fundamental_profile.hpp").read_text()
        self.assertIn("threshold * threshold", helper)
        self.assertIn("static_cast<float>(squared)", helper)
        self.assertIn("confidence < DBL_EPSILON", helper)
        self.assertIn("confidence > 1 - DBL_EPSILON", helper)
        self.assertIn("std::hypot(a1,b1)", helper)
        self.assertIn("error <= threshold", helper)
        for workflow in ("cross-platform.yml", "opencv-compatibility.yml"):
            text = (ROOT / ".github/workflows" / workflow).read_text()
            for example in ("pnp_synthetic", "homography_synthetic", "fundamental_synthetic",
                            "essential_synthetic", "triangulation_synthetic", "relative_motion_synthetic",
                            "metric_triangulation_synthetic"):
                self.assertIn("bin/" + example, text)

    def test_essential_fixed_normalized_pipeline(self):
        text = (ROOT / "cpp/opencv_calib3d_shim.cpp").read_text()
        start = text.index("opencv_calib3d_status opencv_calib3d_find_essential_ransac(")
        end = text.index("opencv_calib3d_status opencv_calib3d_essential_result_found(", start)
        estimator = text[start:end]
        for required in ("count >= 6", "first_input.clone()", "second_input.clone()",
                         "cv::Mat::eye(3,3,CV_64F)", "cv::findEssentialMat(first,second,identity,cv::RANSAC",
                         "final_essential_inlier(value->matrix", "value->inliers.size() >= 5",
                         "std::numeric_limits<double>::max(),mask,cv::noArray()",
                         "value->pose_inliers.size() >= 5"):
            self.assertIn(required, estimator)
        self.assertNotRegex(estimator, r"native_mask\s*\.\s*(at|ptr|copyTo)")
        self.assertNotIn("maxIters", estimator)
        self.assertNotIn("LMEDS", estimator)
        self.assertIn("#if CV_VERSION_MAJOR >= 5", estimator)
        self.assertIn("options->normalized_epipolar_threshold,1000,native_mask);", estimator)
        self.assertIn("options->normalized_epipolar_threshold,native_mask);", estimator)
        self.assertEqual(estimator.count("cv::findEssentialMat("), 2)
        helper = (ROOT / "cpp/essential_profile.hpp").read_text()
        self.assertIn("confidence <= 0 || confidence >= 1", helper)
        self.assertIn("threshold * threshold", helper)
        self.assertIn("std::hypot(std::hypot(ax,ay),std::hypot(bx,by))", helper)

    def test_essential_normalized_only_public_contract(self):
        text = (ROOT / "src/opencv-calib3d.ads").read_text()
        start = text.index("function Estimate_Essential_RANSAC")
        end = text.index("function Found", start)
        self.assertIn("Normalized_Image_Point_Array", text[start:end])
        self.assertNotIn("Camera_Intrinsics", text[start:end])
        self.assertNotIn("Distortion", text[start:end])
        self.assertIn("X_second = R * X_first + lambda * t_hat", text)
        self.assertIn("normalize(-R^T*t_hat)", text)

    def test_epipolar_correction_is_native_and_guarded(self):
        cpp = (ROOT / "cpp/opencv_calib3d_shim.cpp").read_text()
        start = cpp.index("opencv_calib3d_status opencv_calib3d_correct_matches(")
        body = cpp[start:]
        self.assertEqual(body.count("cv::correctMatches("), 1)
        self.assertNotIn("solvePoly", cpp)
        self.assertIn("cv::SVD::compute(normalized, singular, cv::SVD::NO_UV)", body)
        self.assertLess(body.index("cv::correctMatches("), body.index("output = std::move(result)"))
        ada = (ROOT / "src/opencv-calib3d.ads").read_text()
        record = ada[ada.index("type Epipolar_Correction_Options is record"):]
        record = record[:record.index("end record;")]
        self.assertNotIn(":=", record, "correction options must have no library defaults")
        contract = (ROOT / "docs/epipolar-correction-contract.md").read_text()
        self.assertIn("necessary condition", contract)
        self.assertIn("not a global-optimality certificate", contract)
        review = (ROOT / "docs/epipolar-correction-source-review.md").read_text()
        self.assertIn("SAFETY_GATE_NOT_ESTABLISHED", review)

    def test_topology_rejects_extra_windows_branch(self):
        cross = (ROOT / ".github/workflows/cross-platform.yml").read_text()
        windows = (ROOT / ".github/workflows/windows-post-merge.yml").read_text()
        compatibility = (ROOT / ".github/workflows/opencv-compatibility.yml").read_text()
        widened = windows.replace(
            "  push:\n    branches:\n      - main\n",
            "  push:\n    branches:\n      - main\n      - feature/**\n",
            1,
        )
        self.assertNotEqual(widened, windows)
        with self.assertRaises(ValueError):
            check_topology(cross, widened, compatibility)

    def test_topology_rejects_broad_windows_pr(self):
        cross = (ROOT / ".github/workflows/cross-platform.yml").read_text()
        windows = (ROOT / ".github/workflows/windows-post-merge.yml").read_text()
        compatibility = (ROOT / ".github/workflows/opencv-compatibility.yml").read_text()
        broad = windows.replace(
            "      - .github/workflows/windows-post-merge.yml",
            "      - '**'",
        )
        with self.assertRaises(ValueError):
            check_topology(cross, broad, compatibility)
        with self.assertRaises(ValueError):
            check_topology(cross, windows + "  workflow_dispatch:\n", compatibility)

    def test_topology_rejects_extra_pr_job(self):
        cross = (ROOT / ".github/workflows/cross-platform.yml").read_text()
        windows = (ROOT / ".github/workflows/windows-post-merge.yml").read_text()
        compatibility = (ROOT / ".github/workflows/opencv-compatibility.yml").read_text()
        with self.assertRaises(ValueError):
            check_topology(cross + "\n  windows:\n    runs-on: windows-latest\n", windows, compatibility)


if __name__ == "__main__":
    unittest.main()
