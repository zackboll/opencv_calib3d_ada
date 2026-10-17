package OpenCV.Calib3D.Internal.Refinement_Policy is
   --  Production held-out RMS worsening comparison. A nonpositive change is
   --  always within a nonnegative allowance. A positive change uses the direct
   --  Float64 subtraction Refined_RMS - Original_RMS. When
   --  0 <= Original_RMS < Refined_RMS <= Float64'Last, that subtraction is
   --  representable and cannot overflow. No scale, epsilon, or clamp is applied.
   function Holdout_RMS_Worsening_Within_Allowance
     (Original_RMS, Refined_RMS, Allowance : OpenCV.Float64_Value) return Boolean;
end OpenCV.Calib3D.Internal.Refinement_Policy;
