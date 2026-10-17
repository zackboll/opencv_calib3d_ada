package body OpenCV.Calib3D.Internal.Refinement_Policy is
   use type OpenCV.Float64_Value;

   function Is_Finite (Value : OpenCV.Float64_Value) return Boolean is
     (Value = Value
      and then Value >= -OpenCV.Float64_Value'Last
      and then Value <= OpenCV.Float64_Value'Last);

   function Holdout_RMS_Worsening_Within_Allowance
     (Original_RMS, Refined_RMS, Allowance : OpenCV.Float64_Value) return Boolean
   is
      Change : OpenCV.Float64_Value;
   begin
      if not (Is_Finite (Original_RMS) and then Is_Finite (Refined_RMS)
              and then Is_Finite (Allowance) and then Allowance >= 0.0
              and then Original_RMS >= 0.0 and then Refined_RMS >= 0.0)
      then
         raise OpenCV.OpenCV_Error
           with "AP3P held-out RMS worsening is not representable";
      end if;
      if Refined_RMS <= Original_RMS then
         return True;
      end if;
      Change := Refined_RMS - Original_RMS;
      if not Is_Finite (Change) then
         raise OpenCV.OpenCV_Error
           with "AP3P held-out RMS worsening is not representable";
      end if;
      return Change <= Allowance;
   exception
      when Constraint_Error =>
         raise OpenCV.OpenCV_Error
           with "AP3P held-out RMS worsening is not representable";
   end Holdout_RMS_Worsening_Within_Allowance;
end OpenCV.Calib3D.Internal.Refinement_Policy;
