within DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection;
model FreeConvectionHorizontalSurface
  "Free convection in a horizontal rectangular cavity"
  extends BaseClasses.BaseClassFreeConvectionInternal;
  parameter DynTherM.Choices.FlatPlateOrientationOpt orientation;

equation

  // heated from the bottom
  if orientation == DynTherM.Choices.FlatPlateOrientationOpt.facingUp then
    if Ra > 1708 then
      Nu = 0.069*(Ra^(1/3))*(Pr^0.074);
    else
      Nu = 1;
    end if;

  // heated from the top
  elseif orientation == DynTherM.Choices.FlatPlateOrientationOpt.facingDown then
    Nu = 1;
  end if;

  annotation (Documentation(info="<html>
<h4>Reference:</h4>
<p>[1] Incropera, F. P., De Witt, D. P., Bergan, T. L., Lavine, A. S. &quot;Fundamentals of Heat and Mass Transfer&quot;, 2006, 6th ed., John Wiley &amp; Sons, p. 588.</p>
</html>"));
end FreeConvectionHorizontalSurface;
