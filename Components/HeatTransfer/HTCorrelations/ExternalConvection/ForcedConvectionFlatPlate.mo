within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model ForcedConvectionFlatPlate "Forced convection on a flat plate"
  extends BaseClasses.BaseClassForcedConvectionExternal;
  parameter DynTherM.Choices.FlatPlateOrientationOpt orientation;

equation
  T_out = environment.T_amb;

  if orientation == DynTherM.Choices.FlatPlateOrientationOpt.parallelFlow then
    if Re <= 5e5 then
      // laminar regime
      Nu = 0.664*(Re^(1/2))*(Pr^(1/3));
    else
      // transitional regime
      Nu = 0.037*(Re^(4/5) - 871)*(Pr^(1/3));
    end if;

  elseif orientation == DynTherM.Choices.FlatPlateOrientationOpt.orthogonalFlow then
    Nu = 0.205*(Re^0.731)*(Pr^(1/3));

  end if;

  annotation (Documentation(info="<html>
<p>For improved accuracy, set characteristic dimension x equal to aurface area divided by perimeter of the flat plate.</p>
<h4>Reference:</h4>
<p>[1] Incropera, F. P., De Witt, D. P., Bergan, T. L., Lavine, A. S. &quot;Fundamentals of Heat and Mass Transfer&quot;, 2006, 6th ed., John Wiley &amp; Sons, p. 577.</p>
</html>"));
end ForcedConvectionFlatPlate;
