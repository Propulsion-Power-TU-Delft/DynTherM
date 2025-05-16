within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model FreeConvectionHorizontalSurface
  "Free convection on a planar horizontal surface"
  extends BaseClasses.BaseClassFreeConvectionExternal;
  parameter DynTherM.Choices.FlatPlateOrientationOpt orientation;

equation
  T_out = environment.T_amb;

  if orientation == DynTherM.Choices.FlatPlateOrientationOpt.facingUp then
    // hot surface
    if T_surf > environment.T_amb then
      if Ra < 1e7 then
        Nu = 0.54*Ra^(1/4);
      else
        Nu = 0.15*Ra^(1/3);
      end if;

    // cold surface
    else
      Nu = 0.27*Ra^(1/4);
    end if;

  elseif orientation == DynTherM.Choices.FlatPlateOrientationOpt.facingDown then
    // hot surface
    if T_surf > environment.T_amb then
      Nu = 0.27*Ra^(1/4);

    // cold surface
    else
      if Ra < 1e7 then
        Nu = 0.54*Ra^(1/4);
      else
        Nu = 0.15*Ra^(1/3);
      end if;
    end if;
  end if;

  annotation (Documentation(info="<html>
<p>For improved accuracy, set characteristic dimension x equal to aurface area divided by perimeter of the flat plate.</p>
<h4>Reference:</h4>
<p>[1] Incropera, F. P., De Witt, D. P., Bergan, T. L., Lavine, A. S. &quot;Fundamentals of Heat and Mass Transfer&quot;, 2006, 6th ed., John Wiley &amp; Sons, p. 577.</p>
</html>"));
end FreeConvectionHorizontalSurface;
