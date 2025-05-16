within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model FreeConvectionVerticalSurface
  "Free convection on a planar vertical surface"
  extends BaseClasses.BaseClassFreeConvectionExternal;

equation
  T_out = environment.T_amb;

  if Ra < 1e9 then
    Nu = 0.68 + (0.67*Ra^(1/4))/(1 + (0.492/Pr)^(9/16))^(4/9);
  else
    Nu = (0.825 + (0.387*Ra^(1/6))/(1 + (0.492/Pr)^(9/16))^(8/27))^2;
  end if;

  annotation (Documentation(info="<html>
<h4>Reference:</h4>
<p>[1] Incropera, F. P., De Witt, D. P., Bergan, T. L., Lavine, A. S. &quot;Fundamentals of Heat and Mass Transfer&quot;, 2006, 6th ed., John Wiley &amp; Sons, p. 571.</p>
</html>"));
end FreeConvectionVerticalSurface;
