within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model AircraftOnGroundFree
  "Free convection along the fuselage of an aircraft on ground"
  extends BaseClasses.BaseClassFreeConvectionExternal;

protected
  Real C;
  Real n;

equation
  T_out = environment.T_amb;
  Nu = C*Ra^n;

  if Ra <= 1e-2 then
    C = 0.675;
    n = 0.058;
  elseif (Ra > 1e-2) and (Ra <= 1e2) then
    C = 1.02;
    n = 0.148;
  elseif (Ra > 1e2) and (Ra <= 1e4) then
    C = 0.85;
    n = 0.188;
  elseif (Ra > 1e4) and (Ra <= 1e7) then
    C = 0.48;
    n = 0.25;
  else
    C = 0.125;
    n = 0.333;
  end if;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The characteristic dimension x is equal to the fuselage diameter.</p>
<h4>Reference:</h4>
<p>[1] ASHRAE&nbsp;Handbook&nbsp;-&nbsp;HVAC&nbsp;Applications,&nbsp;chapter&nbsp;13.</p>
<p>[2] Incropera, F. P., De Witt, D. P., Bergan, T. L., Lavine, A. S. &quot;Fundamentals of Heat and Mass Transfer&quot;, 2006, 6th ed., John Wiley &amp; Sons, p. 579.</p>
</html>"));
end AircraftOnGroundFree;
