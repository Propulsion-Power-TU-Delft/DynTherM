within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model CylinderCrossFlowForced
  "Forced convection along the fuselage of an aircraft on ground"
  extends BaseClasses.BaseClassForcedConvectionExternal;

protected
  Real C1;
  Real C2;

equation
  Nu = 0.3 + C1*C2;
  C1 = (0.62*sqrt(Re)*Pr^(1/3))/(1 + (0.4/Pr)^(2/3))^(1/4);
  C2 = (1 + (Re/282e3)^(5/8))^(4/5);
  T_out = environment.T_amb;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The characteristic dimension x is equal to the fuselage diameter.</p>
<h4>Reference:</h4>
<p>[1] ASHRAE&nbsp;Handbook&nbsp;-&nbsp;HVAC&nbsp;Applications,&nbsp;chapter&nbsp;13</p>
<p>[2] Incropera, F. P., De Witt, D. P., Bergan, T. L., Lavine, A. S. &quot;Fundamentals of Heat and Mass Transfer&quot;, 2006, 6th ed., John Wiley &amp; Sons, p. 427.</p>
</html>"));
end CylinderCrossFlowForced;
