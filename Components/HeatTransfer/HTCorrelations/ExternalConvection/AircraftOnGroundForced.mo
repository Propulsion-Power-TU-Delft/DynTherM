within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model AircraftOnGroundForced
  "Forced convection along the fuselage of an aircraft on ground"
  extends BaseClasses.BaseClassForcedConvectionExternal;

equation
  Nu = 0.0266*(Re^0.805)*Pr^(1/3);
  T_out = environment.T_amb;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The characteristic dimension x is equal to the fuselage diameter.</p>
<h4>Reference:</h4>
<p>[1] ASHRAE&nbsp;Handbook&nbsp;-&nbsp;HVAC&nbsp;Applications,&nbsp;chapter&nbsp;13</p>
</html>"));
end AircraftOnGroundForced;
