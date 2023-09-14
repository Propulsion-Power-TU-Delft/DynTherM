within DynTherM.CustomInterfaces;
connector FluidPort "Connector for moist air flows"
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  flow Medium.MassFlowRate m_flow
    "Mass flow rate from the connection point into the component";
  Medium.AbsolutePressure P "Pressure in the connection point";
  stream Medium.SpecificEnthalpy h_outflow
    "Specific thermodynamic enthalpy close to the connection point if m_flow < 0";
  stream Medium.MassFraction Xi_outflow[Medium.nX]
    "Independent mixture mass fractions m_i/m close to the connection point if m_flow < 0";
  annotation (Icon(graphics), Documentation(info="<HTML>
</HTML>",
        revisions="<html>
<ul>
<li><i>20 Dec 2004</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       Adapted to Modelica.Media.</li>
<li><i>5 Mar 2004</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       First release.</li>
</ul>
</html>"));
end FluidPort;
