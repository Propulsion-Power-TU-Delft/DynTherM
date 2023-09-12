within DynTherM.CustomInterfaces;
connector MechanicalPort "Connector for exchange of mechanical power"
  Modelica.Units.SI.Torque M "Torque";
  Modelica.Units.SI.AngularVelocity omega "Angular velocity";
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
end MechanicalPort;
