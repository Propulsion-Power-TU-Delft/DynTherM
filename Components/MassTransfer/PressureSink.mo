within DynTherM.Components.MassTransfer;
model PressureSink "Pressure sink"
  package Medium = Modelica.Media.Air.MoistAir;
  outer DynTherM.Components.Environment environment "Environmental properties";
  parameter DynTherM.CustomUnits.HydraulicResistance R=0 "Hydraulic Resistance";
  parameter Boolean allowFlowReversal=environment.allowFlowReversal
    "= true to allow flow reversal, false restricts to design direction";
  parameter Boolean use_ambient=true "Use ambient conditions for the plenum";
  parameter Modelica.Units.SI.Pressure P_di=101325 "Fixed value of pressure" annotation (Dialog(enable=not use_ambient));
  parameter Modelica.Units.SI.Temperature T_di=288.15 "Fixed value of temperature" annotation (Dialog(enable=not use_ambient));
  parameter Medium.MassFraction Xw_di=0.001 "Fixed value of water mass fraction" annotation (Dialog(enable=not use_ambient));

  Modelica.Units.SI.Pressure P_sink "Pressure of the sink";
  Medium.ThermodynamicState state_sink "Thermodynamic state of the sink";

  DynTherM.CustomInterfaces.FluidPort_A inlet(m_flow(min=if allowFlowReversal
           then -Modelica.Constants.inf else 0)) annotation (Placement(
        transformation(extent={{-120,-20},{-80,20}}, rotation=0),
        iconTransformation(extent={{-110,-10},{-90,10}})));
equation
  if use_ambient then

    if R > 0 then
      P_sink = environment.P_amb + inlet.m_flow*R;
    else
      P_sink = environment.P_amb;
    end if;

    state_sink = Medium.setState_pTX(P_sink, environment.T_amb, environment.X_amb);

  else

    P_sink = P_di;
    state_sink = Medium.setState_pTX(P_sink, T_di, {Xw_di, 1 - Xw_di});

  end if;

  inlet.P = P_sink;
  inlet.h_outflow = Medium.specificEnthalpy(state_sink);
  inlet.Xi_outflow = state_sink.X;

  annotation (Documentation(info="<html>
<p><b>Modelling options</b></p>
<p>The actual gas used in the component is determined by the replaceable <tt>Medium</tt> package. In the case of multiple component, variable composition gases, the nominal gas composition is given by <tt>Xnom</tt>, whose default value is <tt>Medium.reference_X</tt> .
<p>If <tt>R</tt> is set to zero, the pressure sink is ideal; otherwise, the inlet pressure increases proportionally to the outgoing flowrate.</p>
<p>If the <tt>in_p</tt> connector is wired, then the source pressure is given by the corresponding signal, otherwise it is fixed to <tt>p0</tt>.</p>
<p>If the <tt>in_T</tt> connector is wired, then the source temperature is given by the corresponding signal, otherwise it is fixed to <tt>T</tt>.</p>
<p>If the <tt>in_X</tt> connector is wired, then the source massfraction is given by the corresponding signal, otherwise it is fixed to <tt>Xnom</tt>.</p>
</html>",
        revisions="<html>
<ul>
<li><i>19 Nov 2004</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       Removed <tt>p0fix</tt> and <tt>Tfix</tt> and <tt>Xfix</tt>; the connection of external signals is now detected automatically.</li>
<br> Adapted to Modelica.Media
<li><i>1 Oct 2003</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       First release.</li>
</ul>
</html>"),
         Icon(graphics={     Ellipse(
          extent={{-90,-90},{90,90}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),             Text(
          extent={{-62,62},{66,-66}},
          lineColor={0,0,0},
          fillColor={159,159,223},
          fillPattern=FillPattern.None,
          textString="P SINK")}));
end PressureSink;
