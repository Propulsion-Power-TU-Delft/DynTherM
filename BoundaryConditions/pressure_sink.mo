within DynTherM.BoundaryConditions;
model pressure_sink "Pressure sink"
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  outer DynTherM.Components.Environment environment "Environmental properties";

  parameter DynTherM.CustomUnits.HydraulicResistance R=0 "Hydraulic Resistance";
  parameter Boolean allowFlowReversal=environment.allowFlowReversal
    "= true to allow flow reversal, false restricts to design direction";
  parameter Boolean use_ambient=true "Use ambient conditions for the plenum";
  input Modelica.Units.SI.Pressure P_di=101325 "Fixed value of pressure" annotation (Dialog(enable=not use_ambient));
  input Modelica.Units.SI.Temperature T_di=288.15 "Fixed value of temperature" annotation (Dialog(enable=not use_ambient));
  input Medium.MassFraction X_di[Medium.nX]=Medium.reference_X "Fixed value of mass fractions" annotation (Dialog(enable=not use_ambient));

  Medium.ThermodynamicState state_sink "Thermodynamic state of the sink";

  DynTherM.CustomInterfaces.FluidPort_A inlet(redeclare package Medium = Medium,
                                              m_flow(min=if allowFlowReversal
           then -Modelica.Constants.inf else 0)) annotation (Placement(
        transformation(extent={{-120,-20},{-80,20}}, rotation=0),
        iconTransformation(extent={{-110,-10},{-90,10}})));
equation
  if use_ambient then

    if R > 0 then
      inlet.P = environment.P_amb + inlet.m_flow*R;
    else
      inlet.P = environment.P_amb;
    end if;

    state_sink = Medium.setState_pTX(environment.P_amb, environment.T_amb, environment.X_amb);

  else

    inlet.P = P_di;
    state_sink = Medium.setState_pTX(P_di, T_di, X_di);
  end if;

  inlet.Xi_outflow = X_di;
  inlet.h_outflow = Medium.specificEnthalpy(state_sink);

  annotation (Documentation(info="<html>
<p>The actual gas used in the component is determined by the replaceable Medium package.</p>
<p>The sink pressure, temperature and mass fraction can be either specified as parameter or input.</p>
<p>Model adapted from ThermoPower library by Francesco Casella.</p>
</html>",
        revisions="<html>
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
end pressure_sink;
