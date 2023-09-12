within ThermalManagement.Components.MassTransfer;
model ValveLin "Valve for gas flows with linear pressure drop"
  package Medium = Modelica.Media.Air.MoistAir;
  parameter ThermalManagement.CustomUnits.HydraulicConductance Kv
    "Hydraulic conductance";
  parameter Boolean allowFlowReversal=environment.allowFlowReversal
    "= true to allow flow reversal, false restricts to design direction";
  outer ThermalManagement.Components.Environment environment "Environmental properties";
  Medium.MassFlowRate massFlow "Mass flowrate";
  ThermalManagement.CustomInterfaces.FluidPort_A inlet(m_flow(min=if
          allowFlowReversal then -Modelica.Constants.inf else 0)) annotation (
      Placement(transformation(extent={{-120,-20},{-80,20}}, rotation=0),
        iconTransformation(extent={{-110,-10},{-90,10}})));
  ThermalManagement.CustomInterfaces.FluidPort_B outlet(m_flow(max=if
          allowFlowReversal then +Modelica.Constants.inf else 0)) annotation (
      Placement(transformation(extent={{80,-20},{120,20}}, rotation=0),
        iconTransformation(extent={{90,-10},{110,10}})));
  Modelica.Blocks.Interfaces.RealInput opening annotation (Placement(
        transformation(
        origin={0,70},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-16,-16},{16,16}},
        rotation=270,
        origin={0,54})));
equation
  inlet.m_flow + outlet.m_flow = 0 "Mass balance";
  inlet.m_flow = massFlow;

  // Flow characteristics
  if allowFlowReversal then
    massFlow = Kv*opening*(inlet.P - outlet.P);
  else
    if (inlet.P - outlet.P > 0) then
      massFlow = Kv*opening*(inlet.P - outlet.P);
    else
      massFlow = 0;
    end if;
  end if;

  // Energy balance
  inlet.h_outflow = inStream(outlet.h_outflow);
  outlet.h_outflow = inStream(inlet.h_outflow);

  // Independent composition mass balances
  inlet.Xi_outflow = inStream(outlet.Xi_outflow);
  outlet.Xi_outflow = inStream(inlet.Xi_outflow);
  annotation (
    Icon(graphics={
        Line(
          points={{0,40},{0,0}},
          color={0,0,0}),
        Polygon(
          points={{-80,40},{-80,-40},{0,0},{-80,40}},
          lineColor={128,128,128},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{80,40},{0,0},{80,-40},{80,40}},
          lineColor={128,128,128},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-86,40},{-86,-40}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{86,40},{86,-40}},
          color={0,0,0},
          thickness=0.5)}),
    Documentation(info="<html>
<p>This very simple model provides a pressure drop which is proportional to the flowrate and to the <tt>cmd</tt> signal, without computing any fluid property.</p>
</html>",
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
end ValveLin;
