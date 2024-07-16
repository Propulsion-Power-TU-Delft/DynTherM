within DynTherM.Utilities;
model DepthOfDischarge

  parameter ElectricCharge C_nom "Nominal capacity";
  parameter Real eta "Charging/discharging efficiency";
  parameter Real DoD_start(start=0) "Depth of discharge - start value" annotation (Dialog(tab="Initialization"));

  Modelica.Blocks.Continuous.Integrator integrator(k=-1) "Used Energy"
    annotation (Placement(transformation(extent={{-60,30},{-40,50}})));
  Modelica.Blocks.Math.Add add
    annotation (Placement(transformation(extent={{10,10},{30,30}})));
  Modelica.Blocks.Math.Division division
    annotation (Placement(transformation(extent={{50,-10},{70,10}})));
  Modelica.Blocks.Math.Gain gain(final k=DoD_start) "Inital value of DOD"
    annotation (Placement(transformation(extent={{-58,-48},{-42,-32}})));

  Modelica.Blocks.Interfaces.RealOutput DoD "Depth of discharge"
    annotation (Placement(transformation(extent={{88,-14},{116,14}}),
        iconTransformation(extent={{88,-14},{116,14}})));
  Modelica.Blocks.Sources.Constant nominalCapacity(k=C_nom) annotation (Placement(transformation(extent={{-100,-80},{-80,-60}})));
  Modelica.Blocks.Interfaces.RealInput I annotation (Placement(transformation(
          extent={{-114,-14},{-86,14}}), iconTransformation(extent={{-114,-14},{
            -86,14}})));

  Modelica.Blocks.Math.Gain efficiency(final k=eta)
    "Charge/discharge efficiency"
    annotation (Placement(transformation(extent={{-28,32},{-12,48}})));
equation
  assert(DoD > 1, "SoC < 0 detected", level=AssertionLevel.warning);
  assert(DoD < 0, "SoC > 1 detected", level=AssertionLevel.warning);

  connect(I, integrator.u) annotation (Line(points={{-100,0},{-80,0},{-80,40},{-62,
          40}}, color={0,0,127}));
  connect(gain.y, add.u2) annotation (Line(
      points={{-41.2,-40},{-20,-40},{-20,14},{8,14}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(division.y,DoD)  annotation (Line(
      points={{71,0},{86,0},{86,1.77636e-15},{102,1.77636e-15}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(division.u2,gain. u) annotation (Line(
      points={{48,-6},{40,-6},{40,-70},{-70,-70},{-70,-40},{-59.6,-40}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(nominalCapacity.y, gain.u) annotation (Line(
      points={{-79,-70},{-70,-70},{-70,-40},{-59.6,-40}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(integrator.y, efficiency.u)
    annotation (Line(points={{-39,40},{-29.6,40}}, color={0,0,127}));
  connect(efficiency.y, add.u1) annotation (Line(points={{-11.2,40},{0,40},{0,
          26},{8,26}}, color={0,0,127}));
  connect(add.y, division.u1) annotation (Line(points={{31,20},{40,20},{40,6},
          {48,6}}, color={0,0,127}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
            100}})),
    Icon(coordinateSystem(extent={{-100,-100},{100,100}}, preserveAspectRatio=
            false), graphics={
                            Text(
          extent={{-150,16},{-116,4}},
          lineColor={0,0,0},
          textString="%"),
                        Rectangle(
          extent={{-40,60},{40,-80}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.VerticalCylinder),
                                          Rectangle(
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.VerticalCylinder,
          extent={{-20,6},{20,-6}},
          origin={0,66},
          rotation=180),                            Rectangle(
          fillColor={170,255,85},
          fillPattern=FillPattern.VerticalCylinder,
          extent={{-40,40},{40,-40}},
          origin={-7.10543e-15,-40},
          rotation=180,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Line(
          points={{-10,40},{10,40}},
          color={0,0,0},
          thickness=1),
        Line(
          points={{-10,0},{10,0}},
          color={0,0,0},
          thickness=1,
          origin={0,40},
          rotation=90),
        Line(
          points={{-10,-60},{10,-60}},
          color={0,0,0},
          thickness=1)}),
    Documentation(info="<html>
<p>The depth of discharge (DOD) is the ratio between the removed charge (integral of the battery current) and the nominal charge.</p>
<p>If DOD is zero the battery is fully charged and if DOD is one the battery is fully discharged.</p>
</html>"));
end DepthOfDischarge;
