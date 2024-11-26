within DynTherM.Utilities;
model StateOfCharge

  parameter ElectricCharge C_nom "Nominal capacity";
  parameter Real eta "Charging/discharging efficiency";
  parameter Real SoC_start "Start Condition" annotation (Dialog(tab="Initialization"));

  Modelica.Blocks.Math.Add toSOC(k1=1, k2=-1)
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  Modelica.Blocks.Sources.Constant full_charge(k=1)
    annotation (Placement(transformation(extent={{-60,20},{-40,40}})));
  DepthOfDischarge DoD(
    C_nom=C_nom,
    eta=eta,
    DoD_start=1 - SoC_start)
    annotation (Placement(transformation(extent={{-60,-50},{-20,-10}})));
  Modelica.Blocks.Interfaces.RealInput I annotation (Placement(transformation(
          extent={{-114,-14},{-86,14}}), iconTransformation(extent={{-114,-14},{
            -86,14}})));
  Modelica.Blocks.Interfaces.RealOutput SoC "State of charge"
    annotation (Placement(transformation(extent={{88,-14},{116,14}}),
        iconTransformation(extent={{88,-14},{116,14}})));

equation
  connect(full_charge.y, toSOC.u1)
    annotation (Line(points={{-39,30},{0,30},{0,6},{18,6}}, color={0,0,127}));
  connect(DoD.DoD, toSOC.u2) annotation (Line(points={{-19.6,-30},{0,-30},{0,-6},
          {18,-6}}, color={0,0,127}));
  connect(I, DoD.I) annotation (Line(points={{-100,1.77636e-15},{-70,1.77636e-15},
          {-70,-30},{-60,-30}}, color={0,0,127}));
  connect(toSOC.y, SoC)
    annotation (Line(points={{41,0},{102,0}}, color={0,0,127}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-60},{100,
            60}})),
    Icon(graphics={         Text(
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
          origin={0,-40},
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
<p>The state of charge (SOC) indicates how full the battery is currently.</p>
<p>If SOC is zero the battery is empty and if SOC is one the battery is fully charged.</p>
</html>"));
end StateOfCharge;
