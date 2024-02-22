within DynTherM.Systems.HydrogenTank.Subsystems;
model TankPressureController
  parameter Pressure P_vent "Venting pressure";
  parameter MassFlowRate f_vent_max=100 "Maximum allowable venting mass flow rate";

  Modelica.Blocks.Sources.RealExpression Vent_Pressure(y=P_vent)
    annotation (Placement(transformation(extent={{-98,-20},{-70,0}})));
  Modelica.Blocks.Sources.RealExpression Zero
    annotation (Placement(transformation(extent={{-76,50},{-54,68}})));
  Modelica.Blocks.Math.Max max1
    annotation (Placement(transformation(extent={{-28,2},{-12,18}})));
  Modelica.Blocks.Math.Feedback Subtract2 annotation (Placement(visible=true,
        transformation(
        origin={-51,-10},
        extent={{-7,8},{7,-8}},
        rotation=90)));
  Modelica.Blocks.Math.Gain gain(k=-0.00012)
                                         annotation (Placement(transformation(
        extent={{8,-8},{-8,8}},
        rotation=180,
        origin={8,10})));
  Modelica.Blocks.Math.Min min1 annotation (Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=0,
        origin={48,54})));
  Modelica.Blocks.Math.Add add annotation (Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=90,
        origin={30,30})));
  Modelica.Blocks.Interfaces.RealInput Tank_Pressure annotation (Placement(
        transformation(extent={{-120,-80},{-80,-40}}),  iconTransformation(
          extent={{-120,-80},{-80,-40}})));

  Modelica.Blocks.Math.Max max2
    annotation (Placement(transformation(extent={{70,50},{90,70}})));
  Modelica.Blocks.Sources.RealExpression Max_f_vent(y=f_vent_max)
    annotation (Placement(transformation(extent={{-80,70},{-52,90}})));
  Modelica.Blocks.Math.Gain gain1(k=-1)  annotation (Placement(transformation(
        extent={{8,-8},{-8,8}},
        rotation=180,
        origin={-32,80})));

  Modelica.Blocks.Interfaces.RealInput External_Massflow annotation (
      Placement(transformation(extent={{-120,-50},{-80,-10}}),
        iconTransformation(extent={{-120,40},{-80,80}})));
  Modelica.Blocks.Interfaces.RealOutput Required_Massflow annotation (
      Placement(transformation(extent={{96,40},{136,80}}),
        iconTransformation(extent={{80,-20},{120,20}})));
equation
  connect(Subtract2.y, max1.u2) annotation (Line(points={{-51,-3.7},{-51,5.2},{-29.6,
          5.2}}, color={0,0,127}));
  connect(max1.y, gain.u)
    annotation (Line(points={{-11.2,10},{-1.6,10}},color={0,0,127}));
  connect(Vent_Pressure.y, Subtract2.u2)
    annotation (Line(points={{-68.6,-10},{-57.4,-10}},
                                                     color={0,0,127}));
  connect(Zero.y, max1.u1) annotation (Line(points={{-52.9,59},{-44,59},{-44,14.8},
          {-29.6,14.8}},color={0,0,127}));
  connect(Zero.y, min1.u1) annotation (Line(points={{-52.9,59},{38,59},{38,58},{
          38.4,58},{38.4,58.8}},
                        color={0,0,127}));
  connect(add.y, min1.u2) annotation (Line(points={{30,38.8},{30,49.2},{38.4,49.2}},
        color={0,0,127}));
  connect(gain.y, add.u1) annotation (Line(points={{16.8,10},{25.2,10},{25.2,20.4}},
        color={0,0,127}));
  connect(Tank_Pressure, Subtract2.u1) annotation (Line(points={{-100,-60},
          {-51,-60},{-51,-15.6}},
                        color={0,0,127}));
  connect(min1.y, max2.u2) annotation (Line(points={{56.8,54},{68,54}},
                        color={0,0,127}));
  connect(Max_f_vent.y, gain1.u)
    annotation (Line(points={{-50.6,80},{-41.6,80}},
                                                  color={0,0,127}));
  connect(gain1.y, max2.u1) annotation (Line(points={{-23.2,80},{60,80},{60,66},
          {68,66}},           color={0,0,127}));
  connect(External_Massflow, add.u2) annotation (Line(points={{-100,-30},{
          34.8,-30},{34.8,20.4}}, color={0,0,127}));
  connect(max2.y, Required_Massflow)
    annotation (Line(points={{91,60},{116,60}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid), Text(
          extent={{-60,54},{60,-40}},
          lineColor={0,0,0},
          textString="Tank
Pressure
Control")}),    Diagram(coordinateSystem(preserveAspectRatio=false)));
end TankPressureController;
