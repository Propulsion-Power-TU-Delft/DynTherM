within DynTherM.Components.OneDimensional;
model ConvectionRadiation1D "Model of combined convection and radiation with 1D discretisation"

  input Area A "Heat transfer area" annotation (Dialog(enable=true));
  input CoefficientOfHeatTransfer ht=12 "Convective heat transfer coefficient" annotation (Dialog(enable=true));
  input Real eps(min=0,max=1)=0.1 "Emissivity: 1 is black body, 0 is pure reflection" annotation (Dialog(enable=true));
  parameter Integer N(min=1) "Number of control volumes";

  HeatFlowRate Q_rad[N] "Heat flow rate associated with radiation";
  HeatFlowRate Q_conv[N] "Heat flow rate associated with convection";

  CustomInterfaces.OneDimensional.HeatPort1D_A inlet(Nx=N) annotation (
      Placement(transformation(extent={{-40,-140},{40,-60}}),
        iconTransformation(extent={{-40,-140},{40,-60}})));
  CustomInterfaces.OneDimensional.HeatPort1D_B outlet(Nx=N) annotation (
     Placement(transformation(extent={{-40,60},{40,140}}), iconTransformation(
          extent={{-40,60},{40,140}})));

equation
  for i in 1:N loop
    Q_rad[i] = eps*A/N*sigma*(inlet.ports[i].T^4 - outlet.ports[i].T^4);
    Q_conv[i] = ht*A/N*(inlet.ports[i].T - outlet.ports[i].T);
    inlet.ports[i].Q_flow = Q_rad[i] + Q_conv[i];
    inlet.ports[i].Q_flow + outlet.ports[i].Q_flow = 0;
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{100,-68},{-100,-92}},
          fillColor={192,192,192},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{100,92},{-100,68}},
          fillColor={192,192,192},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{-100,68},{100,60}},
          lineColor={191,0,0},
          fillColor={191,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,-60},{100,-68}},
          lineColor={191,0,0},
          fillColor={191,0,0},
          fillPattern=FillPattern.Solid),
        Line(points={{-62,0},{38,0}},   color={191,0,0},
          origin={-80,-12},
          rotation=-90),
        Line(points={{-5,-3},{5,3}},   color={191,0,0},
          origin={-77,45},
          rotation=-90),
        Line(points={{-5,3},{5,-3}},   color={191,0,0},
          origin={-83,45},
          rotation=-90),
        Line(points={{-62,0},{38,0}},   color={191,0,0},
          origin={-50,-12},
          rotation=-90),
        Line(points={{-5,-3},{5,3}},   color={191,0,0},
          origin={-53,-45},
          rotation=-90),
        Line(points={{-5,3},{5,-3}},   color={191,0,0},
          origin={-47,-45},
          rotation=-90),
        Line(points={{-62,0},{38,0}},   color={191,0,0},
          origin={50,-12},
          rotation=-90),
        Line(points={{-5,-3},{5,3}},   color={191,0,0},
          origin={53,45},
          rotation=-90),
        Line(points={{-5,3},{5,-3}},   color={191,0,0},
          origin={47,45},
          rotation=-90),
        Line(points={{-62,0},{38,0}},   color={191,0,0},
          origin={-20,-12},
          rotation=-90),
        Line(points={{-5,-3},{5,3}},   color={191,0,0},
          origin={-17,45},
          rotation=-90),
        Line(points={{-5,3},{5,-3}},   color={191,0,0},
          origin={-23,45},
          rotation=-90),
        Line(points={{-62,0},{38,0}},   color={191,0,0},
          origin={20,-12},
          rotation=-90),
        Line(points={{-5,-3},{5,3}},   color={191,0,0},
          origin={23,45},
          rotation=-90),
        Line(points={{-5,3},{5,-3}},   color={191,0,0},
          origin={17,45},
          rotation=-90),
        Line(points={{-62,0},{38,0}},   color={191,0,0},
          origin={80,-12},
          rotation=-90),
        Line(points={{-5,-3},{5,3}},   color={191,0,0},
          origin={83,45},
          rotation=-90),
        Line(points={{-5,3},{5,-3}},   color={191,0,0},
          origin={77,45},
          rotation=-90),
        Line(points={{-58,-22},{-58,68}},
                                        color={28,108,200},
          origin={22,-58},
          rotation=-90),
        Line(points={{-5,-3},{5,3}},   color={28,108,200},
          origin={5,3},
          rotation=360),
        Line(points={{-5,3},{5,-3}},   color={28,108,200},
          origin={5,-3},
          rotation=360),
        Line(points={{-58,-22},{-58,68}},
                                        color={28,108,200},
          origin={22,-28},
          rotation=-90),
        Line(points={{-5,-3},{5,3}},   color={28,108,200},
          origin={5,33},
          rotation=360),
        Line(points={{-5,3},{5,-3}},   color={28,108,200},
          origin={5,27},
          rotation=360),
        Line(points={{-58,-22},{-58,68}},
                                        color={28,108,200},
          origin={22,-88},
          rotation=-90),
        Line(points={{-5,-3},{5,3}},   color={28,108,200},
          origin={5,-27},
          rotation=360),
        Line(points={{-5,3},{5,-3}},   color={28,108,200},
          origin={5,-33},
          rotation=360)}), Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Convection and radiation are modeled according to the simple methods implemented in the standard Modelica library</p>
</html>"));
end ConvectionRadiation1D;
