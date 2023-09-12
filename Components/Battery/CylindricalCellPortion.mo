within ThermalManagement.Components.Battery;
model CylindricalCellPortion "1D thermal model of 1/4 portion of a cylindrical cell"

  replaceable model CellMat=ThermalManagement.Materials.NCA18650  constrainedby
    ThermalManagement.Materials.CellProperties "Material properties of the prescribed cell type" annotation (choicesAllMatching=true);

  parameter Real SOC "State-of-charge [%]";
  parameter Modelica.Units.SI.Length H_cell "Height of the cell";
  parameter Modelica.Units.SI.Length D_cell "Diameter of the cell";
  parameter Modelica.Units.SI.Length D_pin "Diameter of the central pin";
  parameter Modelica.Units.SI.Mass M_cell "Weight of the cell portion";
  constant Real pi=Modelica.Constants.pi;

  // Initialization
  parameter Modelica.Units.SI.Temperature Tstart "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter ThermalManagement.Choices.InitOpt initOpt "Initialization option" annotation (Dialog(tab="Initialization"));

  Modelica.Units.SI.SpecificHeatCapacity cm "Specific heat capacity at the prescribed SOC";
  Modelica.Units.SI.ThermalConductivity lambda "Thermal conductivity at the prescribed SOC";
  Modelica.Units.SI.HeatCapacity Cm "Heat capacity of the cell portion";
  Modelica.Units.SI.Temperature T_cell(start=Tstart) "Average temperature of the cell portion";

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a int annotation (Placement(
        transformation(extent={{-10,10},{10,30}}),  iconTransformation(extent={{-10,10},
            {10,30}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b ext annotation (Placement(
        transformation(extent={{30,30},{50,50}}), iconTransformation(extent={{-10,70},
            {10,90}})));
protected
  Modelica.Units.SI.SpecificHeatCapacity cm_vec[3] "Specific heat capacity as a function of SOC";
  Modelica.Units.SI.ThermalConductivity lambda_vec[3] "Thermal conductivity as a function of SOC";

equation

  // Cell material properties
  cm_vec = {
    Modelica.Math.Polynomials.evaluate(CellMat.poly_cm_SOC0, T_cell),
    Modelica.Math.Polynomials.evaluate(CellMat.poly_cm_SOC50, T_cell),
    Modelica.Math.Polynomials.evaluate(CellMat.poly_cm_SOC100, T_cell)};

  lambda_vec = {
    Modelica.Math.Polynomials.evaluate(CellMat.poly_lambda_SOC0, T_cell),
    Modelica.Math.Polynomials.evaluate(CellMat.poly_lambda_SOC50, T_cell),
    Modelica.Math.Polynomials.evaluate(CellMat.poly_lambda_SOC100, T_cell)};

  cm = Modelica.Math.Vectors.interpolate(CellMat.SOC_vec, cm_vec, SOC);
  lambda = Modelica.Math.Vectors.interpolate(CellMat.SOC_vec, lambda_vec, SOC);
  Cm = M_cell*cm/4;

  // Heat transfer
  Cm*der(T_cell) = int.Q_flow + ext.Q_flow "Energy balance";
  int.Q_flow = (lambda*2*pi*H_cell/4*(int.T - T_cell))/
    Modelica.Math.log((D_pin/2 + D_cell/2)/D_pin)
    "Heat conduction through the internal half-thickness";
  ext.Q_flow = (lambda*2*pi*H_cell/4*(ext.T - T_cell))/
    Modelica.Math.log(D_cell/(D_pin/2 + D_cell/2))
    "Heat conduction through the external half-thickness";

  // Sanity checks
  assert(SOC >= 0, "SOC must be higher or equal to 0%");
  assert(SOC <= 100, "SOC must be lower or equal to 100%");

initial equation
  if initOpt == ThermalManagement.Choices.InitOpt.steadyState then
    der(T_cell) = 0;
  elseif initOpt == ThermalManagement.Choices.InitOpt.fixedState then
    T_cell = Tstart;
  else
    // do nothing
  end if;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-40,-60},{40,20}},
          lineColor={0,0,0},
          startAngle=45,
          endAngle=135,
          closure=EllipseClosure.None,
          lineThickness=0.5),
        Ellipse(
          extent={{-100,-120},{100,80}},
          lineColor={0,0,0},
          startAngle=44,
          endAngle=136,
          closure=EllipseClosure.None,
          lineThickness=0.5),
        Line(
          points={{-72,50},{-28,8}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{28,8},{72,50}},
          color={0,0,0},
          thickness=0.5)}),                                      Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p><img src=\"modelica://ThermalManagement/Figures/CylindricalCell.png\"/></p>
<p>Assumptions:</p>
<p>1. Heat transfer is considered only in the radial direction</p>
<p><br>References:</p>
<p>[1] M. Macdonald. Early Design Stage Evaluation of Thermal Performance of Battery Heat Acuisition System of a Hybrid Electric Aircraft, 2020.</p>
</html>"));
end CylindricalCellPortion;
