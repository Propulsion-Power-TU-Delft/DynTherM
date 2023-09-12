within DynTherM.Components.Battery;
model CylindricalCell

  replaceable model CellMat=DynTherM.Materials.NCA18650 constrainedby
    DynTherM.Materials.CellProperties          "Material properties of the prescribed cell type" annotation (choicesAllMatching=true);

  parameter Real SOC "State-of-charge [%]";
  parameter Modelica.Units.SI.Length H_cell "Height of the cell";
  parameter Modelica.Units.SI.Length D_cell "Diameter of the cell";
  parameter Modelica.Units.SI.Length D_pin "Diameter of the central pin";
  parameter Modelica.Units.SI.Mass M_cell "Weight of the cell portion";
  parameter Modelica.Units.NonSI.Energy_kWh E_cell "Energy stored in the cell (SOC = 100%)";
  parameter DynTherM.CustomUnits.VolumetricHeatFlowRate Q_vol
    "Volumetric heat flow rate";
  final parameter Modelica.Units.SI.Volume V_cell = Modelica.Constants.pi*D_cell^2/4*H_cell;

  // Initialization
  parameter Modelica.Units.SI.Temperature Tstart "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter DynTherM.Choices.InitOpt initOpt "Initialization option"
    annotation (Dialog(tab="Initialization"));

  CylindricalCellPortion northPortion(
    redeclare model CellMat = CellMat,
    SOC=SOC,
    H_cell=H_cell,
    D_cell=D_cell,
    D_pin=D_pin,
    M_cell=M_cell,
    Tstart=Tstart,
    initOpt=initOpt)
    annotation (Placement(transformation(extent={{-60,-32},{60,88}})));
  CylindricalCellPortion southPortion(
    redeclare model CellMat = CellMat,
    SOC=SOC,
    H_cell=H_cell,
    D_cell=D_cell,
    D_pin=D_pin,
    M_cell=M_cell,
    Tstart=Tstart,
    initOpt=initOpt)
    annotation (Placement(transformation(extent={{-60,32},{60,-88}})));
  CylindricalCellPortion eastPortion(
    redeclare model CellMat = CellMat,
    SOC=SOC,
    H_cell=H_cell,
    D_cell=D_cell,
    D_pin=D_pin,
    M_cell=M_cell,
    Tstart=Tstart,
    initOpt=initOpt)
    annotation (Placement(transformation(
        extent={{-60,-60},{60,60}},
        rotation=-90,
        origin={28,0})));
  CylindricalCellPortion westPortion(
    redeclare model CellMat = CellMat,
    SOC=SOC,
    H_cell=H_cell,
    D_cell=D_cell,
    D_pin=D_pin,
    M_cell=M_cell,
    Tstart=Tstart,
    initOpt=initOpt)
    annotation (Placement(transformation(
        extent={{-60,-60},{60,60}},
        rotation=90,
        origin={-28,0})));
  BoundaryConditions.thermal internal(
    Q=-Q_vol*V_cell,
    use_Q=true,
    use_T=false)
    annotation (Placement(transformation(extent={{-24,-12},{12,12}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b E annotation (Placement(
        transformation(extent={{88,-12},{112,12}}), iconTransformation(extent={{90,-10},
            {110,10}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b N annotation (Placement(
        transformation(extent={{-14,86},{10,110}}), iconTransformation(extent={{-10,90},
            {10,110}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b W annotation (Placement(
        transformation(extent={{-114,-14},{-90,10}}), iconTransformation(extent={{-110,
            -10},{-90,10}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b S annotation (Placement(
        transformation(extent={{-12,-88},{12,-112}}), iconTransformation(extent={{-10,
            -110},{10,-90}})));

equation
  connect(internal.thermal, eastPortion.int) annotation (Line(points={{0,0},{20,
          0},{20,-2.22045e-15},{40,-2.22045e-15}}, color={191,0,0}));
  connect(internal.thermal, northPortion.int)
    annotation (Line(points={{0,0},{0,40}}, color={191,0,0}));
  connect(internal.thermal, southPortion.int)
    annotation (Line(points={{0,0},{0,-40}}, color={191,0,0}));
  connect(internal.thermal, westPortion.int) annotation (Line(points={{0,0},{-20,
          0},{-20,4.44089e-16},{-40,4.44089e-16}}, color={191,0,0}));
  connect(eastPortion.ext, E) annotation (Line(points={{76,-8.88178e-15},{88,-8.88178e-15},
          {88,0},{100,0}}, color={191,0,0}));
  connect(S, southPortion.ext)
    annotation (Line(points={{0,-100},{0,-76}}, color={191,0,0}));
  connect(W, westPortion.ext) annotation (Line(points={{-102,-2},{-88,-2},{
          -88,3.10862e-15},{-76,3.10862e-15}},
                              color={191,0,0}));
  connect(N, northPortion.ext)
    annotation (Line(points={{-2,98},{-2,88},{0,88},{0,76}},
                                              color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid), Ellipse(
          extent={{-64,64},{66,-66}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid)}),                      Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end CylindricalCell;
