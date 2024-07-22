within DynTherM.Components;
package Electrical

  model CylindricalCellPortion "Model of 1/4 portion of a cylindrical cell"

    replaceable model CellMat=DynTherM.Materials.NCA18650 constrainedby
      DynTherM.Materials.CellProperties "Material properties of the prescribed cell type" annotation (choicesAllMatching=true);

    parameter Real SOC "State-of-charge [%]";
    parameter Length H_cell "Height of the cell";
    parameter Length D_cell "Diameter of the cell";
    parameter Length D_pin "Diameter of the central pin";
    parameter Mass M_cell "Weight of the cell portion";
    constant Real pi=Modelica.Constants.pi;

    // Initialization
    parameter Temperature Tstart "Temperature start value" annotation (Dialog(tab="Initialization"));
    parameter DynTherM.Choices.InitOpt initOpt "Initialization option"
      annotation (Dialog(tab="Initialization"));

    SpecificHeatCapacity cm "Specific heat capacity at the prescribed SOC";
    ThermalConductivity lambda "Thermal conductivity at the prescribed SOC";
    HeatCapacity Cm "Heat capacity of the cell portion";
    Temperature T_cell(start=Tstart) "Average temperature of the cell portion";

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a int annotation (Placement(
          transformation(extent={{-10,10},{10,30}}),  iconTransformation(extent={{-10,10},
              {10,30}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b ext annotation (Placement(
          transformation(extent={{30,30},{50,50}}), iconTransformation(extent={{-10,70},
              {10,90}})));
  protected
    SpecificHeatCapacity cm_vec[3] "Specific heat capacity as a function of SOC";
    ThermalConductivity lambda_vec[3] "Thermal conductivity as a function of SOC";

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
    if initOpt == DynTherM.Choices.InitOpt.steadyState then
      der(T_cell) = 0;
    elseif initOpt == DynTherM.Choices.InitOpt.fixedState then
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

  model CylindricalCell

    replaceable model CellMat=DynTherM.Materials.NCA18650 constrainedby
      DynTherM.Materials.CellProperties "Material properties of the prescribed cell type" annotation (choicesAllMatching=true);

    parameter Real SOC "State-of-charge [%]";
    parameter Length H_cell "Height of the cell";
    parameter Length D_cell "Diameter of the cell";
    parameter Length D_pin "Diameter of the central pin";
    parameter Mass M_cell "Weight of the cell portion";
    parameter Modelica.Units.NonSI.Energy_kWh E_cell "Energy stored in the cell (SOC = 100%)";
    parameter DynTherM.CustomUnits.VolumetricHeatFlowRate Q_vol
      "Volumetric heat flow rate";
    final parameter Volume V_cell = Modelica.Constants.pi*D_cell^2/4*H_cell;

    // Initialization
    parameter Temperature Tstart "Temperature start value" annotation (Dialog(tab="Initialization"));
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
    connect(W, westPortion.ext) annotation (Line(points={{-102,-2},{-88,-2},{-88,
            3.10862e-15},{-76,3.10862e-15}},
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

  model Cell18650
    extends CylindricalCell(
      H_cell=0.065,
      D_cell=0.0184,
      D_pin=0.0025,
      E_cell=11.25,
      M_cell=45e-3,
      Q_vol=84.5e3);
  end Cell18650;

  model RC1 "First order equivalent circuit model"
    parameter ElectricCharge C_nom "Nominal capacity";
    parameter Real eta "Charging/discharging efficiency";
    parameter Real SoC_start(start = 1) "Start Condition" annotation (Dialog(tab="Initialization"));

    HeatFlowRate Q_rev "Reversible heat generation";
    HeatFlowRate Q_irrev "Irreversible heat generation";

    Modelica.Electrical.Analog.Sources.SignalVoltage sourceOCV annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-30,0})));
    Modelica.Electrical.Analog.Basic.VariableResistor R0
      annotation (Placement(transformation(extent={{0,-10},{20,10}})));
    Modelica.Electrical.Analog.Basic.VariableResistor R1
      annotation (Placement(transformation(extent={{50,10},{70,30}})));
    Modelica.Electrical.Analog.Basic.VariableCapacitor C1
      annotation (Placement(transformation(extent={{50,-10},{70,-30}})));
    Modelica.Blocks.Tables.CombiTable1Ds Entropic_interpolation(table=[0,-0.22; 0.10,
          -0.3; 0.20,-0.1; 0.30,-0.05; 0.40,0.11; 0.50,0.12; 0.60,0.06; 0.70,-0.02;
          0.80,-0.02; 0.90,0.015; 1.00,0.025])
      annotation (Placement(transformation(extent={{-80,-120},{-60,-100}})));
    Modelica.Blocks.Tables.CombiTable2Ds R0_interpolation(table=[0,283.15,298.15,318.15;
          0,0.002172727,0.001354545,0.000872728; 0.10,0.002172727,0.001227273,0.000854545;
          0.20,0.001845455,0.001163636,0.000809091; 0.30,0.001736364,0.001090909,0.000772727;
          0.40,0.001863636,0.001081818,0.000745455; 0.50,0.001718182,0.001090909,0.000754545;
          0.60,0.001690909,0.0011,0.000772727; 0.70,0.001654545,0.001054545,0.000754545;
          0.80,0.001781818,0.001045455,0.000736364; 0.90,0.0017,0.0011,0.000754545;
          1.00,0.0017,0.0011,0.000754545], smoothness=Modelica.Blocks.Types.Smoothness.ContinuousDerivative)
      annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=-90,
          origin={10,70})));
    Modelica.Blocks.Tables.CombiTable2Ds OCV_interpolation(table=[0,283.15,298.15,
          318.15; 0,3.401,3.395,3.394; 0.10,3.486,3.479,3.477; 0.20,3.566,3.559,3.554;
          0.30,3.607,3.608,3.609; 0.40,3.641,3.642,3.644; 0.50,3.698,3.697,3.7; 0.60,
          3.809,3.81,3.811; 0.70,3.909,3.911,3.912; 0.80,4.012,4.013,4.014; 0.90,4.118,
          4.121,4.121; 1.00,4.118,4.121,4.121], smoothness=Modelica.Blocks.Types.Smoothness.ContinuousDerivative)
      annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=-90,
          origin={-30,70})));
    Modelica.Blocks.Tables.CombiTable2Ds R1_interpolation(table=[0,283.15,298.15,318.15;
          0,0.001087396,0.001136109,0.000903105; 0.10,0.001012533,0.001027663,0.000897299;
          0.20,0.001057631,0.000909021,0.000790583; 0.30,0.000983819,0.000847077,0.000740259;
          0.40,0.000925584,0.000899386,0.000832152; 0.50,0.001297334,0.001115926,0.000994332;
          0.60,0.001304479,0.001093345,0.000923337; 0.70,0.001288013,0.001092672,0.000890758;
          0.80,0.000989563,0.001147979,0.000936039; 0.90,0.001262494,0.001067136,0.000636587;
          1.00,0.001262494,0.001067136,0.000636587], smoothness=Modelica.Blocks.Types.Smoothness.ContinuousDerivative)
      annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=-90,
          origin={60,70})));
    Modelica.Blocks.Tables.CombiTable2Ds C1_interpolation(table=[0,283.15,298.15,318.15;
          0,27588.84528,26405.91704,33218.72872; 0.10,29628.66395,29192.44928,33433.6715;
          0.20,28365.28052,33002.5379,37946.67986; 0.30,30493.41393,35415.9067,40526.35632;
          0.40,32411.96909,33356.0896,36051.10605; 0.50,23124.34577,26883.50303,30171.00928;
          0.60,22997.6872,27438.7316,32490.8457; 0.70,23291.69038,27455.6317,33679.1811;
          0.80,30316.4124,26132.88222,32049.94664; 0.90,23762.48917,28112.63044,47126.31581;
          1.00,23762.48917,28112.63044,47126.31581], smoothness=Modelica.Blocks.Types.Smoothness.ContinuousDerivative)
      annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=90,
          origin={60,-60})));
    Utilities.StateOfCharge stateOfCharge(C_nom=C_nom,
      eta=eta,                                         SoC_start=SoC_start)
                                          annotation (Placement(transformation(
          extent={{-12,12},{12,-12}},
          rotation=180,
          origin={-80,-40})));
    Modelica.Blocks.Interfaces.RealInput T annotation (Placement(transformation(
            extent={{-12,-12},{12,12}},
          rotation=-90,
          origin={0,126}),                  iconTransformation(extent={{-132,70},{
              -108,94}})));
    Modelica.Blocks.Math.Gain gain(k=1/1000)
      annotation (Placement(transformation(extent={{-38,-118},{-22,-102}})));
    Modelica.Blocks.Interfaces.RealOutput Q "Internal heat generation"
      annotation (Placement(transformation(
          extent={{14,-14},{-14,14}},
          rotation=180,
          origin={8,-110}),  iconTransformation(extent={{-12,-12},{12,12}},
          rotation=-90,
          origin={20,-100})));

    Modelica.Electrical.Analog.Interfaces.PositivePin p annotation (Placement(
          transformation(extent={{-130,-10},{-110,10}}), iconTransformation(
            extent={{-92,18},{-74,36}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin n annotation (Placement(
          transformation(extent={{110,-10},{130,10}}), iconTransformation(extent=
              {{-92,-86},{-74,-68}})));

    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
      annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
    Modelica.Blocks.Interfaces.RealOutput SoC "State of charge" annotation (
        Placement(transformation(
          extent={{-14,-14},{14,14}},
          rotation=180,
          origin={-128,-40}), iconTransformation(
          extent={{-12,-12},{12,12}},
          rotation=-90,
          origin={-20,-100})));
  equation
    Q = Q_rev + Q_irrev;
    Q_rev = currentSensor.p.i*T*gain.y;
    Q_irrev = R0.LossPower + R1.LossPower;

    connect(R0.n, R1.p)
      annotation (Line(points={{20,0},{40,0},{40,20},{50,20}}, color={0,0,255}));
    connect(R0.n, C1.p) annotation (Line(points={{20,0},{40,0},{40,-20},{50,-20}},
          color={0,0,255}));
    connect(OCV_interpolation.y, sourceOCV.v)
      annotation (Line(points={{-30,59},{-30,12}}, color={0,0,127}));
    connect(R0_interpolation.y, R0.R)
      annotation (Line(points={{10,59},{10,12}}, color={0,0,127}));
    connect(R1_interpolation.y, R1.R)
      annotation (Line(points={{60,59},{60,32}}, color={0,0,127}));
    connect(C1_interpolation.y, C1.C)
      annotation (Line(points={{60,-49},{60,-32}}, color={0,0,127}));
    connect(T, OCV_interpolation.u2) annotation (Line(points={{0,126},{0,100},{-24,
            100},{-24,82}},color={0,0,127}));
    connect(T, R0_interpolation.u2)
      annotation (Line(points={{0,126},{0,100},{16,100},{16,82}},
                                                              color={0,0,127}));
    connect(T, R1_interpolation.u2) annotation (Line(points={{0,126},{0,100},{66,
            100},{66,82}},
                      color={0,0,127}));
    connect(T, C1_interpolation.u2) annotation (Line(points={{0,126},{0,100},{-10,
            100},{-10,-80},{54,-80},{54,-72}},
                                           color={0,0,127}));
    connect(Entropic_interpolation.y[1], gain.u)
      annotation (Line(points={{-59,-110},{-39.6,-110}},
                                                      color={0,0,127}));
    connect(R1.n, n) annotation (Line(points={{70,20},{80,20},{80,0},{120,0}},
          color={0,0,255}));
    connect(C1.n, n) annotation (Line(points={{70,-20},{80,-20},{80,0},{120,0}},
          color={0,0,255}));
    connect(stateOfCharge.SoC, OCV_interpolation.u1) annotation (Line(points={{-92.24,
            -40},{-100,-40},{-100,90},{-36,90},{-36,82}}, color={0,0,127}));
    connect(stateOfCharge.SoC, R0_interpolation.u1) annotation (Line(points={{-92.24,
            -40},{-100,-40},{-100,90},{4,90},{4,82}}, color={0,0,127}));
    connect(stateOfCharge.SoC, R1_interpolation.u1) annotation (Line(points={{-92.24,
            -40},{-100,-40},{-100,90},{54,90},{54,82}}, color={0,0,127}));
    connect(stateOfCharge.SoC, C1_interpolation.u1) annotation (Line(points={{-92.24,
            -40},{-100,-40},{-100,-90},{66,-90},{66,-72}}, color={0,0,127}));
    connect(stateOfCharge.SoC, Entropic_interpolation.u) annotation (Line(points={{-92.24,
            -40},{-100,-40},{-100,-110},{-82,-110}},       color={0,0,127}));
    connect(sourceOCV.n, R0.p)
      annotation (Line(points={{-20,0},{0,0}}, color={0,0,255}));
    connect(currentSensor.i, stateOfCharge.I)
      annotation (Line(points={{-60,-11},{-60,-40},{-68,-40}}, color={0,0,127}));
    connect(p, currentSensor.p)
      annotation (Line(points={{-120,0},{-70,0}}, color={0,0,255}));
    connect(currentSensor.n, sourceOCV.p)
      annotation (Line(points={{-50,0},{-40,0}}, color={0,0,255}));
    connect(stateOfCharge.SoC, SoC)
      annotation (Line(points={{-92.24,-40},{-128,-40}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},
              {120,120}}), graphics={Bitmap(
            extent={{-118,-120},{118,116}},
            imageSource="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"
                 + "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",
            fileName="modelica://DynTherM/Figures/RC1.PNG")}),     Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},{120,120}})));
  end RC1;

  model PouchCell1D
    "Electro-thermal model of a pouch cell featuring 1D discretization"

    replaceable model InPlaneMat = Materials.PolestarCellInPlane constrainedby
      Materials.Properties "In-plane material properties"
      annotation (choicesAllMatching=true);

    replaceable model CrossPlaneMat = Materials.PolestarCellCrossPlane constrainedby
      Materials.Properties "Cross-plane material properties"
      annotation (choicesAllMatching=true);

    // Geometry
    parameter Length W "Width" annotation (Dialog(tab="Geometry"));
    parameter Length H "Height" annotation (Dialog(tab="Geometry"));
    parameter Length t "Thickness" annotation (Dialog(tab="Geometry"));

    // Electrical parameters
    parameter Real eta=0.98 "Charging/discharging efficiency";
    parameter ElectricCharge C_nom "Nominal capacity";

    // Initialization
    parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
      "Initialization option" annotation (Dialog(tab="Initialization"));
    parameter Real SoC_start "Starting state of charge" annotation (Dialog(tab="Initialization"));
    parameter Temperature Tstart=288.15 "Temperature start value" annotation (Dialog(tab="Initialization"));

    // Discretization
    parameter Integer N(min=1)=10 "Number of vertical sections in which the cell is discretized";

    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor annotation (Placement(transformation(extent={{-14,34},
              {-26,46}})));
    Components.OneDimensional.PouchCellThermal1D thermal(
      redeclare model Mat = InPlaneMat,
      A=W*t,
      H=H,
      Tstart=Tstart,
      initOpt=initOpt,
      N=N) annotation (Placement(transformation(extent={{22,-28},{80,28}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a Top annotation (Placement(
          transformation(extent={{44,34},{56,46}}), iconTransformation(extent={{-16,40},
              {-4,52}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a Bottom annotation (
        Placement(transformation(extent={{44,-46},{56,-34}}),
                                                           iconTransformation(
            extent={{-16,-54},{-4,-42}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow prescribedHeatFlow annotation (Placement(transformation(
          extent={{-6,6},{6,-6}},
          rotation=0,
          origin={20,-50})));
    RC1 electrical(
      C_nom=C_nom,
      eta=eta,
      SoC_start=SoC_start)
      annotation (Placement(transformation(extent={{-68,-20},{-14,34}})));

    CustomInterfaces.DistributedHeatPort_A Left(Nx=N, Ny=1) annotation (Placement(
          transformation(
          extent={{-10,-5},{10,5}},
          rotation=90,
          origin={5,20}), iconTransformation(
          extent={{-30,-9},{30,9}},
          rotation=90,
          origin={-63,0})));
    CustomInterfaces.DistributedHeatPort_A Right(Nx=N, Ny=1) annotation (
        Placement(transformation(
          extent={{-10,-5},{10,5}},
          rotation=90,
          origin={5,-20}), iconTransformation(
          extent={{-30,-9},{30,9}},
          rotation=90,
          origin={45,0})));
    TwoDimensional.WallConductionDiscretized cross_plane_conduction_left(
      redeclare model Mat = CrossPlaneMat,
      t=t/2,
      A=W*H,
      Tstart=Tstart,
      initOpt=initOpt,
      Nx=N,
      Ny=1) annotation (Placement(transformation(
          extent={{-10,-9},{10,9}},
          rotation=-90,
          origin={13,20})));
    TwoDimensional.WallConductionDiscretized cross_plane_conduction_right(
      redeclare model Mat = CrossPlaneMat,
      t=t/2,
      A=W*H,
      Tstart=Tstart,
      initOpt=initOpt,
      Nx=N,
      Ny=1) annotation (Placement(transformation(
          extent={{-10,-9},{10,9}},
          rotation=-90,
          origin={13,-20})));
    Modelica.Electrical.Analog.Interfaces.PositivePin p annotation (Placement(
          transformation(extent={{-100,6},{-86,20}}),    iconTransformation(
            extent={{56,14},{66,24}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin n annotation (Placement(
          transformation(extent={{-86,-28},{-100,-14}}),iconTransformation(extent=
             {{56,-26},{66,-16}})));
    Modelica.Electrical.Batteries.Interfaces.CellBus cellBus annotation (
        Placement(transformation(extent={{-70,-54},{-42,-26}}),
          iconTransformation(extent={{-22,-12},{2,12}})));
    Modelica.Electrical.Analog.Sensors.MultiSensor multiSensor
      annotation (Placement(transformation(extent={{-80,8},{-70,18}})));
  equation

    connect(Top,thermal. Top) annotation (Line(points={{50,40},{50.42,40},{50.42,17.36}},
                     color={191,0,0}));
    connect(Bottom,thermal. Bottom) annotation (Line(points={{50,-40},{50,-18.68},
            {50.42,-18.68},{50.42,-17.36}}, color={191,0,0}));
    connect(temperatureSensor.T, electrical.T) annotation (Line(points={{-26.6,40},
            {-86,40},{-86,25.45},{-68,25.45}},       color={0,0,127}));
    connect(temperatureSensor.port, thermal.Average) annotation (Line(points={{-14,40},
            {40,40},{40,0},{50.42,0}},     color={191,0,0}));
    connect(prescribedHeatFlow.port, thermal.Average) annotation (Line(points={{26,-50},
            {40,-50},{40,0},{50.42,0}},                          color={191,0,0}));
    connect(Left, cross_plane_conduction_left.outlet)
      annotation (Line(points={{5,20},{10.3,20}}, color={191,0,0}));
    connect(Right, cross_plane_conduction_right.outlet)
      annotation (Line(points={{5,-20},{10.3,-20}}, color={191,0,0}));
    connect(cross_plane_conduction_left.inlet, thermal.Distributed) annotation (
        Line(points={{15.7,20},{20,20},{20,0},{26.35,0}}, color={191,0,0}));
    connect(cross_plane_conduction_right.inlet, thermal.Distributed) annotation (
        Line(points={{15.7,-20},{20,-20},{20,0},{26.35,0},{26.35,3.55271e-15}},
          color={191,0,0}));
    connect(temperatureSensor.T, cellBus.T) annotation (Line(points={{-26.6,40},{
            -86,40},{-86,-40},{-60,-40},{-60,-39.93},{-55.93,-39.93}}, color={0,0,
            127}), Text(
        string="%second",
        index=1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(multiSensor.nc, electrical.p) annotation (Line(points={{-70,13},{-70,
            13.075},{-59.675,13.075}}, color={0,0,255}));
    connect(multiSensor.pc, multiSensor.pv)
      annotation (Line(points={{-80,13},{-80,18},{-75,18}}, color={0,0,255}));
    connect(multiSensor.v, cellBus.v) annotation (Line(points={{-72,7.5},{-72,
            -39.93},{-55.93,-39.93}}, color={0,0,127}), Text(
        string="%second",
        index=1,
        extent={{-3,-6},{-3,-6}},
        horizontalAlignment=TextAlignment.Right));
    connect(multiSensor.i, cellBus.i) annotation (Line(points={{-78,7.5},{-78,
            -39.93},{-55.93,-39.93}}, color={0,0,127}), Text(
        string="%second",
        index=1,
        extent={{-3,-6},{-3,-6}},
        horizontalAlignment=TextAlignment.Right));
    connect(multiSensor.power, cellBus.power) annotation (Line(points={{-80.5,10},
            {-82,10},{-82,-39.93},{-55.93,-39.93}}, color={0,0,127}), Text(
        string="%second",
        index=1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(electrical.Q, cellBus.lossPower) annotation (Line(points={{-36.5,
            -15.5},{-36.5,-39.93},{-55.93,-39.93}}, color={0,0,127}), Text(
        string="%second",
        index=1,
        extent={{-3,-6},{-3,-6}},
        horizontalAlignment=TextAlignment.Right));
    connect(electrical.SoC, cellBus.soc) annotation (Line(points={{-45.5,-15.5},{
            -45.5,-39.93},{-55.93,-39.93}}, color={0,0,127}), Text(
        string="%second",
        index=1,
        extent={{-3,-6},{-3,-6}},
        horizontalAlignment=TextAlignment.Right));
    connect(electrical.Q, prescribedHeatFlow.Q_flow) annotation (Line(points={{
            -36.5,-15.5},{-36.5,-50},{14,-50}}, color={0,0,127}));
    connect(p, multiSensor.pc)
      annotation (Line(points={{-93,13},{-80,13}}, color={0,0,255}));
    connect(n, electrical.n) annotation (Line(points={{-93,-21},{-59.675,-21},{
            -59.675,-10.325}}, color={0,0,255}));
    connect(n, multiSensor.nv)
      annotation (Line(points={{-93,-21},{-75,-21},{-75,8}}, color={0,0,255}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-60},{80,60}}),
                                                        graphics={Bitmap(
              extent={{-79,-66},{79,66}},
            imageSource=
                "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"
                 +
                "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",
            fileName="modelica://DynTherM/Figures/PouchCell.PNG",
            origin={4,-1},
            rotation=-90)}),
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-60},{80,60}})),
      experiment(StopTime=1200, __Dymola_Algorithm="Dassl"),
                Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p>Heat conduction is modelled both in-plane and cross-plane.</p>
<p><br><img src=\"modelica://DynTherM/Figures/pouch_cell_thermal_1D.png\"/></p>
</html>"));
  end PouchCell1D;
  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),
    Rectangle(
      origin={20.3125,82.8571},
      extent={{-45.3125,-57.8571},{4.6875,-27.8571}}),
    Line(
      origin={8,48},
      points={{32.0,-58.0},{72.0,-58.0}}),
    Line(
      origin={9,54},
      points={{31.0,-49.0},{71.0,-49.0}}),
    Line(
      origin={-2,55},
      points={{-83.0,-50.0},{-33.0,-50.0}}),
    Line(
      origin={-3,45},
      points={{-72.0,-55.0},{-42.0,-55.0}}),
    Line(
      origin={1,50},
      points={{-61.0,-45.0},{-61.0,-10.0},{-26.0,-10.0}}),
    Line(
      origin={7,50},
      points={{18.0,-10.0},{53.0,-10.0},{53.0,-45.0}}),
    Line(
      origin={6.2593,48},
      points={{53.7407,-58.0},{53.7407,-93.0},{-66.2593,-93.0},{-66.2593,-58.0}})}));
end Electrical;
