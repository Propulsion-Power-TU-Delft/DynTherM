within DynTherM.Systems.HydrogenTank.Subsystems;
model CylinderHeatTransfer
  "Model of heat transfer from external environment through the fuselage to inner section of the tank"
  outer DynTherM.Components.Environment environment "Environmental properties";

  replaceable model HTC_ext =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClasses.BaseClassExternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClasses.BaseClassExternal
    "External convection correlation" annotation (choicesAllMatching=true);

  parameter Real coeff "Fraction of cylinder with active heat transfer";
  parameter Length L_tank "Length of the tank cylindrical section";
  parameter Length R_ext "External radius of the tank";
  parameter Length t_tank "Overall fuselage/tank thickness";
  parameter Length t_insulation "Overall insulation thickness";
  parameter Angle csi "Tilt angle of the surface wrt horizontal";
  parameter Irradiance E_tb "Beam component of the clear-sky solar irradiance";
  parameter Irradiance E_td "Diffuse component of the clear-sky solar irradiance";
  parameter Irradiance E_tr "Ground reflected component of the clear-sky solar irradiance";
  parameter Angle theta "Incidence angle";
  final parameter Mass m=fuselage.m + insulation.m "Mass";

  parameter Temperature T_start_tank
    "Fuselage/tank temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature T_start_insulation
    "Insulation temperature start value" annotation (Dialog(tab="Initialization"));

  final parameter Area A_ext=coeff*2*environment.pi*
      L_tank*R_ext "External tank area";

  Components.HeatTransfer.ExternalConvection extConvection(A=A_ext,
    redeclare model HTC = HTC_ext)
    annotation (Placement(transformation(extent={{16,66},{64,18}})));
  Components.HeatTransfer.WallRadiation wallRadiation(
    A=A_ext,
    csi=csi)
    annotation (Placement(transformation(extent={{-64,66},{-16,18}})));
  Components.HeatTransfer.SolarRadiation solarRadiation(
    E_tb_fixed=E_tb,
    E_td_fixed=E_td,
    E_tr_fixed=E_tr,
    theta_fixed=theta,
    csi=csi)
    annotation (Placement(transformation(extent={{-62,96},{-18,52}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatToInner annotation (
      Placement(transformation(extent={{-10,-110},{10,-90}}),
        iconTransformation(extent={{-10,-70},{10,-50}})));

  Components.HeatTransfer.TubeConduction fuselage(
    redeclare model Mat = DynTherM.Materials.Aluminium7075T6,
    coeff=coeff,
    L=L_tank,
    R_ext=R_ext,
    R_int=R_ext - t_tank,
    Tstart=T_start_tank,
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{-30,30},{30,-30}})));
  Components.HeatTransfer.TubeConduction insulation(
    redeclare model Mat = DynTherM.Materials.NFRSB31,
    coeff=coeff,
    L=L_tank,
    R_ext=R_ext - t_tank,
    R_int=R_ext - t_tank - t_insulation,
    Tstart=T_start_insulation,
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{-30,-20},{30,-80}})));
equation
  connect(solarRadiation.inlet, wallRadiation.outlet)
    annotation (Line(points={{-40,60.36},{-40,45.36}}, color={191,0,0}));
  connect(fuselage.inlet, insulation.outlet)
    annotation (Line(points={{0,-10.2},{0,-39.8}}, color={191,0,0}));
  connect(insulation.inlet, heatToInner)
    annotation (Line(points={{0,-60.2},{0,-100}}, color={191,0,0}));
  connect(wallRadiation.inlet, fuselage.outlet) annotation (Line(points={{-40,33.84},
          {-40,10.2},{0,10.2}}, color={191,0,0}));
  connect(fuselage.outlet, extConvection.inlet) annotation (Line(points={{4.44089e-16,
          10.2},{40,10.2},{40,33.84}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-100,-180},{100,20}},
          lineColor={0,0,0},
          startAngle=0,
          endAngle=180,
          closure=EllipseClosure.None,
          lineThickness=0.5),
        Ellipse(
          extent={{-80,-160},{80,0}},
          lineColor={0,0,0},
          startAngle=0,
          endAngle=180,
          closure=EllipseClosure.None,
          lineThickness=0.5),
        Line(
          points={{-100,-80},{-80,-80}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{80,-80},{100,-80}},
          color={0,0,0},
          thickness=0.5),
        Ellipse(
          extent={{-70,-150},{68,-12}},
          lineColor={28,108,200},
          startAngle=0,
          endAngle=80,
          closure=EllipseClosure.None,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{70,-150},{-68,-12}},
          lineColor={28,108,200},
          startAngle=0,
          endAngle=80,
          closure=EllipseClosure.None,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(points={{-10,-12},{-16,-18}}, color={28,108,200}),
        Line(points={{-10,-12},{-18,-10}}, color={28,108,200}),
        Line(points={{10,-12},{16,-18}}, color={28,108,200}),
        Line(points={{10,-12},{18,-10}}, color={28,108,200}),
        Line(points={{-20,24},{-20,80}},  color={238,46,47}),
        Line(points={{-14,34},{-20,24}},  color={238,46,47}),
        Line(points={{-26,34},{-20,24}},  color={238,46,47}),
        Line(points={{-40,18},{-40,74}},  color={238,46,47}),
        Line(points={{-34,28},{-40,18}},  color={238,46,47}),
        Line(points={{-46,28},{-40,18}},  color={238,46,47}),
        Line(points={{20,24},{20,80}},    color={238,46,47}),
        Line(points={{26,34},{20,24}},    color={238,46,47}),
        Line(points={{14,34},{20,24}},    color={238,46,47}),
        Line(points={{40,18},{40,74}},    color={238,46,47}),
        Line(points={{46,28},{40,18}},    color={238,46,47}),
        Line(points={{34,28},{40,18}},    color={238,46,47}),
        Text(
          extent={{-30,-16},{28,-46}},
          lineColor={0,0,0},
          lineThickness=0.5,
          textString="FUSELAGE")}),                              Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end CylinderHeatTransfer;
