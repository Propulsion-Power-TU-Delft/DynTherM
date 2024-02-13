within DynTherM.Systems.Aircraft.Subsystems;
model UpperFuselageHeatTransfer
  "Model of heat transfer from external environment to inner section of upper fuselage (without transparencies)"
  outer DynTherM.Components.Environment environment "Environmental properties";

  replaceable model HTC_int =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    "Internal convection correlation" annotation (choicesAllMatching=true);

  replaceable model HTC_ext =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
    "External convection correlation" annotation (choicesAllMatching=true);

  parameter Real coeff "Fraction of cylinder with active heat transfer";
  parameter Modelica.Units.SI.Length L_fuselage
    "Length of the fuselage cylindrical section";
  parameter Modelica.Units.SI.Length R_ext "External radius of the fuselage";
  parameter Modelica.Units.SI.Length t_fuselage "Overall fuselage thickness";
  parameter Modelica.Units.SI.Angle csi
    "Tilt angle of the surface wrt horizontal";
  parameter Modelica.Units.SI.Irradiance E_tb "Beam component of the clear-sky solar irradiance";
  parameter Modelica.Units.SI.Irradiance E_td "Diffuse component of the clear-sky solar irradiance";
  parameter Modelica.Units.SI.Irradiance E_tr "Ground reflected component of the clear-sky solar irradiance";
  parameter Modelica.Units.SI.Angle theta "Incidence angle";
  parameter Modelica.Units.SI.Temperature Tstart_fuselage
    "Fuselage temperature start value" annotation (Dialog(tab="Initialization"));

  final parameter Modelica.Units.SI.Length R_int=composite.interiorPanel_int.R_int
    "Internal radius of the fuselage";
  final parameter Modelica.Units.SI.Area A_int=coeff*2*environment.pi*
      L_fuselage*R_int "Internal fuselage area";
  final parameter Modelica.Units.SI.Area A_ext=coeff*2*environment.pi*
      L_fuselage*R_ext "External fuselage area";

  Components.HeatTransfer.ExternalConvection extConvection(A=A_ext,
    redeclare model HTC = HTC_ext)
    annotation (Placement(transformation(extent={{6,56},{34,28}})));
  Components.HeatTransfer.WallRadiation wallRadiation(
    A=A_ext,
    csi=csi)
    annotation (Placement(transformation(extent={{-34,56},{-6,28}})));
  Components.HeatTransfer.SolarRadiation solarRadiation(
    E_tb_fixed=E_tb,
    E_td_fixed=E_td,
    E_tr_fixed=E_tr,
    theta_fixed=theta,
    csi=csi)
    annotation (Placement(transformation(extent={{-40,92},{0,52}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatToInner annotation (
      Placement(transformation(extent={{-10,-110},{10,-90}}),
        iconTransformation(extent={{-10,-70},{10,-50}})));
  Components.HeatTransfer.InternalConvection intConvection(A=A_int,
   redeclare model HTC = HTC_int)
    annotation (Placement(transformation(extent={{-16,-24},{16,-56}})));

  FuselageSandwichStructure composite(
    coeff=coeff,
    L_fuselage=L_fuselage,
    R_ext=R_ext,
    t_tot=t_fuselage,
    L_window=0,
    H_window=0,
    Nw_side=0,
    Tstart=Tstart_fuselage)
    annotation (Placement(transformation(extent={{-26,-18},{26,20}})));
equation
  connect(solarRadiation.inlet, wallRadiation.outlet)
    annotation (Line(points={{-20,59.6},{-20,43.96}},  color={191,0,0}));
  connect(heatToInner, intConvection.inlet)
    annotation (Line(points={{0,-100},{0,-45.44}},     color={191,0,0}));
  connect(wallRadiation.inlet, composite.ext) annotation (Line(points={{-20,37.24},
          {-20,14.3},{0,14.3}}, color={191,0,0}));
  connect(extConvection.inlet, composite.ext)
    annotation (Line(points={{20,37.24},{20,14.3},{0,14.3}}, color={191,0,0}));
  connect(composite.int, intConvection.outlet)
    annotation (Line(points={{0,-12.3},{0,-34.56}}, color={191,0,0}));
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
end UpperFuselageHeatTransfer;
