within DynTherM.Components.HeatTransfer;
model WallRadiation "Model of incident and emitted thermal radiation"

  replaceable model Material=DynTherM.Materials.Paints.WhitePaint
    constrainedby DynTherM.Materials.Paints.BasePaint "Material choice" annotation (choicesAllMatching=true);
  Material Mat;
  outer DynTherM.Components.Environment environment "Environmental properties";
  parameter Modelica.Units.SI.Area A "Heat transfer surface";
  parameter Modelica.Units.SI.Angle csi
    "Tilt angle of the surface wrt horizontal";

  Real F_sky "View factor between the wall and the sky dome";
  Real F_ground "View factor between the object and the ground";
  Real beta "Coefficient splitting the heat exchange with the sky dome between sky and air radiation";
  Modelica.Units.SI.HeatFlowRate Q_absorbed
    "Heat flow rate absorbed by the object from the solar radiation";
  Modelica.Units.SI.HeatFlowRate Q_em_sky
    "Heat flow rate emitted by the object to the surrounding environment";
  Modelica.Units.SI.HeatFlowRate Q_em_air
    "Heat flow rate emitted by the object to the surrounding environment";
  Modelica.Units.SI.HeatFlowRate Q_em_ground
    "Heat flow rate emitted by the object to the surrounding environment";
  Modelica.Units.SI.HeatFlowRate Q_emitted
    "Heat flow rate emitted by the object to the surrounding environment";

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet
    annotation (Placement(transformation(extent={{-14,20},{14,48}})));
  CustomInterfaces.IrradiancePort outlet
    annotation (Placement(transformation(extent={{-14,-28},{14,0}})));
equation
  // Solar irradiative heat flux
  Q_absorbed = -A*Mat.abs*(outlet.E_tb + outlet.E_td + outlet.E_tr);

  // Long-wave (infrared) radiative heat flux
  F_sky = (1 + cos(csi))/2;
  F_ground = (1 - cos(csi))/2;
  beta = sqrt(F_sky);
  Q_em_sky = A*environment.sigma*Mat.eps*F_sky*beta*(inlet.T^4 - environment.T_sky^4);
  Q_em_air = A*environment.sigma*Mat.eps*F_sky*(1 - beta)*(inlet.T^4 - environment.T_amb^4);
  Q_em_ground = A*environment.sigma*Mat.eps*F_ground*(inlet.T^4 - environment.T_ground_corr^4);
  Q_emitted = Q_em_sky + Q_em_air + Q_em_ground;

  inlet.Q_flow = Q_absorbed + Q_emitted;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,20},{100,0}},
          fillColor={192,192,192},
          fillPattern=FillPattern.Backward),
        Line(points={{-60,-76},{-60,-8}}, color={238,46,47}),
        Line(points={{-30,-76},{-30,-8}}, color={238,46,47}),
        Line(points={{-60,-8},{-66,-18}}, color={238,46,47}),
        Line(points={{-60,-8},{-54,-18}}, color={238,46,47}),
        Line(points={{-30,-8},{-24,-18}}, color={238,46,47}),
        Line(points={{-30,-8},{-36,-18}}, color={238,46,47}),
        Line(points={{30,-76},{30,-8}},   color={238,46,47}),
        Line(points={{60,-76},{60,-8}},   color={238,46,47}),
        Line(points={{54,-66},{60,-76}}, color={238,46,47}),
        Line(points={{24,-66},{30,-76}}, color={238,46,47}),
        Line(points={{66,-66},{60,-76}}, color={238,46,47}),
        Line(points={{36,-66},{30,-76}}, color={238,46,47})}),   Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Reference:</p>
<p>[1]&nbsp;F.&nbsp;Goia,&nbsp;et&nbsp;al.&nbsp;-&nbsp;A&nbsp;Numerical&nbsp;Model&nbsp;to&nbsp;Evaluate&nbsp;the&nbsp;thermal&nbsp;Behaviour&nbsp;of&nbsp;PCM&nbsp;Glazing&nbsp;System&nbsp;Configurations,&nbsp;2012.</p>
</html>"));
end WallRadiation;
