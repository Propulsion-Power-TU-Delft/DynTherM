within DynTherM.Components;
package ThreeDimensional

  model WallConductionCV3D
    "Control volume dynamic model of 3D heat conduction in a planar surface"
    replaceable model MatX=Materials.Aluminium constrainedby Materials.Properties
      "Material properties along x direction" annotation (choicesAllMatching=true);
    replaceable model MatY=Materials.Aluminium constrainedby Materials.Properties
      "Material properties along y direction" annotation (choicesAllMatching=true);
    replaceable model MatZ=Materials.Aluminium constrainedby Materials.Properties
      "Material properties along z direction" annotation (choicesAllMatching=true);

    input Length x "Distance between east and west ports" annotation (Dialog(enable=true));
    input Length y "Distance between north and south ports" annotation (Dialog(enable=true));
    input Length z "Distance between front and rear ports" annotation (Dialog(enable=true));
    input Mass dm(start=0) "Mass variation over time" annotation (Dialog(enable=true));

    // Initialization
    parameter Temperature Tstart=300 "Temperature start value" annotation (Dialog(tab="Initialization"));
    parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
      "Initialization option" annotation (Dialog(tab="Initialization"));

    Mass m "Mass";
    Volume V "Volume";
    HeatCapacity Cm "Heat capacity";
    Temperature T_vol "Average temperature";

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a North annotation (
        Placement(transformation(extent={{-6,14},{6,26}}),   iconTransformation(
            extent={{-6,14},{6,26}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b West annotation (
        Placement(transformation(extent={{-66,-16},{-54,-4}}),
          iconTransformation(extent={{-66,-16},{-54,-4}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b South annotation (
        Placement(transformation(extent={{-6,-46},{6,-34}}),   iconTransformation(
            extent={{-6,-46},{6,-34}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a East annotation (
        Placement(transformation(extent={{54,-16},{66,-4}}),  iconTransformation(
            extent={{54,-16},{66,-4}})));
    Modelica.Blocks.Interfaces.RealInput Q_int annotation (Placement(
          transformation(extent={{-68,36},{-28,76}}), iconTransformation(
          extent={{-8,-8},{8,8}},
          rotation=-90,
          origin={40,60})));

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a Front annotation (
        Placement(transformation(extent={{-52,-32},{-30,-10}}),
          iconTransformation(extent={{-46,-56},{-34,-44}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b Rear annotation (
        Placement(transformation(extent={{10,10},{30,30}}), iconTransformation(
            extent={{34,24},{46,36}})));
  equation
    V = x*y*z;
    m = MatX.rho*V - dm;
    Cm = MatX.cm*m;

    Cm*der(T_vol) = North.Q_flow + South.Q_flow + East.Q_flow + West.Q_flow +
      Front.Q_flow + Rear.Q_flow + Q_int "Energy balance";

    North.Q_flow = (MatY.lambda*x*z*(North.T - T_vol))/(y/2) "Heat conduction through northern side";
    South.Q_flow = (MatY.lambda*x*z*(South.T - T_vol))/(y/2) "Heat conduction through the southern side";
    East.Q_flow = (MatX.lambda*y*z*(East.T - T_vol))/(x/2) "Heat conduction through the eastern side";
    West.Q_flow = (MatX.lambda*y*z*(West.T - T_vol))/(x/2) "Heat conduction through the western side";
    Front.Q_flow = (MatZ.lambda*x*y*(Front.T - T_vol))/(z/2) "Heat conduction through the front side";
    Rear.Q_flow = (MatZ.lambda*x*y*(Rear.T - T_vol))/(z/2) "Heat conduction through the rear side";

    assert(MatX.rho == MatY.rho, "The density of the material must be isotropic");
    assert(MatX.rho == MatZ.rho, "The density of the material must be isotropic");
    assert(MatX.cm == MatY.cm, "The specific heat capacity of the material must be isotropic");
    assert(MatX.cm == MatZ.cm, "The specific heat capacity of the material must be isotropic");

  initial equation
    if initOpt == Choices.InitOpt.steadyState then
      der(T_vol) = 0;
    elseif initOpt == Choices.InitOpt.fixedState then
      T_vol = Tstart;
    else
      // do nothing
    end if;
    annotation (
      Icon(graphics={   Rectangle(
            extent={{-60,20},{60,-40}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Forward),
          Ellipse(
            extent={{8,-18},{-8,-2}},
            lineColor={0,0,0},
            fillColor={238,46,47},
            fillPattern=FillPattern.Sphere),
          Line(
            points={{100,60},{-20,60}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{100,0},{100,60}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{20,-80},{100,0}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{100,0},{-20,0}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(points={{-100,-20},{-20,60}}, color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{20,-20},{100,60}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{-20,60},{-20,0}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{20,-80},{-100,-80}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{20,-20},{-100,-20}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{-100,-20},{-100,-80}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{20,-20},{20,-80}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{-100,-80},{-20,0}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{-40,-50},{40,30}},
            color={0,0,0},
            pattern=LinePattern.Dash)}),
      Documentation(info="<html>
<p>Extension of the <i>WallConduction </i>model to calculate three-dimensional heat transfer through a planar surface.</p>
<p>The model accounts for anisotropic thermal conductivity in all directions. The remaining material properties must be isotropic.</p>
<p>The model accounts for internal heat generation within the control volume, if any.</p>
<p><br><img src=\"modelica://DynTherM/Figures/Wall Conduction 2D.png\"/></p>
</html>", revisions="<html>
</html>"));
  end WallConductionCV3D;

  model WallConduction3D "Dynamic model of 3D heat conduction in a planar surface"

    replaceable model MatX=Materials.Aluminium constrainedby Materials.Properties
      "Material properties along x direction" annotation (choicesAllMatching=true);
    replaceable model MatY=Materials.Aluminium constrainedby Materials.Properties
      "Material properties along y direction" annotation (choicesAllMatching=true);
    replaceable model MatZ=Materials.Aluminium constrainedby Materials.Properties
      "Material properties along z direction" annotation (choicesAllMatching=true);
    model CV = DynTherM.Components.ThreeDimensional.WallConductionCV3D
      "Control volume";

    // Geometry
    input Length x "Distance between east and west ports" annotation (Dialog(enable=true));
    input Length y "Distance between north and south ports" annotation (Dialog(enable=true));
    input Length z "Distance between front and rear ports" annotation (Dialog(enable=true));

    // Initialization
    parameter Temperature Tstart=288.15 "Temperature start value" annotation (Dialog(tab="Initialization"));
    parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
      "Initialization option" annotation (Dialog(tab="Initialization"));

    // Discretization
    parameter Integer Nx(min=1) "Number of control volumes in x direction";
    parameter Integer Ny(min=1) "Number of control volumes in y direction";
    parameter Integer Nz(min=1) "Number of control volumes in z direction";

    CV cv[Nx,Ny,Nz](
      redeclare model MatX=MatX,
      redeclare model MatY=MatY,
      redeclare model MatZ=MatZ,
      each x=x/Nx,
      each y=y/Ny,
      each z=z/Nz,
      each dm=0,
      each Tstart=Tstart,
      each initOpt=initOpt);

    Mass m "Mass";
    Volume V "Volume";

    CustomInterfaces.ThreeDimensional.HeatPort3D_B South(Nx=Nx, Ny=Nz)
      annotation (Placement(transformation(
          extent={{-20,-10},{20,10}},
          rotation=-90,
          origin={20,3.55271e-15}), iconTransformation(
          extent={{-16,-16},{16,16}},
          rotation=180,
          origin={0,-20})));
    CustomInterfaces.ThreeDimensional.HeatPort3D_A North(Nx=Nx, Ny=Nz)
      annotation (Placement(transformation(
          extent={{-20,-10},{20,10}},
          rotation=-90,
          origin={-20,3.55271e-15}), iconTransformation(
          extent={{-16,-16},{16,16}},
          rotation=180,
          origin={0,20})));
    CustomInterfaces.ThreeDimensional.HeatPort3D_B Rear(Nx=Nx, Ny=Ny)
      annotation (Placement(transformation(extent={{34,44},{66,76}}),
          iconTransformation(extent={{34,44},{66,76}})));
    CustomInterfaces.ThreeDimensional.HeatPort3D_A Front(Nx=Nx, Ny=Ny)
      annotation (Placement(transformation(extent={{-66,-76},{-34,-44}}),
          iconTransformation(extent={{-66,-76},{-34,-44}})));
    CustomInterfaces.ThreeDimensional.HeatPort3D_A West(Nx=Ny, Ny=Nz)
      annotation (Placement(transformation(
          extent={{-20,-10},{20,10}},
          rotation=-90,
          origin={-20,3.55271e-15}), iconTransformation(
          extent={{-16,-16},{16,16}},
          rotation=270,
          origin={-50,0})));
    CustomInterfaces.ThreeDimensional.HeatPort3D_B East(Nx=Ny, Ny=Nz)
      annotation (Placement(transformation(
          extent={{-20,-10},{20,10}},
          rotation=-90,
          origin={20,3.55271e-15}), iconTransformation(
          extent={{-16,-16},{16,16}},
          rotation=270,
          origin={50,0})));

  equation
    m = sum(cv.m);
    V = sum(cv.V);

    for i in 1:Nx loop
      for j in 1:Ny loop
        for k in 1:Nz loop
          cv[i,j,k].Q_int = 0;
        end for;
      end for;
    end for;

    // External connections
    for i in 1:Nx loop
      for j in 1:Ny loop
        connect(Front[i,j], cv[i,j,1].Front);
        connect(Rear[i,j], cv[i,j,end].Rear);
      end for;
    end for;

    for i in 1:Nx loop
      for k in 1:Nz loop
        connect(North.ports[i,k], cv[i,1,k].North);
        connect(South.ports[i,k], cv[i,end,k].South);
      end for;
    end for;

    for j in 1:Ny loop
      for k in 1:Nz loop
        connect(West.ports[j,k], cv[1,j,k].West);
        connect(East.ports[j,k], cv[end,j,k].East);
      end for;
    end for;

    // Internal connections
    for i in 1:Nx loop
      for j in 1:Ny loop
        for k in 1:(Nz - 1) loop
          connect(cv[i,j,k].Rear, cv[i,j,k + 1].Front);
        end for;
      end for;
    end for;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Line(
            points={{0,-80},{-100,-80}},
            color={0,0,0}),
          Line(
            points={{0,-80},{0,-40}},
            color={0,0,0}),
          Line(
            points={{-100,-80},{-100,-40}},
            color={0,0,0}),
          Line(
            points={{0,-40},{-100,-40}},
            color={0,0,0}),
          Line(
            points={{100,40},{0,40}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{100,40},{100,80}},
            color={0,0,0}),
          Line(
            points={{0,40},{0,80}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{100,80},{0,80}},
            color={0,0,0}),
          Line(
            points={{100,40},{0,-80}},
            color={0,0,0}),
          Line(
            points={{100,80},{0,-40}},
            color={0,0,0}),
          Line(
            points={{0,80},{-100,-40}},
            color={0,0,0}),
          Line(
            points={{0,40},{-100,-80}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{50,-20},{-50,-20}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{-50,-20},{-50,20}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{50,-20},{50,20}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{50,20},{-50,20}},
            color={0,0,0},
            pattern=LinePattern.Dash)}),         Diagram(coordinateSystem(
            preserveAspectRatio=false)),
                Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p>Heat transfer is modelled in the transversal, i.e., through the wall thickness, and horizontal, i.e., through adjacent control volumes, directions.</p>
<p>The model accounts for anisotropic thermal conductivity in x and y directions. The remaining material properties must be isotropic.</p>
</html>"));
  end WallConduction3D;
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
          radius=25.0),      Text(
          extent={{-84,88},{92,-88}},
          lineColor={0,0,0},
          textString="3D")}));
end ThreeDimensional;
