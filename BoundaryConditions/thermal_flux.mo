within DynTherM.BoundaryConditions;
model thermal_flux "Model to impose heat flux and temperature"
  extends DynTherM.Base.tfin1;
  input Modelica.Units.SI.Temperature T=273.15 "Temperature" annotation (Dialog(tab="Boundary conditions", enable=use_T));
  input Modelica.Units.SI.HeatFlux phi=1e3 "Heat flux" annotation (Dialog(tab="Boundary conditions", enable=use_Q));
  parameter Boolean use_phi = true "True if heat flux is given"  annotation (Dialog(tab="Boundary conditions"));
  parameter Boolean use_T = false "True if temperature is given" annotation (Dialog(tab="Boundary conditions"));
  parameter Boolean use_in_T = false "Use connector input for the temperature" annotation (Dialog(group="External inputs"), choices(checkBox=true));
  parameter Boolean use_in_phi = false "Use connector input for the heat flux" annotation (Dialog(group="External inputs"), choices(checkBox=true));
  Modelica.Blocks.Interfaces.RealInput in_T if use_in_T annotation (Placement(
        transformation(
        origin={-60,64},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={54,20})));
  Modelica.Blocks.Interfaces.RealInput in_phi if use_in_phi annotation (Placement(
        transformation(
        origin={0,90},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={54,-20})));
protected
  Modelica.Blocks.Interfaces.RealInput in_T0 annotation (Placement(
        transformation(
        origin={-60,64},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,60})));
  Modelica.Blocks.Interfaces.RealInput in_phi0 annotation (Placement(
        transformation(
        origin={0,90},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,0})));
equation
  //Boundary equations
  if use_T then
    thermal_flux.T = T;
  end if;
  if use_phi then
    thermal_flux.phi + phi = 0;
  end if;

  in_T0 = thermal_flux.T;
  in_phi0 + thermal_flux.phi = 0;

  // Connect protected connectors to public conditional connectors
  connect(in_T, in_T0);
  connect(in_phi, in_phi0);
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{60,-20},
            {120,20}})),                  Icon(coordinateSystem(
          preserveAspectRatio=false, extent={{60,-20},{120,20}}),     graphics={Text(
          extent={{66,8},{82,-8}},
          lineColor={0,0,0},
          textString="AT")}));
end thermal_flux;
