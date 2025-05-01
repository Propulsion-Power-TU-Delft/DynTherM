within DynTherM.Base;
partial model tfin1 "One thermal flux inlet"
  CustomInterfaces.ZeroDimensional.HeatFluxPort_A thermal_flux
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics));
end tfin1;
