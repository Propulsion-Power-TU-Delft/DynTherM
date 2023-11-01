within DynTherM.CustomInterfaces;
connector DistributedIrradiancePort
  "Distributed irradiance connector"
  connector IrradiancePort = CustomInterfaces.IrradiancePort;
  parameter Integer Nx(min=1) "Number of ports in x-direction";
  parameter Integer Ny(min=1) "Number of ports in y-direction";
  IrradiancePort ports[Nx,Ny];
  annotation (Icon(graphics={
                         Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={238,46,47},
          fillColor={255,127,0},
          fillPattern=FillPattern.Solid), Ellipse(
          extent={{-60,16},{60,-16}},
          lineColor={191,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end DistributedIrradiancePort;
