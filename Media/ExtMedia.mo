within DynTherM.Media;
package ExtMedia
  package CoolProp
    package Hydrogen "Hydrogen (CoolProp model)"
      extends ExternalMedia.Media.CoolPropMedium(
        mediumName="Hydrogen",
        substanceNames={"H2"},
        ThermoStates=Modelica.Media.Interfaces.Choices.IndependentVariables.ph,
        AbsolutePressure(start=1e6),
        SpecificEnthalpy(start=1e6));
    end Hydrogen;
  end CoolProp;

  package FluidProp
  end FluidProp;

  package RefProp
  end RefProp;
end ExtMedia;
